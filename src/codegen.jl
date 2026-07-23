# Code generation for the discrete swing-up controller.
#
# The `Swingup` controller in `FurutaSwingup` is a purely discrete (clocked)
# subsystem: algebraic blocks plus discrete-time velocity estimators
# (`DiscreteDerivative` + `ExponentialFilter`). SynchToolkit's `stkcompile` can
# compile such a subsystem into a standalone synchronous node and emit Julia- or
# C-executable code for it.
#
# The generated node has the runtime signature
#     u = step(shoulder_angle, elbow_angle, tick, gains, auto)
# where `tick` is the periodic-clock trigger (`Bool`), `gains` carries the
# tunable LQR gains `L1..L4` and the stabilizer saturation `umax`, and `auto`
# carries the remaining model parameters (resolved to their model values). The
# single output is the control voltage.

using SynchToolkit
using SynchToolkit: ClockedInput, ClockedOutput, InputClock, ParametersStruct,
                    SynchExecutable
import SynchCompiler
import SynchJulia
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using DiscreteComponents: PeriodicClock
using OrderedCollections: OrderedDict
using ControlSystemsMTK: named_ss
using ControlSystemsBase: c2d, ss, lqr
using LinearAlgebra: Diagonal, I, pinv

export build_discrete_controller, generate_swingup_controller, export_swingup_c,
       SwingupController, design_lqr

"""
    build_discrete_controller(; Ts=0.005, overrides...)

Build a purely discrete `System` wrapping the `Swingup` controller, driven by a
`PeriodicClock` at sample time `Ts`. The measured angles enter as clocked input
variables and the control voltage leaves as a clocked output.

The clock is linked to the controller through the equation
`clock.y ~ swingup.shoulder_angle`, which places the whole controller partition on
the periodic clock while leaving the two measured angles as free (input) signals.

Returns a named tuple with the `sys`, the `swingup` and `clock` subsystems, the
`Clock` object `clk`, and the resolved parameter value map `values` (parameter =>
`Float64`).

The controller uses the `Swingup` model's tuned defaults (energy-swingup gain,
arm-centering, LQR gains and saturations), which are set for the QuanserInterface-
matched `QubePendulum` plant. Pass `overrides` (e.g. `var"lqrstabilizer.umax" =>
...`) to change them.

`L` (a 4-vector) and `umax` override the stabilizer feedback gain and saturation
directly: they are written into the wrapper system's `defaults`, so both the resolved
`values` map and the code-generated `StaticGains` struct defaults reflect them.
"""
function build_discrete_controller(; Ts = 0.005, L = nothing, umax = nothing, overrides...)
    @named swingup = SwingupWithHoming(; overrides...)
    @named clock = PeriodicClock(dt = Ts)
    lqr_stab = swingup.runtime.swingup.lqrstabilizer
    # Dyad parameter values live in `initial_conditions` (not `getdefault`), so override
    # the stabilizer gain/saturation there: this feeds both the resolved `values` map
    # below and SynchToolkit's `StaticGains` struct defaults.
    ics = Dict{Any, Any}()
    L    === nothing || (ics[lqr_stab.L]    = collect(float.(L)))
    umax === nothing || (ics[lqr_stab.umax] = float(umax))
    sys = System([clock.y ~ swingup.shoulder_angle], t;
                 systems = [swingup, clock], name = :controller, initial_conditions = ics)

    # Resolve every parameter to a concrete number. Dyad components store values in
    # `default_values` (as `initial_conditions`), and some values are expressions of
    # other parameters (e.g. `energy.l => Lp/2`), so substitute to a fixed point.
    ss = ModelingToolkit.expand_connections(sys)
    dv = ModelingToolkit.default_values(ss)
    values = Dict{Any, Any}()
    for p in ModelingToolkit.parameters(ss)
        pu = ModelingToolkit.unwrap(p)
        values[pu] = _resolve_value(dv[pu], dv)
    end

    return (; sys, swingup, clock, clk = ModelingToolkit.Clock(Ts), values)
end

# Resolve a possibly-symbolic default to a concrete number (or vector of numbers for
# array parameters), substituting the default map to a fixed point.
function _resolve_value(v, dv)
    x = ModelingToolkit.unwrap(v)
    for _ in 1:20
        (x isa Number || x isa AbstractArray{<:Number}) && break
        x = ModelingToolkit.unwrap(ModelingToolkit.fixpoint_sub(x, dv))
    end
    xv = Symbolics.value(x)
    return xv isa AbstractArray ? Float64.(vec(collect(xv))) : Float64(xv)
end

"""
    design_lqr(; Ts=0.005, Q1=[1000.0, 10.0, 1.0, 1.0], Q2=100.0) -> L::Vector{Float64}

Design the LQR state-feedback gain `L` for the `LQRstabilizer`. The `FurutaSwingup`
plant is linearized about the upright equilibrium (with the controller loop opened at
the `u_plant`/`shoulder_y`/`elbow_y` analysis points), discretized at sample time `Ts`,
and an LQR problem is solved with state penalty `Q1` and control penalty `Q2`.

`Q1` is the diagonal of the state cost in the order `[shoulder_angle, elbow_angle,
shoulder_velocity, elbow_velocity]`; `Q2` is the scalar control cost. The returned `L`
is the 4-element gain expected by `LQRstabilizer.L`.
"""
function design_lqr(; Ts = 0.005, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 100.0)
    @named model = FurutaSwingup()
    ssys = ModelingToolkit.toggle_namespacing(model, false)
    op = Dict(
        ssys.qubependulum.elbow_joint.phi    => pi,
        ssys.qubependulum.shoulder_joint.phi => 0.0,
        ssys.qubependulum.elbow_joint.w      => 0.0,
        ssys.qubependulum.shoulder_joint.w   => 0.0,
        ssys.qubependulum.voltage            => 0.0,
        ssys.elbow_sampler.u    => 0.0,
        ssys.shoulder_sampler.u => 0.0,
    )
    # Outputs define the order of the `Q1` diagonal below.
    outputs = [
        ssys.qubependulum.shoulder_angle,
        ssys.qubependulum.elbow_angle,
        ssys.qubependulum.shoulder_joint.w,
        ssys.qubependulum.elbow_joint.w,
    ]
    P = named_ss(model, [ssys.u_plant], outputs;
        op,
        loop_openings = [ssys.u_plant, ssys.shoulder_y, ssys.elbow_y],
        warn_empty_op = true,
        additional_passes = [SynchToolkit.compile_lustre],
        MultibodyComponents.linsys...,
    )
    Pd = c2d(ss(P), Ts)
    Q1mat = P.C' * Diagonal(collect(float.(Q1))) * P.C
    Q2mat = float(Q2) * I(1)
    return vec(lqr(Pd, Q1mat, Q2mat) * pinv(P.C))
end

"""
    generate_swingup_controller(; Ts=0.005, overrides...)

Compile the discrete swing-up controller to a SynchJulia node. Returns a named
tuple `(; topmod, controller, gain_syms)` where `topmod` is the evaluated module
(with `topmod.top`, `topmod.StaticGains`, `topmod.AutoPars`) and `controller` is
the [`build_discrete_controller`](@ref) result.

The node argument order is `(shoulder_angle::Float64, elbow_angle::Float64,
tick::Bool, gains::StaticGains, auto::AutoPars)`; the LQR gains `L` and the
stabilizer `umax` are the runtime-settable `StaticGains` fields, the rest are
baked into `AutoPars`.
"""
function generate_swingup_controller(; Ts = 0.005, kwargs...)
    c = build_discrete_controller(; Ts, kwargs...)
    lqr = c.swingup.runtime.swingup.lqrstabilizer
    gain_syms = OrderedDict{Any, Symbol}(
        ModelingToolkit.unwrap(lqr.L)   => :L,
        ModelingToolkit.unwrap(lqr.umax) => :umax,
    )
    inputs = [
        ClockedInput(c.swingup.shoulder_angle),
        ClockedInput(c.swingup.elbow_angle),
        InputClock(c.clk),
        ParametersStruct(; arg_name = :gains, struct_name = :StaticGains,
                           parameters = gain_syms, generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
    ]
    outputs = [ClockedOutput(c.swingup.u)]
    topmod = SynchToolkit.stkcompile(c.sys; inputs, outputs)
    return (; topmod, controller = c, gain_syms)
end

"""
    SwingupController(; Ts=0.005, backend=:julia, L=nothing, umax=nothing, overrides...)

A ready-to-call runtime wrapper around the generated swing-up controller. Compiles
the controller, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and
populates the parameter structs. Call it as `u = controller(shoulder, elbow)` to
advance one control step (ticking the periodic clock) and obtain the voltage.

`L` overrides the LQR feedback gains `[L1, L2, L3, L4]` and `umax` the stabilizer
saturation; both default to the model's tuned values.
"""
struct SwingupController{E, G, A}
    exe::E
    gains::G
    auto::A
end

function SwingupController(; Ts = 0.005, backend::Symbol = :julia,
                            L = nothing, umax = nothing, kwargs...)
    gen = generate_swingup_controller(; Ts, kwargs...)
    return _make_runtime(gen; backend, L, umax)
end

# Construct the concrete `StaticGains`/`AutoPars` parameter objects for the compiled
# controller `gen`, applying the LQR-gain / saturation overrides `L`/`umax` (each `nothing`
# keeps the model default). These are the objects the runtime hands to the node's `step`
# (as opaque pointers in the C backend); their in-memory field bytes are exactly what the
# exported C reads at its baked-in `fieldoffset`s. `stkcompile` evaluates a fresh module,
# so the generated `StaticGains`/`AutoPars` types may be newer than the current world age;
# reach them through `invokelatest` so a one-shot `compile + build` in a single call works.
function _build_params(gen; L = nothing, umax = nothing)
    topmod = gen.topmod
    SG = Base.invokelatest(getproperty, topmod, :StaticGains)
    AP = Base.invokelatest(getproperty, topmod, :AutoPars)
    vals = gen.controller.values

    # AutoPars fields are named after the (namespaced) parameter symbols.
    sym_by_name = Dict(Symbol(string(p)) => ModelingToolkit.unwrap(p)
                       for p in keys(vals))
    apfields = Base.invokelatest(fieldnames, AP)
    apkw = Dict(f => vals[sym_by_name[f]] for f in apfields if haskey(sym_by_name, f))
    auto = Base.invokelatest(AP; apkw...)

    lqr = gen.controller.swingup.runtime.swingup.lqrstabilizer
    Ldef = vals[ModelingToolkit.unwrap(lqr.L)]                # model default (4-vector)
    Lv = L === nothing ? Ldef : collect(float.(L))
    umaxv = umax === nothing ? vals[ModelingToolkit.unwrap(lqr.umax)] : umax
    gains = Base.invokelatest(SG; L = Lv, umax = umaxv)
    return (; gains, auto, umax = umaxv)
end

# Build a runtime from an already-compiled controller (avoids recompiling). See
# `_build_params` for the `invokelatest` rationale; `top` is reached the same way.
function _make_runtime(gen; backend::Symbol = :julia, L = nothing, umax = nothing)
    node = Base.invokelatest(getproperty, gen.topmod, :top)
    (; gains, auto) = _build_params(gen; L, umax)
    exe = Base.invokelatest(SynchExecutable, node,
                            (Float64, Float64, Bool, typeof(gains), typeof(auto)); backend)
    return SwingupController(exe, gains, auto)
end

"""
    (c::SwingupController)(shoulder_angle, elbow_angle; tick=true) -> u

Advance the controller one step and return the control voltage. Angles are in
radians (shoulder/arm angle and pendulum angle; the pendulum-upright reference is
π, matching `QuanserInterface`).
"""
# The executable's step/reset dispatch on types generated by `stkcompile`'s `eval`,
# so route through `invokelatest` to stay correct even when a controller is compiled
# and used within the same world (e.g. one test block). The overhead is negligible
# for a control step, and the C backend (the performance path) does not run Julia here.
function (c::SwingupController)(shoulder_angle, elbow_angle; tick::Bool = true)
    out = Base.invokelatest(SynchJulia.step!, c.exe, Float64(shoulder_angle),
                            Float64(elbow_angle), tick, c.gains, c.auto)
    return only(values(out))
end

SynchToolkit.reset!(c::SwingupController) = Base.invokelatest(SynchToolkit.reset!, c.exe)

"""
    export_swingup_c(dir; Ts=0.005, L=nothing, umax=nothing, Tf=10.0, overrides...)

Generate the swing-up controller and export standalone C sources into `dir`: the
SynchToolkit node (`top.c`, `top.h`, `top.pc`, `synchjulia.h`) plus a runnable hardware
control loop (`run_hardware.c`, `Makefile`) that drives the Quanser QUBE-Servo pendulum
via the HIL SDK (see [`emit_hardware_harness`](@ref)). `L`/`umax` override the LQR gains
and stabilizer saturation; `Tf` is the run duration baked into the harness.

Returns `(; dir, topmod, mangled, gains, auto)` where `mangled` is the base symbol name
for the emitted `<mangled>_step` / `<mangled>_reset` functions and `gains`/`auto` are the
parameter objects embedded into the harness.
"""
function export_swingup_c(dir; Ts = 0.005, L = nothing, umax = nothing, Tf = 10.0, kwargs...)
    gen = generate_swingup_controller(; Ts, L, umax, kwargs...)
    node = Base.invokelatest(getproperty, gen.topmod, :top)
    SG = Base.invokelatest(getproperty, gen.topmod, :StaticGains)
    AP = Base.invokelatest(getproperty, gen.topmod, :AutoPars)
    Base.invokelatest(SynchCompiler.export_c, dir, node)
    mangled = SynchCompiler.mangle("top", Float64, Float64, Bool, SG, AP)
    (; gains, auto) = _build_params(gen; L, umax)
    emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto)
    return (; dir, gen.topmod, mangled, gains, auto)
end

# Raw little-endian bytes of a runtime parameter object (`StaticGains`/`AutoPars`), packed
# into 64-bit words. Both are mutable structs whose fields are stored inline, so
# `pointer_from_objref` points at the flat field-data region the exported C reads via its
# baked-in `fieldoffset`s. Packing to `uint64_t` guarantees the 8-byte alignment the C
# pointer casts require (the data region is a whole number of 8-byte fields; pad defensively).
function _param_words(obj)
    n = sizeof(typeof(obj))
    npad = cld(n, 8) * 8
    bytes = zeros(UInt8, npad)
    GC.@preserve obj bytes begin
        unsafe_copyto!(pointer(bytes), Ptr{UInt8}(pointer_from_objref(obj)), n)
    end
    return reinterpret(UInt64, bytes)
end

"""
    emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto)

Write a standalone C hardware control loop (`run_hardware.c`) plus a `Makefile` into `dir`,
next to the exported `top.c`/`top.h`. The loop mirrors `test/hardware_swingup.jl`: it opens
the Quanser QUBE-Servo pendulum via the HIL SDK, zeroes the encoders (home the arm and let
the pendulum hang before starting), then every `Ts` seconds reads the two encoder angles,
calls the generated `<mangled>_step`, and writes the returned voltage to the motor (clamped
to the ±10 V hardware limit). The `gains`/`auto` parameter objects are serialized to raw
bytes and embedded so `_step` sees the exact byte layout it was compiled against.
"""
function emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto)
    words(obj) = join(("0x" * string(w; base = 16, pad = 16) * "ULL" for w in _param_words(obj)), ", ")
    c_src = """
    /* Auto-generated by QuanserComponents.export_swingup_c — do not edit by hand.
     *
     * Standalone hardware control loop for the exported swing-up controller, mirroring
     * test/hardware_swingup.jl: measure -> step -> actuate at a fixed rate, talking to the
     * Quanser QUBE-Servo pendulum directly through the HIL SDK. Build with `make`. */
    #define _GNU_SOURCE
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <time.h>
    #include <signal.h>
    #include <stdint.h>

    #include "hil.h"
    #include "quanser_messages.h"
    #include "top.h"

    /* Parameter blocks that the generated `_step` reads as opaque pointers: the raw bytes
     * of the Julia StaticGains (LQR gains L[4] + stabilizer saturation umax) and AutoPars
     * (all other model constants), serialized so the byte offsets match top.c exactly. */
    static const uint64_t gains_words[] = { $(words(gains)) };
    static const uint64_t auto_words[]  = { $(words(auto)) };
    #define GAINS_PTR ((int64_t)(intptr_t)gains_words)
    #define AUTO_PTR  ((int64_t)(intptr_t)auto_words)

    static const char   board_type[]       = "qube_servo3_usb";
    static const char   board_identifier[] = "0";
    static const double Ts    = $(Float64(Ts));          /* sample time [s]  */
    static const double Tf    = $(Float64(Tf));          /* run duration [s] */
    static const double VLIM  = 10.0;                    /* motor voltage hardware limit [V] */
    static const double COUNTS2RAD = 2.0 * 3.14159265358979323846 / 2048.0;

    static volatile sig_atomic_t stop_flag = 0;
    static void on_signal(int sig) { (void)sig; stop_flag = 1; }

    int main(void) {
        t_card  board;
        t_error result;
        const t_uint32 encoder_channels[2] = {0u, 1u};   /* 0 = arm/shoulder, 1 = pendulum/elbow */
        const t_uint32 analog_channel      = 0u;         /* motor voltage */
        const t_uint32 digital_channel     = 0u;         /* amplifier enable */
        const t_int32  zero_counts[2]      = {0, 0};
        t_int32   counts[2] = {0, 0};
        t_double  voltage   = 0.0;
        t_boolean enable    = 1;

        signal(SIGINT, on_signal);
        signal(SIGTERM, on_signal);

        result = hil_open(board_type, board_identifier, &board);
        if (result < 0) {
            char msg[512];
            msg_get_error_message(NULL, result, msg, sizeof(msg));
            fprintf(stderr, "hil_open failed (%d): %s\\n", (int)result, msg);
            return 1;
        }

        /* Home by zeroing the encoders: place the arm at its home position and the pendulum
         * hanging straight down before starting. The controller's GoHome state then drives
         * the arm; the pendulum reference is elbow = 0 down, pi up. */
        hil_set_encoder_counts(board, encoder_channels, 2, zero_counts);
        voltage = 0.0;
        hil_write_analog(board, &analog_channel, 1, &voltage);
        hil_write_digital(board, &digital_channel, 1, &enable);

        $(mangled)_mem state;
        memset(&state, 0, sizeof(state));
        $(mangled)_reset(&state);

        FILE *logf = fopen("run_hardware.csv", "w");
        if (logf) fprintf(logf, "time,shoulder_angle,elbow_angle,control_input\\n");

        const long N = (long)(Tf / Ts);
        const long period_ns = (long)(Ts * 1e9);
        struct timespec t0, next;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        next = t0;

        for (long i = 0; i < N && !stop_flag; ++i) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double t = (double)(now.tv_sec - t0.tv_sec) + (double)(now.tv_nsec - t0.tv_nsec) * 1e-9;

            hil_read_encoder(board, encoder_channels, 2, counts);
            double shoulder_angle = counts[0] * COUNTS2RAD;
            double elbow_angle    = counts[1] * COUNTS2RAD;

            $(mangled)_out out =
                $(mangled)_step(shoulder_angle, elbow_angle, true, GAINS_PTR, AUTO_PTR, &state);
            voltage = out.swingup_u_t_;
            if (voltage >  VLIM) voltage =  VLIM;
            if (voltage < -VLIM) voltage = -VLIM;
            hil_write_analog(board, &analog_channel, 1, &voltage);

            if (logf) fprintf(logf, "%.6f,%.6f,%.6f,%.6f\\n",
                              t, shoulder_angle, elbow_angle, voltage);

            next.tv_nsec += period_ns;
            while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        }

        /* Stop the motor and release the board. */
        voltage = 0.0;
        hil_write_analog(board, &analog_channel, 1, &voltage);
        hil_close(board);
        if (logf) fclose(logf);
        return 0;
    }
    """
    makefile = """
    # Auto-generated by QuanserComponents.export_swingup_c.
    # Build the hardware control loop for the exported swing-up controller: `make`.
    QUANSER_DIR ?= /opt/quanser/hil_sdk
    CC          ?= cc
    CFLAGS      += -I. -I\$(QUANSER_DIR)/include -O2 -Wall
    LDFLAGS     += -L\$(QUANSER_DIR)/lib
    LIBS        += -lhil -lquanser_runtime -lquanser_common -lrt -lpthread -ldl -lm -lc

    run_hardware: run_hardware.c top.c top.h synchjulia.h
    \t\$(CC) \$(CFLAGS) run_hardware.c top.c -o \$@ \$(LDFLAGS) \$(LIBS)

    .PHONY: clean
    clean:
    \trm -f run_hardware run_hardware.csv
    """
    write(joinpath(dir, "run_hardware.c"), c_src)
    write(joinpath(dir, "Makefile"), makefile)
    return (; c = "run_hardware.c", makefile = "Makefile")
end

"""
    compile_hardware_harness(dir; quanser_dir="/opt/quanser/hil_sdk") -> exe_path

Compile the emitted `run_hardware.c` + `top.c` in `dir` into a `run_hardware` executable,
linking the Quanser HIL SDK. Uses the system C compiler (`cc`, falling back to `gcc`).
Linking uses the static SDK libraries, so this does not require the hardware to be
connected — it doubles as a build check for the generated sources.
"""
function compile_hardware_harness(dir; quanser_dir = "/opt/quanser/hil_sdk")
    cc = Sys.which("cc")
    cc === nothing && (cc = Sys.which("gcc"))
    cc === nothing && error("compile_hardware_harness: no C compiler (cc/gcc) found on PATH")
    exe = abspath(joinpath(dir, "run_hardware"))
    inc = joinpath(quanser_dir, "include")
    lib = joinpath(quanser_dir, "lib")
    args = [cc, "-I$dir", "-I$inc", "-O2", "-Wall",
            joinpath(dir, "run_hardware.c"), joinpath(dir, "top.c"), "-o", exe,
            "-L$lib", "-lhil", "-lquanser_runtime", "-lquanser_common",
            "-lrt", "-lpthread", "-ldl", "-lm", "-lc"]
    run(Cmd(args))
    return exe
end

"""
    run_hardware_harness(exe) -> csv_path

Execute the compiled `run_hardware` binary, blocking for its baked-in run duration `Tf`.
The binary controls the physical pendulum and writes `run_hardware.csv` (columns `time,
shoulder_angle, elbow_angle, control_input`) in its working directory; that path is returned.
"""
function run_hardware_harness(exe)
    exe = abspath(exe)
    dir = dirname(exe)
    # Run with the child's working directory set to `dir` (not the current process's) so
    # the binary — addressed by absolute path — is found and `run_hardware.csv` lands in `dir`.
    run(Cmd(`$exe`; dir = dir))
    return joinpath(dir, "run_hardware.csv")
end
