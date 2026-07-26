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
using SynchToolkit: ClockedInput, ClockedOutput, InputClock, ParametersStruct
import SynchCompiler
import SynchJulia
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using DiscreteComponents: PeriodicClock
using OrderedCollections: OrderedDict
using ControlSystemsMTK: named_ss
using ControlSystemsBase: c2d, ss, lqr
using LinearAlgebra: Diagonal, I, pinv

export generate_swingup_controller, export_swingup_c, SwingupController, design_lqr

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
    generate_swingup_controller(; Ts=0.005, L=nothing, umax=nothing, overrides...)

Compile the discrete swing-up controller to a SynchJulia node: build a purely discrete
`System` wrapping the `Swingup` controller on a `PeriodicClock` at sample time `Ts` (the
equation `clock.y ~ swingup.shoulder_angle` places the whole controller partition on the
clock while leaving the two measured angles as free input signals), then `stkcompile` it.

Returns a named tuple `(; topmod, Ldef, umaxdef)` where `topmod` is the evaluated runtime
module (with `topmod.top`, `topmod.StaticGains`, `topmod.AutoPars`, `topmod.executable`)
and `Ldef`/`umaxdef` are the resolved stabilizer gain/saturation values used to populate
the runtime-settable `StaticGains` struct.

The node argument order is `(shoulder_angle::Float64, elbow_angle::Float64,
tick::Bool, gains::StaticGains, auto::AutoPars)`; the LQR gains `L` and the
stabilizer `umax` are the runtime-settable `StaticGains` fields, the rest are
baked into `AutoPars`.

The controller uses the `Swingup` model's tuned defaults (energy-swingup gain,
arm-centering, LQR gains and saturations), which are set for the QuanserInterface-
matched `QubePendulum` plant. Pass `overrides` (e.g. `var"lqrstabilizer.umax" =>
...`) to change them; `L` (a 4-vector) and `umax` override the stabilizer feedback
gain and saturation directly via the wrapper system's `initial_conditions`.
"""
function generate_swingup_controller(; Ts = 0.005, L = nothing, umax = nothing, overrides...)
    @named swingup = SwingupWithHoming(; overrides...)
    @named clock = PeriodicClock(dt = Ts)
    lqr_stab = swingup.runtime.swingup.lqrstabilizer
    # Dyad parameter values live in `initial_conditions` (not `getdefault`), so override
    # the stabilizer gain/saturation there: this feeds the resolved `Ldef`/`umaxdef`
    # below and the value resolution SynchToolkit performs for the `AutoPars` struct.
    ics = Dict{Any, Any}()
    L    === nothing || (ics[lqr_stab.L]    = collect(float.(L)))
    umax === nothing || (ics[lqr_stab.umax] = float(umax))
    sys = System([clock.y ~ swingup.shoulder_angle], t;
                 systems = [swingup, clock], name = :controller, initial_conditions = ics)

    # Resolve the two stabilizer values carried by the runtime-settable `StaticGains`
    # struct. All other parameters end up in the autogenerated `AutoPars` struct, whose
    # field defaults SynchToolkit resolves from the system itself; only static structs
    # carry no defaults, so these two are resolved here with `evaluate_varmap!`, which
    # substitutes the default map into itself to a fixed point (values may be expressions
    # of other parameters). Resolve against the full pre-partition default map: expression
    # defaults may reference parameters the compiled partition drops (SynchToolkit#144).
    SymT = ModelingToolkit.SymbolicT
    dv = Dict{SymT, SymT}(ModelingToolkit.default_values(ModelingToolkit.expand_connections(sys)))
    Lsym, usym = ModelingToolkit.unwrap(lqr_stab.L), ModelingToolkit.unwrap(lqr_stab.umax)
    ModelingToolkit.evaluate_varmap!(dv, [Lsym, usym])
    Ldef    = Float64.(vec(collect(Symbolics.value(dv[Lsym]))))
    umaxdef = Float64(Symbolics.value(dv[usym]))

    gain_syms = OrderedDict{Any, Symbol}(Lsym => :L, usym => :umax)
    inputs = [
        ClockedInput(swingup.shoulder_angle),
        ClockedInput(swingup.elbow_angle),
        InputClock(ModelingToolkit.Clock(Ts)),
        ParametersStruct(; arg_name = :gains, struct_name = :StaticGains,
                           parameters = gain_syms, generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
    ]
    outputs = [ClockedOutput(swingup.u)]
    topmod = SynchToolkit.stkcompile(sys; inputs, outputs)
    return (; topmod, Ldef, umaxdef)
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

# Consume the compiled runtime module: construct the `StaticGains`/`AutoPars` parameter
# objects (applying the LQR-gain / saturation overrides `L`/`umax`; `nothing` keeps the
# model default), optionally build a `SynchExecutable` on `backend`, and optionally export
# the C sources into `export_dir`. These objects are what the runtime hands to the node's
# `step` (as opaque pointers in the C backend); their in-memory field bytes are exactly
# what the exported C reads at its baked-in `fieldoffset`s.
#
# `stkcompile` evaluates the runtime module in a newer world than this frame, so per
# SynchToolkit's consumption contract the boundary is crossed exactly once, here: the
# single `@invokelatest` around `consume` raises the world for all module member reads and
# constructor calls, and everything returned (values, types, the executable) is world-safe
# to use from any frame afterwards.
function _instantiate(gen; L = nothing, umax = nothing,
                      backend::Union{Nothing, Symbol} = nothing, export_dir = nothing)
    Lv = collect(float.(something(L, gen.Ldef)))
    umaxv = float(something(umax, gen.umaxdef))
    function consume(m)
        gains = m.StaticGains(; L = Lv, umax = umaxv)
        # pass the static struct: AutoPars defaults may be expressions of its fields
        auto = m.AutoPars(gains)
        exe = backend === nothing ? nothing : m.executable(backend)
        export_dir === nothing || SynchCompiler.export_c(export_dir, m.top)
        return (; gains, auto, exe, SG = m.StaticGains, AP = m.AutoPars)
    end
    return Base.@invokelatest consume(gen.topmod)
end

# Build a runtime from an already-compiled controller (avoids recompiling).
function _make_runtime(gen; backend::Symbol = :julia, L = nothing, umax = nothing)
    r = _instantiate(gen; L, umax, backend)
    return SwingupController(r.exe, r.gains, r.auto)
end

"""
    (c::SwingupController)(shoulder_angle, elbow_angle; tick=true) -> u

Advance the controller one step and return the control voltage. Angles are in
radians (shoulder/arm angle and pendulum angle; the pendulum-upright reference is
π, matching `QuanserInterface`).
"""
# A held executable keeps stepping the code it was built with (SynchJulia ≥ 0.4), so
# `step!`/`reset!` are world-safe from any frame — no `invokelatest` in the hot path.
function (c::SwingupController)(shoulder_angle, elbow_angle; tick::Bool = true)
    out = SynchJulia.step!(c.exe, Float64(shoulder_angle), Float64(elbow_angle),
                           tick, c.gains, c.auto)
    return only(values(out))
end

SynchToolkit.reset!(c::SwingupController) = SynchToolkit.reset!(c.exe)

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
    r = _instantiate(gen; L, umax, export_dir = dir)
    mangled = SynchCompiler.mangle("top", Float64, Float64, Bool, r.SG, r.AP)
    emit_hardware_harness(dir; Ts, Tf, mangled, r.gains, r.auto)
    return (; dir, gen.topmod, mangled, r.gains, r.auto)
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

It writes a tab-separated `run_hardware.csv` matching `test/hardware_swingup.jl`'s
`swingup.csv` layout — columns `time, shoulder_angle, elbow_angle, control_input` (load with
`D = readdlm("run_hardware.csv", skipstart=1)'` and pass to `plotD`) plus two timing
diagnostics `dt` (achieved period) and `exec` (loop-body duration). Timing mirrors the Julia
`@periodically` loop (run body, sleep the remainder of `Ts`); a one-line timing summary is
printed to stderr on exit.
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

    /* elapsed seconds a - b for two CLOCK_MONOTONIC timestamps */
    static double tsub(struct timespec a, struct timespec b) {
        return (double)(a.tv_sec - b.tv_sec) + (double)(a.tv_nsec - b.tv_nsec) * 1e-9;
    }

    int main(void) {
        t_card  board;
        t_error result;
        const t_uint32 encoder_channels[2] = {0u, 1u};   /* 0 = arm/shoulder, 1 = pendulum/elbow */
        const t_uint32 analog_channel      = 0u;         /* motor voltage */
        const t_uint32 digital_channel     = 0u;         /* amplifier enable */
        t_int32   counts[2]  = {0, 0};
        t_int32   counts0[2] = {0, 0};   /* initial counts, subtracted as homing offsets */
        t_double  voltage    = 0.0;
        t_boolean enable     = 1;

        signal(SIGINT, on_signal);
        signal(SIGTERM, on_signal);

        result = hil_open(board_type, board_identifier, &board);
        if (result < 0) {
            char msg[512];
            msg_get_error_message(NULL, result, msg, sizeof(msg));
            fprintf(stderr, "hil_open failed (%d): %s\\n", (int)result, msg);
            return 1;
        }

        /* Home in software, exactly like QuanserInterface: the encoder counters are left
         * untouched (hil_set_encoder_counts is deliberately NOT called — zeroing the
         * counters has been observed to desync the driver's counter extension, producing
         * spurious 2^16-count jumps) and the initial counts are subtracted as offsets.
         * Before starting, place the arm at its home position and let the pendulum hang
         * straight down (0 = down, pi = up). The controller's GoHome drives the arm. */
        voltage = 0.0;
        hil_write_analog(board, &analog_channel, 1, &voltage);
        hil_write_digital(board, &digital_channel, 1, &enable);
        hil_read_encoder(board, encoder_channels, 2, counts0);

        $(mangled)_mem state;
        memset(&state, 0, sizeof(state));
        $(mangled)_reset(&state);

        /* Tab-separated log matching test/hardware_swingup.jl's `swingup.csv` layout
         * (writedlm): first four columns are what `plotD` expects (load with
         * `D = readdlm("run_hardware.csv", skipstart=1)'`). Two extra diagnostic columns:
         * `dt` = achieved period since the previous step, `exec` = body (read+step+write)
         * duration — both in seconds, for spotting timing trouble — plus the raw encoder
         * counts, for diagnosing counter glitches (e.g. spurious 2^16 jumps). */
        FILE *logf = fopen("run_hardware.csv", "w");
        if (logf) fprintf(logf, "time\\tshoulder_angle\\telbow_angle\\tcontrol_input\\tdt\\texec\\tcount_shoulder\\tcount_elbow\\n");

        /* Timing mirrors the Julia @periodically loop: run the body, then sleep the
         * remainder of Ts (relative sleep, no absolute-schedule catch-up — so one slow
         * step just stretches that period instead of compressing the following ones). */
        const long N = (long)(Tf / Ts);
        struct timespec t0, start, done;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        struct timespec prev = t0;
        double sum_dt = 0.0, max_dt = 0.0, max_exec = 0.0;
        long periods = 0;

        for (long i = 0; i < N && !stop_flag; ++i) {
            clock_gettime(CLOCK_MONOTONIC, &start);
            double t  = tsub(start, t0);      /* elapsed at step start [s] */
            double dt = tsub(start, prev);    /* achieved period since previous step [s] */
            prev = start;

            hil_read_encoder(board, encoder_channels, 2, counts);
            double shoulder_angle = (counts[0] - counts0[0]) * COUNTS2RAD;
            double elbow_angle    = (counts[1] - counts0[1]) * COUNTS2RAD;

            $(mangled)_out out =
                $(mangled)_step(shoulder_angle, elbow_angle, true, GAINS_PTR, AUTO_PTR, &state);
            voltage = out.swingup_u_t_;
            if (voltage >  VLIM) voltage =  VLIM;
            if (voltage < -VLIM) voltage = -VLIM;
            hil_write_analog(board, &analog_channel, 1, &voltage);

            clock_gettime(CLOCK_MONOTONIC, &done);
            double exec = tsub(done, start);  /* body duration [s] */

            if (logf) fprintf(logf, "%.6f\\t%.6f\\t%.6f\\t%.6f\\t%.6f\\t%.6f\\t%ld\\t%ld\\n",
                              t, shoulder_angle, elbow_angle, voltage, dt, exec,
                              (long)counts[0], (long)counts[1]);

            if (i > 0) { sum_dt += dt; if (dt > max_dt) max_dt = dt; periods++; }
            if (exec > max_exec) max_exec = exec;

            double remain = Ts - exec;
            if (remain > 0.0) {
                struct timespec ts;
                ts.tv_sec  = (time_t)remain;
                ts.tv_nsec = (long)((remain - (double)ts.tv_sec) * 1e9);
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);   /* relative sleep */
            }
        }

        fprintf(stderr, "run_hardware: Ts=%.4f s | mean dt=%.4f s, max dt=%.4f s, "
                        "max exec=%.4f s over %ld periods\\n",
                Ts, periods > 0 ? sum_dt / (double)periods : 0.0, max_dt, max_exec, periods);

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
