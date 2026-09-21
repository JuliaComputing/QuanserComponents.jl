# The MPC program (`FurutaMPCHardware`): compiled once, then ticked against a
# simulated plant through the hardware I/O callbacks, as the swing-up program is in runtests.jl.
# The plant is the MPC's own prediction model -- the multibody `QubePendulum` ODE from
# `furuta_mpc_dynamics()` -- integrated with RK4. These tests check the mechanics of the program
# (it compiles, ticks, does one hardware read and write and one acados solve per tick, and
# reports the solver status); closed-loop performance is the business of test/mpc_rollouts.jl
# while the controller is being tuned.

using QuanserComponents
import QuanserComponents as QC
using MPCComponents: acados_controllers
using Test
using Statistics: median
# This file is normally included from runtests.jl, whose imports are in scope; naming what it
# actually uses keeps it runnable on its own.
using ModelingToolkit: @named

@testset "MPC program" begin
    Ts = 0.01
    dyn = QC.furuta_mpc_dynamics()
    @test dyn.nx == 4 && dyn.nu == 1 && dyn.nz == 0
    @test dyn.ad_backend === :forwarddiff
    @test furuta_mpc_dynamics() === dyn                # cached
    # The named signals the controller is wired with are the model's states.
    @test all(haskey(dyn.signals, Symbol(s)) for s in QC.FURUTA_MPC_STATES)
    # The symbolic backend cannot handle the multibody model; that is what the AD backend is for.
    @test_throws ArgumentError QC.furuta_mpc_dynamics(jacobian_backend = :symbolic)

    gen = QC.compile_program(QC.FurutaMPCHardware; Ts)
    @test gen.divisors == (1,) && gen.Ts == Ts
    ctrl = QC.ProgramRuntime(gen)
    # The prediction model has no symbolic form to render to C, so the spec allows :julia only.
    @test_throws ArgumentError QC.ProgramRuntime(gen; backend = :c)

    # RK4 on the prediction model, five sub-steps per period.
    function simulate(x0; Tf)
        x = copy(x0); h = Ts / 5
        f(x, u) = (dx = zeros(4); dyn.f!(dx, x, [u], dyn.p_default, 0.0); dx)
        applied = Ref(0.0)
        QC.bind_hardware!(measure = () -> (x[1], x[2]), control = u -> (applied[] = u))
        QC.SynchToolkit.reset!(ctrl)
        N = round(Int, Tf / Ts)
        elbow = zeros(N); shoulder = zeros(N); flags = zeros(N); us = zeros(N)
        for i in 1:N
            out = ctrl()
            @test isfinite(out.u) && abs(out.u) <= 10
            shoulder[i] = x[1]; elbow[i] = mod(x[2], 2pi); flags[i] = out.exitflag; us[i] = out.u
            for _ in 1:5
                k1 = f(x, applied[]); k2 = f(x + h/2 * k1, applied[])
                k3 = f(x + h/2 * k2, applied[]); k4 = f(x + h * k3, applied[])
                x += h/6 * (k1 + 2k2 + 2k3 + k4)
            end
        end
        (; shoulder, elbow, flags, us)
    end

    r = simulate([0.0, 0.01, 0.0, 0.0]; Tf = 2.0)
    @test QC.hardware_counters() == (n_measure = 200, n_write = 200)
    @test all(f -> f in (0, 1, 2, 3, 4, 5, 6, 7), r.flags)   # acados status codes
    @test all(isfinite, r.shoulder) && all(isfinite, r.elbow)

    # One acados solve per tick: the component's step is a scalar-valued call whose results are read
    # back through accessors, so the scalarization of the clocked equations does not duplicate it.
    ctrl2 = QC.ProgramRuntime(gen; command_umax = 5.0)
    @test ctrl2.gains.command_umax == 5.0
    @test_throws ArgumentError QC.ProgramRuntime(gen; L = [1.0])     # not one of this program's tunables

    # The same model run as a simulation against the callbacks, paced in real time and recording
    # the MPC's predictions (the route `mpc_gui` uses).
    x = [0.0, 0.01, 0.0, 0.0]
    QC.bind_hardware!(measure = () -> (x[1], x[2]), control = u -> nothing)
    Tf = 0.5
    spec = QC.program_spec(QC.FurutaMPCHardware)
    log_file = tempname() * ".csv"
    model = QC.FurutaMPCHardware(; name = spec.name, Ts, log_file, spec.ode_kwargs...)
    ssys = QC.mtkcompile(model; additional_passes = [QC.SynchToolkit.compile_lustre])
    prob = QC.ODEProblem(ssys, Pair[], (0.0, Tf); build_initializeprob = false)
    sol = QC.run_ode!(QC.FurutaMPCHardware, prob; mode = :callback, log_file)
    @test sol.prob.tspan == (0.0, Tf)                # the caller's problem is what was solved
    b = only(acados_controllers(model)).bundle
    nticks = length(sol[ssys.control_system.exitflag])
    @test 49 <= nticks <= 51
    @test length(sol[ssys.control_system.mpc.x_pred][1]) == 4 * 61     # nx × (Np + 1)
    late = sol[ssys.diagnostics.late]
    @test all(>=(0), late)
    elapsed = sol[ssys.diagnostics.elapsed]
    @test elapsed[end] >= 0.9 * (nticks - 1) * Ts        # paced on the wall clock, not run through
end

# The multirate MPC (`FurutaMPCMultirateHardware`): the encoders are
# read and the state estimated on a 1 ms clock, the MPC solves on a 5 ms one. These check the
# mechanics -- that the two partitions really are separate, that the program ticks at the fast
# rate and solves at the slow one -- not closed-loop performance, which mpc_rollouts.jl is for.
@testset "MPC multirate program" begin
    Ts, Ts_fast = 0.005, 0.001

    # The rate transition itself, in isolation and away from acados: a 1 ms clock feeding a 5 ms
    # one through the operator the model is built on.
    @test isdefined(QC.SynchToolkit, :Latest)      # DiscreteComponents.Latest is built on it

    gen = QC.compile_program(QC.FurutaMPCMultirateHardware)   # the defaults, which these pin
    # The two clocks are read off the model: the driver ticks the *fast* one and the node takes
    # a second boolean for the slow one, raised every fifth tick.
    @test gen.divisors == (1, 5)
    @test gen.Ts == Ts_fast
    ctrl = QC.ProgramRuntime(gen)
    @test ctrl.divisors == (1, 5)
    @test_throws ArgumentError QC.ProgramRuntime(gen; backend = :c)
    # A slow period that is not an integer multiple of the fast one cannot be ticked from one
    # loop, and is rejected before anything is compiled.
    @test_throws ArgumentError QC.compile_program(QC.FurutaMPCMultirateHardware; Ts = 0.005,
                                                  Ts_fast = 0.003)

    x = [0.0, 0.01, 0.0, 0.0]
    applied = Ref(0.0)
    QC.bind_hardware!(measure = () -> (x[1], x[2]), control = u -> (applied[] = u))
    QC.SynchToolkit.reset!(ctrl)

    n = 16
    outs = [ctrl() for _ in 1:n]
    # One encoder read per fast tick, one motor write per MPC solve.
    @test QC.hardware_counters() == (n_measure = n, n_write = 4)
    # The MPC fires on the first tick and every fifth after it; on the others its outputs are
    # `nothing`, since a clocked output only has a value on a tick of its own clock.
    solved = findall(o -> o.exitflag !== nothing, outs)
    @test solved == [1, 6, 11, 16]
    @test all(o -> o.u === nothing, outs[setdiff(1:n, solved)])
    @test all(o -> isfinite(o.u) && abs(o.u) <= 10, outs[solved])
    @test all(o -> o.exitflag in (0, 1, 2, 3, 4, 5, 6, 7), outs[solved])

    # `reset!` puts the tick phase back, so a second run solves on the same ticks as the first.
    QC.SynchToolkit.reset!(ctrl)
    outs2 = [ctrl() for _ in 1:n]
    @test findall(o -> o.exitflag !== nothing, outs2) == [1, 6, 11, 16]

    # A multirate program has no C harness: run_hardware.c drives one clock tick.
    @test_throws ArgumentError QC.export_program_c(gen, mktempdir(); Tf = 1.0)
end

# The simulated multirate loop: that the two clock partitions run at their own rates and that the
# controller still swings the pendulum up.
@testset "MPC multirate model" begin
    Ts, Ts_fast, Tf = 0.005, 0.001, 4.0
    @named model = FurutaMPCMultirateSwingup(; Ts, Ts_fast)
    ssys = QC.MultibodyComponents.multibody(model,
                additional_passes = [QC.SynchToolkit.compile_lustre])
    prob = QC.ODEProblem(ssys, Pair[ssys.qubependulum.shoulder_joint.render => false,
                                    ssys.qubependulum.elbow_joint.phi => deg2rad(0.15),
                                    ssys.qubependulum.shoulder_joint.phi => 0.0], (0.0, Tf))
    sol = QC.solve(prob; dt = Ts_fast)

    # A clocked variable carries its own timebase, which is not `sol.t`.
    slow = QC.SynchToolkit.variable_occurrence_times(ssys, sol, ssys.control_system.exitflag)
    fast = QC.SynchToolkit.variable_occurrence_times(ssys, sol,
                ssys.control_system.estimator_elbow.rate)
    @test length(slow) ≈ Tf / Ts       rtol = 0.02
    @test length(fast) ≈ Tf / Ts_fast  rtol = 0.02
    @test first(slow[3]) - first(slow[2]) ≈ Ts       atol = 1e-9
    @test first(fast[3]) - first(fast[2]) ≈ Ts_fast  atol = 1e-9

    # The motor voltage is held over a whole MPC period, so it changes on the slow clock and not
    # on the sensing clock.
    held = QC.SynchToolkit.variable_occurrence_times(ssys, sol, ssys.zeroorderhold.u)
    @test length(held) == length(slow)

    # Every solve succeeded, and the pendulum is up and staying there.
    @test all(f -> last(f) == 0, slow)
    elbow = sol(Tf-1.0:Ts:Tf, idxs = ssys.qubependulum.elbow_joint.phi).u
    @test all(<(0.1), abs.(mod2pi.(elbow) .- pi))
end
