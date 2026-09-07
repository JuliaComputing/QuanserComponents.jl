# The MPC program (`FurutaMPCHardware`, `MPCController`): compiled once, then ticked against a
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

    ctrl = QC.MPCController(; Ts)
    @test_throws ArgumentError QC.MPCController(; Ts, backend = :c)

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
    ctrl2 = QC.MPCController(; Ts, command_umax = 5.0)
    @test ctrl2.gains.command_umax == 5.0
    @test_throws ArgumentError QC.make_runtime(QC.generate_mpc_controller(; Ts), QC.MPC_OUTPUT_NAMES;
                                               gains = (; L = [1.0]))

    # The same model run as a simulation against the callbacks, paced in real time and recording
    # the MPC's predictions (the route `mpc_gui` uses).
    x = [0.0, 0.01, 0.0, 0.0]
    QC.bind_hardware!(measure = () -> (x[1], x[2]), control = u -> nothing)
    Tf = 0.5
    g = QC.run_mpc_hardware_model(; Tf, Ts, mode = :callback, log_file = tempname() * ".csv")
    b = only(acados_controllers(g.model)).bundle
    ssys = g.sol.prob.f.sys
    nticks = length(g.sol[ssys.control_system.exitflag])
    @test 49 <= nticks <= 51
    @test length(g.sol[ssys.control_system.mpc.x_pred][1]) == 4 * 61     # nx × (Np + 1)
    late = g.sol[ssys.diagnostics.late]
    @test all(>=(0), late)
    elapsed = g.sol[ssys.diagnostics.elapsed]
    @test elapsed[end] >= 0.9 * (nticks - 1) * Ts        # paced on the wall clock, not run through
end

# The multirate MPC (`FurutaMPCMultirateHardware`, `MPCMultirateController`): the encoders are
# read and the state estimated on a 1 ms clock, the MPC solves on an 8 ms one. These check the
# mechanics -- that the two partitions really are separate, that the program ticks at the fast
# rate and solves at the slow one -- not closed-loop performance, which mpc_rollouts.jl is for.
@testset "MPC multirate program" begin
    Ts, Ts_fast = 0.008, 0.001

    # The rate transition itself, in isolation and away from acados: a 1 ms clock feeding an 8 ms
    # one through the operator the model is built on.
    @test isdefined(QC.SynchToolkit, :Latest)      # `SubSampler` needs it; see dyad/subsampler.dyad

    ctrl = QC.MPCMultirateController(; Ts, Ts_fast, Np = 75)
    @test ctrl.divisors == (1, 8)
    @test ctrl.Ts == Ts_fast                       # the driver ticks the *fast* clock
    @test_throws ArgumentError QC.MPCMultirateController(; Ts, Ts_fast, backend = :c)
    # The ratio has to be an integer of at least 2, and is what reaches the compiler rather than
    # a second period that could disagree with the model's in the last bit.
    @test_throws ArgumentError QC.generate_mpc_multirate_controller(; Ts = 0.008, Ts_fast = 0.003)
    @test_throws ArgumentError QC.generate_mpc_multirate_controller(; Ts = 0.001, Ts_fast = 0.001)

    x = [0.0, 0.01, 0.0, 0.0]
    applied = Ref(0.0)
    QC.bind_hardware!(measure = () -> (x[1], x[2]), control = u -> (applied[] = u))
    QC.SynchToolkit.reset!(ctrl)

    n = 17
    outs = [ctrl() for _ in 1:n]
    # One encoder read per fast tick, one motor write per MPC solve.
    @test QC.hardware_counters() == (n_measure = n, n_write = 3)
    # The MPC fires on the first tick and every eighth after it; on the others its outputs are
    # `nothing`, since a clocked output only has a value on a tick of its own clock.
    solved = findall(o -> o.exitflag !== nothing, outs)
    @test solved == [1, 9, 17]
    @test all(o -> o.u === nothing, outs[setdiff(1:n, solved)])
    @test all(o -> isfinite(o.u) && abs(o.u) <= 10, outs[solved])
    @test all(o -> o.exitflag in (0, 1, 2, 3, 4, 5, 6, 7), outs[solved])

    # `reset!` puts the tick phase back, so a second run solves on the same ticks as the first.
    QC.SynchToolkit.reset!(ctrl)
    outs2 = [ctrl() for _ in 1:n]
    @test findall(o -> o.exitflag !== nothing, outs2) == [1, 9, 17]

    # A multirate program has no C harness: run_hardware.c drives one clock tick.
    gen = QC.generate_mpc_multirate_controller(; Ts, Ts_fast, Np = 75)
    @test_throws ArgumentError QC.export_program_c(gen, mktempdir(); Tf = 1.0)
end

# The simulated multirate loop: that the two clock partitions run at their own rates and that the
# controller still swings the pendulum up.
@testset "MPC multirate model" begin
    Ts, Ts_fast, Tf = 0.008, 0.001, 4.0
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
