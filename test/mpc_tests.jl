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
