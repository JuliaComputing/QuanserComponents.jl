# The MPC program (`FurutaMPCHardware`, `MPCController`): compiled once, then ticked against a
# simulated plant through the hardware I/O callbacks, as the swing-up program is in runtests.jl.
# The plant is the MPC's own prediction model -- the multibody `QubePendulum` ODE from
# `furuta_mpc_dynamics()` -- integrated with RK4, so this checks the program end to end (angle
# wrapping, velocity estimation and the acados solve) rather than the model.

using QuanserComponents
import QuanserComponents as QC
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

    # From hanging down: the MPC swings the pendulum up and holds it, arm centred.
    r = simulate([0.0, 0.01, 0.0, 0.0]; Tf = 8.0)
    @test QC.hardware_counters() == (n_measure = 800, n_write = 800)
    @test all(abs.(r.elbow[end-100:end] .- pi) .< 0.05)
    @test abs(r.shoulder[end]) < 0.1

    # From a perturbed upright state it balances directly, keeping the arm inside the end
    # stops. (The velocity estimators start cold, so the first tick sees a spurious velocity
    # spike; the solver must shrug it off.)
    r = simulate([0.3, pi - 0.25, 0.0, 0.0]; Tf = 3.0)
    @test all(abs.(r.elbow[end-50:end] .- pi) .< 0.02)
    @test abs(r.shoulder[end]) < 0.05
    @test all(abs.(r.shoulder) .< 1.9198621771937625)

    # The runtime-settable tunable: the command clamp.
    ctrl2 = QC.make_runtime(QC.generate_mpc_controller(; Ts), QC.MPC_OUTPUT_NAMES;
                            gains = (; command_umax = 5.0))
    @test ctrl2.gains.command_umax == 5.0
    @test_throws ArgumentError QC.make_runtime(QC.generate_mpc_controller(; Ts), QC.MPC_OUTPUT_NAMES;
                                               gains = (; L = [1.0]))
end
