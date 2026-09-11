# Development notes

## FurutaMPC: the solver settings, and where the tick goes

Measured by ticking the compiled program (a `ProgramRuntime`, the very node the rig runs) against the
simulated plant -- `test/mpc_rollouts.jl`'s harness with the timing kept and the plots dropped --
24 rollouts of 6 s per configuration from the same initial conditions (3 from rest near hanging,
21 from the random operating space), 14 400 ticks each, every configuration swinging up and
balancing in all 24. Timings are the wall time of one `ctrl()` call, which is the hardware read,
the velocity estimators, the acados solve, the write and the log row.

`qp_cond_N`, the horizon HPIPM partially condenses the QP to, is the one setting of the two with a
real effect, and the existing default of 5 is its optimum:

| `qp_cond_N`         |    1 |    2 |    3 | **5**    |   10 |   20 |   30 | -1 (= 60) |
| ------------------- | ---- | ---- | ---- | -------- | ---- | ---- | ---- | --------- |
| median tick [ms]    | 1.45 | 1.01 | 0.98 | **0.97** | 1.00 | 1.07 | 1.13 |      1.30 |
| 99th %ile tick [ms] |  5.9 |  4.3 |  3.1 |  **2.3** |  2.5 |  3.0 |  3.5 |       5.1 |

3 to 10 are within a few percent of each other, so the setting is not delicate; condensing to 1 or
2 stages costs more than it saves, and the uncondensed horizon costs 34 % at the median and 2.2× at
the 99th percentile. The profile has the same shape and the same optimum under the `FiniteDiff`
backend (0.93 / 0.93 / 0.96 / 1.03 / 1.26 ms at the median for 3, 5, 10, 20, -1), so the optimum is
a property of the QP and not of how the model is differentiated -- which is the answer to whether a
different Jacobian backend would move it.

Of the ~0.97 ms tick, about half is the prediction model. The Jacobian costs 3.3 µs (minimum) to
4.5 µs (median) per call and the right-hand side 0.6 µs, and ERK with 2 stages over 60 shooting
intervals evaluates them 120 times a tick. The rest is HPIPM and acados' own bookkeeping.

The tick also allocates 1.0 MB, which is neither acados' nor MPCComponents' doing: the numeric
right-hand side allocates 2.6 kB *per call* on its own (`@allocated dyn.f!`, steady state, not a
first-call effect) and the AD Jacobian 6 to 7 kB, inside the cached linear solve MTK compiles the
multibody model into (`ModelingToolkitTearing.safe_ldiv` builds and solves a fresh `LinearProblem`
each call). At 120 evaluations a tick that is the whole megabyte. It costs 9 % of tick time in
garbage collection and, worse, the occasional 40 to 150 ms tick -- 46 of 14 400 ticks over the 10 ms
period. That allocation, not the QP, is what a further round of tuning should attack; the horizon
and the condensing are already at their optimum.

## FurutaMPC: why the sparse AD Jacobian backends are of no use here

`ACADOSMPC` gained `SparseForwardDiff`/`SparseFiniteDiff` in
JuliaComputing/MPCComponents.jl#19 -- detect the Jacobian sparsity pattern once with
SparseConnectivityTracer, colour the columns, and evaluate one AD direction per colour instead of
one per entry of `[x; u]`. Updating to that pull request left every tick statistic of this
controller unchanged: 0.97 ms median, 2.3 ms at the 99th percentile and 993 kB per tick before and
after, and the same at `qp_cond_N` 10 and -1. Its other two improvements do not reach this model
either -- the allocation fix is for a `reshape` that ForwardDiff only performs above 12 AD
directions (this model has 5), and `cse` shares subexpressions in `build_function`-built code,
which here is only the tiny `pendulum_energy_ratio` output.

Two independent reasons the sparse backends cannot help:

1. **The pattern cannot be detected.** `continuous_dynamics(...; jacobian_backend =
   :sparse_forwarddiff)` throws at construction. Detection traces the right-hand side with
   SparseConnectivityTracer's *global* `GradientTracer`, which carries index sets and no value, so
   every comparison on it (`<`, `>`, `isless`, `<=`, `==` -- all registered as zero-derivative
   2-to-1 operators) returns another tracer rather than a `Bool`. The compiled multibody model
   reaches a linear solve, and its pivot search branches on a comparison:

       LinearAlgebra.generic_lufact!    stdlib/LinearAlgebra/src/lu.jl:170   `if absi > amax`
       LinearSolve.solve!               factorization.jl:697
       ModelingToolkitTearing.safe_ldiv reassemble.jl:715
       <the MTK generated control function>
       MPCComponents._acados_dynamics_numeric's f!

   giving `TypeError: non-boolean (GradientTracer{Int64, BitSet}) used in boolean context`.
   MPCComponents catches this and says to use the dense backend.

   SparseConnectivityTracer's own answer to a value-dependent branch is the *local* detector,
   `TracerLocalSparsityDetector`, whose `Dual` tracers do carry a primal, so the comparison returns
   a real `Bool` and the branch is taken as it would be at that point. That does trace this model,
   and the resulting sparse Jacobian equals the dense one exactly.

2. **The pattern has nothing to exploit.** With the local detector the pattern is 12 of 20 entries:

       ⋅  ⋅  1  ⋅  ⋅          (d/dt shoulder angle  = shoulder velocity)
       ⋅  ⋅  ⋅  1  ⋅          (d/dt elbow angle     = elbow velocity)
       1  1  1  1  1          (the two accelerations depend on everything,
       1  1  1  1  1           as a two-link manipulator's must)

   Rows 3 and 4 are dense, so no two of the five columns may share a colour: a column colouring
   needs 5 directions, exactly what dense forward AD already does. Measured, the sparse path through
   the local detector is 21 % *slower* per call (4.07 µs minimum against 3.35 µs) for the
   compression and decompression it adds. This is a property of the model, not of its size --
   any serial-link manipulator whose state is `[q; q̇]` has a Jacobian of this shape, and the
   acceleration block is dense in `q` and `q̇` whatever the link count. Sparse AD pays on models
   with weak coupling (a mass chain, a discretized PDE), not on this one.

So the dense `:forwarddiff` backend stays, and `furuta_mpc_dynamics` accepts the `:sparse_` names
only so that the comparison is one keyword away should detection ever work. `:finitediff` is a
shade faster at the median (0.93 ms) but worse where it matters -- 3.0 ms at the 99th percentile,
1.7 MB per tick against 1.0 MB, 114 of 14 400 ticks over the 10 ms period against 46 -- and its
Jacobian is only an approximation.

## The multirate MPC: what the fast clock actually buys

`FurutaMPCMultirate` samples the encoders and estimates the state at 1 ms while the MPC solves at
5 ms (8 ms when the measurements below were taken; the ratio is what they are about). The obvious
reading -- that sampling faster gives a better velocity estimate -- is wrong, and measuring it was
what led to the design.

The metric below is the RMS error of the velocity estimate against the truth, on an angle quantized
to real encoder counts, over trajectories the controller actually sees. It folds lag and noise into
the one number the LQR pays for. Two earlier metrics were discarded first, and both are worth
naming because they are the obvious ones to reach for:

  - **An impulse-response noise gain** assumes the encoder error is white. It is not: the QUBE's
    encoder is 2048 counts per revolution, so one count is 3.07 mrad and, at 1 ms, 3.07 rad/s.
    Below that speed the angle simply does not change between samples, and the error is a
    staircase strongly correlated with the trajectory.
  - **A phase-deficit lag** flatters any estimator whose internal model contains the test signal: a
    constant-jerk tracker scores exactly zero lag on a constant-acceleration ramp.

The balancing trajectory is a 0.3 rad oscillation at the pendulum's natural frequency (8.07 rad/s,
peak 2.4 rad/s -- under one count per tick, so quantization dominates); the swing-up is a 3.7 rad
one (peak 30 rad/s, ten counts per tick, so lag dominates).

| estimator | balance | swing | kick | impact |
| --- | --- | --- | --- | --- |
| difference + lowpass, 10 ms, `a = 0.8` (the single-rate controller) | 0.141 | 1.280 | 0.241 | 0.158 |
| difference + lowpass, 1 ms, `a = 0.1487` (equal time constant) | 0.164 | 1.067 | 0.164 | 0.147 |
| difference + lowpass, 1 ms, `a = 0.125` (equal lag) | 0.156 | 1.279 | 0.179 | 0.133 |
| linear-fit FIR differentiator, 16 taps at 1 ms | 0.110 | 1.275 | 0.202 | 0.103 |
| quadratic-fit FIR differentiator, 32 taps at 1 ms | 0.055 | 0.144 | 0.210 | 0.114 |
| alpha-beta tracker, 1 ms, `q = 1e4` | 0.104 | 0.642 | 0.141 | 0.102 |
| **alpha-beta-gamma tracker, 1 ms (`AlphaBetaGammaFilter`)** | **0.035** | **0.243** | 0.300 | 0.124 |

Three things fall out of it:

1. **Sampling the same estimator faster buys nothing.** Retuning the exponential filter to hold
   its time constant scores 0.164 against the baseline's 0.141 while balancing -- slightly worse --
   and only 16 % better on the swing-up. The estimator structure, not the rate, is the binding
   constraint. (An earlier version of this note claimed a `sqrt(10)` noise penalty for that
   retuning; that was wrong, because it compared at equal filter time constant and ignored that the
   difference's own half-sample lag drops from 5 ms to 0.5 ms. The real figure is about 1.16x.)
2. **Estimator *order* is what matters.** Every constant-velocity estimator is stuck near the
   baseline on the swing-up -- the 16-tap linear fit scores 1.275 against 1.280 -- because a
   swinging pendulum's velocity is never locally constant, so the estimate carries an
   acceleration-induced bias that no bandwidth choice removes. Fitting the acceleration removes it.
3. **The fast clock is what makes order 2 affordable.** A 32 ms window costs 32 samples at 1 ms and
   3 at 10 ms, and 3 samples cannot support a quadratic fit's noise rejection. So the two halves of
   the design are one decision, not two.

`AlphaBetaGammaFilter` was upstreamed to DiscreteComponents (#119) rather than written here. Its
`beta` default is the same Kalata steady-state relation the alpha-beta filter already used --
solving the Riccati recursion for the constant-acceleration model reproduces it to 0.002 % -- and
`gamma = beta^2/(2*alpha)` completes it to within 0.2 %.

The cost is in the last two columns. On a 60 rad/s disturbance, far above the pendulum's own
8.07 rad/s and above anything the motor can excite, the third-order tracker is worse than the
baseline (0.300 against 0.241): a model that extrapolates acceleration overshoots on something that
is not accelerating smoothly. The constant-acceleration model also only pays when the signal is
oversampled -- on a sine the crossover is near `w*Ts = 0.15`, and at 1 ms with the pendulum's
`w = 8.07` we sit at 0.008, three orders inside the useful regime. `velocity_alpha` stays runtime-
settable, and `test/mpc_rollouts.jl` is where it should be swept, since it is the only place
encoder quantization is simulated in closed loop.

## Two clocks through `stkcompile`

Three findings from making the multirate program, none of them documented upstream:

  - **`stkcompile` takes more than one `InputClock`.** Each becomes one `Bool` argument of the
    generated node, in the order the `inputs` vector lists them, so a two-clock program is stepped
    as `step!(exe, tick_fast, tick_slow, gains, auto)`.
  - **An output on a clock that did not tick comes back as `nothing`**, so slow-clock outputs are
    `Union{Nothing, Float64}`. That is why `MPC_OUTPUT_NAMES`' `u` and `exitflag` are empty on
    four ticks out of five.
  - **`InputClock(clk; name = ...)` does not work** on this SynchToolkit: `build_input` calls
    `insert_clock(::LustreTranslator, ...)`, which has only a `Dict` method. The clocks are left
    unnamed. Fixed upstream by SynchToolkit.jl#189, which is on `main` but not on the branch this
    package pins.

The MPC's clock cannot be *derived* inside the node instead: `VariableClock`/`ClockValue` codegen
dereferences a `_runtime` that only `compile_sim` installs, so those clocks are simulation-only,
and any clock that is not declared as an `InputClock` is emitted as an unbound identifier.

`ProgramRuntime` therefore generates the tick pattern itself from an integer divisor rather than
taking it from the caller. Raising the slow tick without the fast one compiles and runs, silently
on the previous period's measurements, so the phase is a property of the runtime and `reset!` puts
it back.

## The 1 ms slot does not hold the MPC solve

Per 5 ms period the work is five fast ticks -- an encoder read and two filter updates each, a
microsecond at the median -- plus one solve of about 0.83 ms. That is roughly 17 % utilization, so
the *loop* is comfortable. The *slot* is not: the tick that carries the solve costs 0.83 ms of a
1.0 ms slot before the encoder read is counted, and 1.6 ms at the 99th percentile eats the next
fast slot, which then fires back to back. One of every five encoder samples therefore arrives with
a `dt` of about 0 rather than 1 ms -- on the very clock whose purpose is a clean 1 kHz estimate.
The 50 to 400 ms tail ticks (0.2 % of solves, the allocation problem above) swallow 50 to 400 fast
slots.

`run_inprocess!` keeps an absolute schedule and never skips, so this is jitter rather than failure,
and the `dt` and `exec` log columns measure it directly. If a rig run shows it mattering, `Ts_fast`
is structural: a 2.5 ms base with a divisor of 2 keeps the 5 ms MPC and puts the solve at a third
of a slot. Worth checking first is whether the device sustains 1 kHz reads at all -- each
`hil_read` is a transaction of order 100 to 300 us, and the `exec` column of an existing 5 ms
swing-up log gives the real number.

## The non-uniform shooting grid: what the rate is worth, and what the horizon is not

The MPC's period and its prediction horizon used to be one number: `Np` uniform intervals of
length `Ts`, so 0.6 s at 5 ms would cost 120 decision variables instead of 75 at 8 ms.
JuliaComputing/MPCComponents.jl#44 breaks that link. `ACADOSMPC`'s `time_steps` gives the length of
each shooting interval separately, and `linear_time_steps(Ts, Np, Tf)` grows them by a constant
increment from `Ts` to cover `Tf`; the first interval stays at `Ts`, since its control is the one
the component applies and holds for a clock period, and the stage cost of interval `k` is weighted
by `Δt_k / Ts` so that a long interval counts for as much as the short ones it replaces.
`FurutaMPCMultirate` therefore has a `horizon` parameter beside `Ts` and `Np`, and at its defaults
50 intervals grow from 5 ms to 27 ms and span 0.8 s.

Measured with `test/mpc_rollouts.jl` in its multirate mode -- the compiled program ticked against
the simulated pendulum with the QUBE's encoder quantization, 50 rollouts of 10 s from the same
seeds, the identified plant, `qp_cond_N = 5` -- every configuration below swung up and balanced in
all 50, from rest near hanging and from the random operating space alike:

| `Ts` / `Np` / horizon | grid | catch median / 90 % [s] | solve median / 99 % [ms] | status ≠ 0 |
| --- | --- | --- | --- | --- |
| 8 ms / 75 / 0.60 s (the previous default) | uniform | 1.04 / 1.64 | 1.23 / 2.78 | 0 of 62 500 |
| 5 ms / 50 / 0.80 s (the default now) | linear, 5 → 27 ms | 0.36 / 1.36 | 0.82 / 1.64 | 2 of 100 000 |
| 5 ms / 50 / 0.80 s, `integrator_stages = 4` | linear, 5 → 27 ms | 0.48 / 1.25 | 1.36 / 2.40 | 5 of 100 000 |
| 5 ms / 50 / 0.25 s | uniform | 0.35 / 0.55 | 0.83 / 1.09 | 0 of 100 000 |

and from the random operating space, where a long horizon should have the most to say:

| `Ts` / `Np` / horizon | catch median / 90 % / max [s] | arm median [rad] | solve median / 99 % [ms] | status ≠ 0 |
| --- | --- | --- | --- | --- |
| 8 ms / 75 / 0.60 s, uniform | 1.14 / 1.88 / 3.51 | 2.81 | 1.24 / 2.92 | 98 of 62 500 |
| 5 ms / 50 / 0.80 s, linear | 0.97 / 1.47 / 1.92 | 2.56 | 0.82 / 1.95 | 295 of 100 000 |
| 5 ms / 50 / 0.25 s, uniform | 0.75 / 1.07 / 1.28 | 2.46 | 0.83 / 1.46 | 175 of 100 000 |

Four things fall out of it:

1. **The rate is what pays.** Both 5 ms configurations dominate the 8 ms one on every column:
   a third less solve time at the median, a third less at the 99th percentile, the swing-up caught
   in a third of the time from rest and the arm's excursion smaller. 0.83 ms in a 5 ms slot is
   17 % utilization against the 15 % the 8 ms controller had, so the rate increase is nearly free.
2. **The 0.8 s horizon is not.** A uniform 0.25 s grid at the same `Ts` and `Np` catches sooner
   (0.75 s against 0.97 s at the median, 1.07 against 1.47 at the 90th percentile, and 1.28 against
   1.92 in the worst rollout), returns a nonzero acados status on 0.18 % of solves against 0.30 %,
   and costs the same. The same ordering holds on the perturbed plant -- the 15 % weaker motor with
   the heavier arm -- where the long horizon should help most: 0.86 / 1.25 / 1.55 s against
   1.14 / 2.07 / 3.68 s. The explanation is the cost the horizon is spent on: `energy_weight = 1e5`
   on the pendulum's energy error makes the stage cost a shaping term that plans the pump without
   needing to see the catch, and stretching the grid both dilutes that near-term shaping with stage
   weights of up to 5.4 and pushes the terminal LQR cost -- the only term that knows about
   balancing -- 0.8 s out. `horizon = Np * Ts` is the whole change back.
3. **The integration error over a 27 ms interval does not matter in closed loop.** acados
   integrates every interval with one step of the same scheme, so the last interval of the grid is
   integrated 5.4 times more coarsely than the first. Against a finely integrated reference over
   the states a swing-up traverses, one step of the 2-stage scheme is off by up to 4.1 rad/s over
   27 ms and 0.034 rad/s over 5 ms (RK4: 0.33 and 0.0010). Buying that back with
   `integrator_stages = 4` costs 65 % more solve time and swings up no better, so the default
   stays 2. `integrator_steps` per interval would be the accurate and affordable answer, but
   `ACADOSMPC`'s Dyad parameter is a scalar and only `build_acados_mpc` takes the vector form.
4. **`qp_cond_N = 5` survives the smaller horizon.** Re-swept at `Np = 50` (12 rollouts from rest,
   24 000 solves each), the median is 0.92 / 0.82 / 0.83 / 0.86 / 1.08 ms and the 99th percentile
   3.2 / 2.5 / 1.9 / 1.2 / 4.0 ms for 2 / 3 / 5 / 10 / -1 -- the same shape as at `Np = 60`:
   3 to 10 within a few percent, 1 and 2 and the uncondensed horizon worse. 10 has the better tail
   from rest (1.38 ms against 1.65 at 50 rollouts) and the worse one from random starts (2.29
   against 1.99), which is within the noise, so the measured optimum is left where it was.

The grid reaches the whole controller, not only the solver: with `reference_preview` a preview
block would be tied to the shooting nodes rather than to multiples of `Ts` (this controller
previews nothing), and `penalize_increments` is refused outright on a non-uniform grid, which is
why `FurutaMPC`'s `penalize_increments = false` is load-bearing here.
