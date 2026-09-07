# Development notes

## FurutaMPC: the solver settings, and where the tick goes

Measured by ticking the compiled program (`MPCController`, the very node the rig runs) against the
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
