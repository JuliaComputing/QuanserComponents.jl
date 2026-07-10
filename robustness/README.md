# Swingup robustness campaign

Quantifies and improves the robustness of the energy-based swingup
controller against plant-model mismatch. The controller always uses its
nominal parameter values; mismatch is injected by perturbing the plant.

## Setup

All scripts run in the environment defined by `robustness/Project.toml`:

```
julia --project=robustness robustness/<script>.jl
```

The Lustre-compiled model holds a single shared executable, so campaigns run
serially (no threads). Derived plant parameters (`Jr`, `l`, `Jp`) are baked
numeric at model construction and are recomputed by
`perturbation_overrides` whenever their parents are perturbed.

| Script | Purpose |
|---|---|
| `00_baseline.jl` | Nominal run; saves the behavior-neutrality reference and regression-checks against it |
| `01_sweeps_1d.jl` | Per-parameter failure-boundary sweeps with bisection |
| `02_montecarlo.jl` | Joint Monte Carlo over all plant parameters |
| `03_heatmaps_2d.jl` | 2D success grids over parameter pairs, quantization and delay panels |
| `04_catch_region.jl` | LQR region-of-attraction slices (switch forced to LQR via `neartop.th => 1e6`) |
| `05_compare.jl` | Cross-config comparison tables |
| `06_phase4_mc.jl` | Paired MC (same seed) for all controller configs |
| `phase1_smoke.jl`, `phase3*_smoke.jl` | Per-change feature verification |

## Mismatch mechanisms (all off by default, behavior-neutral)

- **Coulomb friction**: `SmoothCoulombFriction` on both joints
  (`qubependulum.shoulder_friction.tau_c`, `...elbow_friction.tau_c`).
- **Encoder quantization**: `elbow_sampler.quantized => true` (13-bit
  midtread over ±4π ≈ the real Qube's 2048 counts/rev).
- **Actuation delay**: `FurutaSwingupDelayed` model variant with structural
  `delay_n` samples between controller and ZOH.

## Controller configurations

- `baseline` — controller from `test/runtests.jl` (k = 100, umax = 3).
- `margin` — normalized energy error + static reference margin `eta = 0.05`.
- `adaptive` — normalized error + online reference adaptation
  (`ErefAdaptation`, `gamma = 0.05`): the energy target grows only when
  swing apexes demonstrably fall short of the top, and bleeds off while
  stabilized.
- `adaptive_ab` — additionally the alpha-beta velocity estimator
  (`alpha = 0.85`), 28% lower velocity RMS error under quantization than
  the original discrete derivative + exponential filter.

## Baseline findings (tf = 15 s budget)

- **1D sweeps: no single-parameter failures.** Within ±30% mass, ±15%
  length, ±20-25% motor parameters, 0.1-20x viscous damping, and Coulomb
  levels up to ~30% of available drive torque, the nominal swingup always
  succeeds; friction only delays the catch (5.7 s → 12 s at the elbow
  Coulomb extreme).
- **Joint Monte Carlo: 69% ± 5% success (N = 300).** Failures concentrate
  where dissipation is high and drive authority low: conditional failure
  rates are 45% for high elbow Coulomb friction, 44% for high motor
  resistance, 39% for high pendulum damping and shoulder Coulomb.
- **The angle-only handover is the structural weakness.** LQR catch-region
  slices show that at the upright the LQR catches up to |ω| ≈ 5.3 rad/s
  nominally, shrinking to ≈ 2.7 rad/s under +20% pendulum mass or realistic
  Coulomb friction — while the baseline swingup arrives at 4.4 rad/s.
  Mismatch simultaneously raises arrival speed and shrinks the catchable
  set; only ~40% of the states inside the `|α−π| < 0.4` switching band are
  actually catchable even nominally. A velocity-aware switching condition
  is the natural next step (not in current scope).
- **Actuation delay is the hardest unmodeled effect.** One sample (5 ms) of
  input delay collapses the LQR catch region from 142/625 to 8/625 grid
  points and makes the nominal loop fail to settle; two samples fail on the
  entire mp × Coulomb heatmap. The discrete LQR (L2 ≈ 394 at 200 Hz) has
  essentially no delay margin; if IO latency is plausible on the real rig,
  the LQR should be redesigned with a delay state or reduced bandwidth.

## Controller comparison (tf = 20 s budget, paired draws)

Same 300 plant draws for every config (fixed seed), N = 300:

| Config | Success | Rescued vs baseline | Broken vs baseline |
|---|---|---|---|
| `baseline` | 76.7% ± 4.8% | — | — |
| `margin` | 80.3% ± 4.5% | 21 | 10 |
| `adaptive` | 89.0% ± 3.5% | 42 | 5 |
| `adaptive_ab` | **93.0% ± 2.9%** | 55 | 6 |

(The tf = 15 baseline of Phase 2 measured 69.0% ± 5.2%; the longer budget
recovers slow catches, so all configs are compared at tf = 20.)

Interpretation:

- The **static margin** rescues under-shooting plants but its
  unconditionally higher arrival speed breaks 10 previously-good cases —
  consistent with the catch-region analysis. Net +11.
- **Online adaptation** only boosts the reference when apexes demonstrably
  fall short, so it rescues twice as many cases while breaking almost none
  (net +37).
- The **alpha-beta estimator** adds a further net +12, mostly in quantized
  runs where the lower-lag velocity estimate improves both the energy
  estimate and the LQR handover.
- The 21 remaining failures have extreme dissipation (median pendulum
  viscous damping 4x the population median, high elbow Coulomb, high Rm)
  and average 2.2 catch-band entries: they reach the band but cannot be
  held. These are catch-capability failures, addressable by more drive
  authority or a velocity-aware switching condition, not by more energy
  pumping.

## Recommendations

1. **(Implemented)** `normalize`, `gamma = 0.05` (adaptation) and the
   alpha-beta estimator (`alpha = 0.85`) are the component defaults; the
   campaign "baseline" config now pins the original controller explicitly.
   `LQRstabilizer.umax` defaults to the value every actual use overrode it
   to; with its old default the out-of-the-box model could not hold the
   catch at all.
2. **(Implemented, opt-in, negative result in simulation)** `CatchCondition`
   replaces the angle-only `NearTop`: it can gate the handover on an
   ellipsoidal sublevel set V = e'Se ≤ c_engage of a Lyapunov function of
   the implemented LQR loop (S from `07_design_catch_condition.jl`,
   calibrated for zero false positives on the empirical catch grids of four
   perturbed plants) with hysteresis release at `c_release`. Findings from
   the paired MC:
   - angle-only switch (default): 93.0%
   - ellipsoid, zero-false-positive calibration (`c_engage = 1`): 79.7%
   - ellipsoid, loosened (`c_engage = 2, c_release = 8`): 76.3% (threshold
     tuning on the broken/rescued subset did not generalize — selection
     bias; draws that were fine under both other configs broke)

   Interpretation: the gate is computed from the nominal model, which is
   exactly what mismatch invalidates, and in simulation failed catch
   attempts are cheap, so refusing marginal attempts costs success. The
   gate's real benefits are qualitative: single chatter-free engagement and
   mean arrival speed under half of the angle-only switch (1.5 vs 3.2 rad/s
   among successes) — relevant on hardware where failed high-speed catch
   attempts cause voltage spikes and mechanical stress. Hence opt-in via
   `catchcondition.use_ellipsoid`, default off. A handover that is both
   gentle and mismatch-robust likely requires redesigning the LQR itself
   (see item 3 and the stale-gain observation in
   `07_design_catch_condition.jl`).
3. **(Implemented)** LQR redesigned from the actual model linearization
   (`08_lqr_redesign.jl`). The original gains did not correspond to any LQR
   design from this model with the documented weights and had no delay
   margin. The redesign keeps the documented output weights and raises the
   control weight until the loop tolerates delay (selected: control weight
   300), validated in simulation rather than by classical margins (the
   plant is unstable, making single-crossover margin numbers misleading):

   | metric | original gains | redesigned |
   |---|---|---|
   | swingup MC (robust config, tf = 20 s) | 93.0% | **93.7%** |
   | swingup MC with 1-sample actuation delay | 57.0% | **92.0%** |
   | swingup with 3-sample delay (nominal) | fails at 1 | catches at 5.8 s |
   | LQR catch region, nominal | 142/625 | 179/625 |
   | LQR catch region, 1-sample delay | 8/625 | 191/625 |
   | catch capability at top (nominal / mp+20%) | 2.5 / 2.5 rad/s | 2.5 / 2.5 rad/s |

   The ellipsoidal catch condition was recalibrated for the new loop
   (`07_design_catch_condition.jl` with `lqr_config = "new"`): the safe
   ellipsoid now retains 63-68% of the catchable set (was 40-44%), still
   with zero false positives across the perturbed plants.

## Reproducing

Baseline campaign: run scripts 00-04 in order. Comparison: `06_phase4_mc.jl`.
CSVs land in `results/`, figures in `figures/` (both gitignored).
