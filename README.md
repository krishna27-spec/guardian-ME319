# Guardian: predictive turret control (ME319 Assignment 2)

Team: Srikrishna (24B2198), target estimation and fire control; Shaunik Jain (24B2183), turret drive and feedback control.

MATLAB / Octave code for estimating a manoeuvring target from radar, predicting it 0.5 s ahead, aiming and firing the laser, and controlling the turret elevation drive. One frozen design, used unchanged for every case.

## Reproduce the results

Run from this folder, in MATLAB or Octave (no toolboxes needed):

| Command | What it produces | Octave time |
|---|---|---|
| `run_estimation_tests` | T1 prediction error, T2 filter consistency (NEES, 30 runs), T3 τ_m sensitivity, T4 aiming experiment, T5 repeated-dot test → `figs/T*.png` | ~6 min |
| `controller_design_check` | plant poles, crossover, phase budget, margins (continuous and sampled), closed-loop and anti-windup poles → `figs/C0_loop_bode.png` | ~5 s |
| `controller_tests` | step, ramp, parabola, slew and clamp tests of the loop → `figs/C1_controller_tests.png` | ~15 s |
| `run_scenarios` | full closed loop: Scenarios A, B, C, BC + 20 random cases → `figs/S_*.png`, `figs/results.csv` | ~6 min |
| `run_multi_blackout` | Scenario B against MB (B with three radar blackouts): the MB row of the paper's Table II, with every pulse | ~25 s |

MATLAB is several times faster than Octave.

## Files

**Assignment interface** (signatures fixed by the assignment)

| File | Role |
|---|---|
| `target_motion_generator.m` | radar: 20 Hz noisy (r, θ), each sample delivered once, blackout windows |
| `turret_controller.m` | the complete controller: estimator → fire check → aim planner → compensator |
| `turret_plant.m` | elevation drive derivatives (DC motor + 100:1 gear + turret), hard stops |

**Estimation and fire control**

| File | Role |
|---|---|
| `target_estimator.m` | Singer-model Kalman filter in Cartesian (x, y), converted measurements, 0.5 s prediction, θ_ref |
| `singer_FQ.m` | discrete Singer F(Δt) and Q(Δt) (Van Loan's method) |
| `aim_planner.m` | lead-and-wait aiming: track the predicted angle, then hold it so the target flies into the beam |
| `fire_check.m` | fire only if the actual beam passes the current estimate within R_eng with a 2σ margin, at closest approach, respecting the recharge time |
| `est_params.m` | every estimation / aiming / fire parameter |

**Turret control** (design from the controls section)

| File | Role |
|---|---|
| `turret_compensator.m` | type-2 PI + notch, Tustin at 500 Hz with its own clock, velocity feed-forward, ±300 V with Hanus anti-windup |
| `ctrl_params.m` | plant constants and the frozen controller parameters |

**Simulation and tests**

| File | Role |
|---|---|
| `simulate_engagement.m` | closed loop with the assignment's pulse rules (0→1 edges, 1 s recharge) and hit test (distance from the beam ray to the true target) |
| `target_truth.m`, `scenario_spec.m` | true trajectories (A, B, C, BC, MB = B with three blackouts, random admissible cases, and 'W<n>' wild cases with hard turns, speed changes and 0-3 blackouts; all with limit checks); blackouts are a list of `[onset end)` rows |
| `plant_rk4_step.m` | RK4 step of the plant with the voltage held |
| `run_case_open_loop.m`, `ctrl_loop_sim.m` | open-loop estimator runs; controller-only runs on synthetic references |
| `loop_margins.m`, `tustin_ss.m`, `settle_time.m`, `shade_blackout.m`, `headless_plots.m` | small helpers |

## Assumptions to state in the paper

* The turret starts horizontal and at rest (θ = 0, θ̇ = 0, i = 0).
* Radar noise is Gaussian (the assignment gives only zero mean and the standard deviations).
* `turret_controller` is called with non-decreasing time; t = 0 (or a jump back > 0.1 s) starts a new run.
* The controller samples at 500 Hz on its own clock, whatever the simulation step.
