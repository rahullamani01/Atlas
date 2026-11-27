# ATLAS — Flagship Engineering & Research Roadmap

**Project:** ATLAS — Autonomous Navigation and Learning System
**Repository:** `github.com/rahullamani01/Atlas`
**Baseline:** Phase 13 plus Phase 10 verification, commit `3e633de` (19 November 2026)
**Purpose:** a shareable strategy for turning ATLAS into a flagship project.

Status of this document: adopted 18 November 2026. Workstream A (verification
and reliability) is complete; Workstream B is in progress. Update
the status column when a workstream starts or lands, and keep the measured
claims in this file in sync with `README.md` and `docs/handoff.md`.

| Workstream | Scope | Status |
|---|---|---|
| A | Verification and engineering reliability | complete (19 Nov 2026) |
| B | Scientific evaluation and benchmark quality | in progress |
| C | ATLAS research dashboard | planned |
| D | Mathematical and technical depth | ongoing (mathematics.md) |
| E | Limitations and future research | planned |
| F | Professional project communication | planned |

---

## 1. Executive vision

ATLAS should be developed as more than a collection of robotics algorithms or
a 2D robot simulation.

The long-term ambition is to present it as a **modular autonomous robotics
research platform**: a system for developing, integrating, visualising and
evaluating how a mobile robot can understand an environment, estimate its own
position, plan movement, recover from failures and explore unknown space.

The goal is not simply to add more features. The goal is to make the existing
work deeper, more reliable, measurable, reproducible and understandable to
other people.

### Central research question

> **How can a robot autonomously understand, navigate and explore an
> environment when its knowledge of the world and its own position is
> incomplete?**

This question provides a unifying story for the project:

- **Kinematics** gives the robot a model of how it moves.
- **Perception** gives it observations of its surroundings.
- **Localization** helps it estimate where it is.
- **Mapping** helps it represent the environment.
- **Planning** helps it decide where to go.
- **SLAM** helps it jointly improve its map and trajectory estimate.
- **Recovery** helps it respond when assumptions or plans fail.
- **Exploration** helps it decide where to gather information next.
- **Evaluation** measures how well the system performs.

---

## 2. Current project baseline

Status as supplied on 18 November 2026. Refresh against the repository before
using it as a formal public claim.

### Project summary

ATLAS is a 2D mobile-robot simulation implemented in Python. NumPy is used for
mathematics, while Matplotlib is used in demonstrations.

The architecture builds an autonomy stack incrementally:

```text
world + robot
      |
   sensors
      |
 localization
      |
 perception
      |
  planning
      |
   control
      |
   recovery
      |
   mapping
      |
 evaluation
```

### Reported scale

- 13 phases and 33 commits on the `main` branch (32 before this documentation
  commit). Pushing is a separate, explicit step.
- Latest implementation commit: `3e633de` (Phase 10 verification).
- 87 tracked files.
- Approximately 6,350 lines of Python in `src/`.
- Approximately 3,290 lines in `tests/`.
- Approximately 2,485 lines in `scripts/`.
- 260 test functions and 290 collected test cases.
- 12 benchmark scenarios, with 3 seeds per scenario.
- JSON and CSV benchmark export.
- Extensive project documentation: README, mathematical derivations, design
  decisions and a handoff guide.

### Phase summary

| Phase | Capability | Reported result / status |
|---|---|---|
| 0 | Scaffold, geometry utilities, angle and distance conventions | Foundation |
| 1 | Differential-drive kinematics, wheel-to-twist conversion, simulation loop | Foundation |
| 2 | Occupancy grids, inflation, Dijkstra and A* | Planning foundation |
| 3 | PID and carrot-chasing path follower | 0.098 m final error, 0.026 m cross-track RMS, 0 collisions |
| 4 | 2D LiDAR ray casting with range noise | Sensor simulation |
| 5 | Odometry with wheel-scale error and particle-filter localization | Localization |
| 6 | Dynamic obstacle detection and recovery supervisor | Detects, replans and resumes |
| 7 | Evaluation harness, metrics, scenarios and benchmark CLI | Shared evaluation runners |
| 8 | Integrated autonomy using the estimated pose | 0.091 m filter RMS, 0.06 m final error, 0 collisions |
| 9 | Kidnapped-robot relocalization | Detection: 0.35 s; reacquisition: 2.35 s; post-recovery RMS: 0.092 m |
| 10 | `LogOddsMap` repair and `MappingNavigator` replanning on the live map | 91.9% of observable prior errors repaired; 0 collisions; 0.101 m filter RMS |
| 11 | Pose-graph SLAM, scan matching and verified loop closure | Odometry ATE: 0.993 m; SLAM ATE: 0.188 m; ATE reduction: 81.1% |
| 12 | Online correction during driving, map rebuild, filter reseeding and continued navigation | Node error: 0.719 m to 0.163 m; second goal error: 0.031 m; 0 collisions |
| 13 | Next-best-view exploration from an empty prior | 95.03% map coverage, 96.21% free space, 18.3 viewpoints, 0.095 m filter RMS, 0 collisions |

### Honest verification status

- All 13 phases are tested, demonstrated and benchmarked.
- The full suite passes: `290 passed` with `python -m pytest -q` on macOS
  (Python 3.14 virtual environment), 2026-11-19.
- The former Phase 10 assertion (`final_map_error <= prior_map_error`) is
  resolved. Verification showed that the expectation, not the mapper, was the
  problem: the prior is an otherwise exact raster, so a sparse SLAM map cannot
  beat it on net cell count even after fixing every planted defect. The metric
  now reports the fraction of prior-error cells repaired, and two genuine
  mapper defects found during verification were fixed.
- Remaining honest caveats: net map error is pessimistic (grazing-ray erosion),
  the benchmark uses one route per scenario, and three seeds are enough for a
  regression baseline but not for statistical claims.

### Existing documentation

- `README.md` — project tour and commands.
- `docs/handoff.md` — current state, APIs and known limitations; recommended
  starting point when resuming work.
- `docs/mathematics.md` — mathematical derivations and measured tables.
- `docs/design_decisions.md` — rationale behind design choices and
  simplifications.
- `ATLAS_FILE_GUIDE.md` — file-by-file guide (kept in `~/Downloads/`).

Before making changes, inspect the actual repository and use the handoff
document as the primary orientation guide.

---

## 3. Strategic principle: make ATLAS deeper, not merely bigger

ATLAS already contains a broad set of capabilities. Adding phases simply to
increase the phase count may produce diminishing returns.

The next stage should focus on five qualities:

1. **Correctness** — the implementation behaves as intended and its assumptions
   are explicit.
2. **Evidence** — claims are supported by tests and measured experiments.
3. **Reproducibility** — another person can run the same experiment and obtain
   comparable results.
4. **Observability** — people can see what the robot estimates, plans and does.
5. **Communication** — the engineering can be explained clearly to someone who
   has not read the source code.

A smaller number of carefully designed experiments can make the project more
convincing than a large number of loosely connected features.

---

## 4. Workstream A — Verification and engineering reliability (complete)

**Outcome (19 November 2026):** the Phase 10 debt is cleared. `python -m pytest
-q` reports `290 passed` in about 90 s on macOS. The failing assertion was
investigated rather than adjusted: `map_repair_ratio` was computing net cell
reduction even though the design document defined it as the fraction of
observable prior error repaired. Fixing the metric to match its documented
meaning made the test assert repair quality instead of a physically
unachievable net count.

Verification also exposed three genuine defects, all fixed: a hit landing
exactly on a cell-boundary face was assigned to the free cell outside the
obstacle; a beam could carve its own terminating cell free; and the truth/prior
rasters omitted the arena boundary that the LiDAR and collision model already
enforce. Measured Phase 10 result: prior error `41` cells, `91.9 ± 2.8%`
repaired, zero collisions, `0.101 m` filter RMS, `59.9 s`. Net map error
(`159.7` cells) is reported separately and documented as pessimistic.

### Objective

Establish a trustworthy baseline before adding major new capabilities.

### Tasks

1. Read `docs/handoff.md`.
2. Reproduce the current test-suite result from a clean checkout.
3. Reproduce the known Phase 10 failure.
4. Run the Phase 10 demonstration (`phase10_demo.py`).
5. Determine whether the assertion is incorrect, the expected metric is wrong,
   or the implementation has a genuine issue.
6. Add focused tests for map repair and `MappingNavigator`.
7. Record the before-and-after behaviour and measured results.
8. Rerun the complete test suite.
9. Update the handoff document and project status.

### Definition of done

- Phase 10 has meaningful behavioural tests, not only import checks.
- The known assertion has a documented resolution.
- The full test result is recorded with the exact command and environment.
- The README and handoff document report the same verification status.

Do not silently change an expected value merely to make a test pass. Explain why
the corrected expectation is valid.

---

## 5. Workstream B — Scientific evaluation and benchmark quality

### Objective

Move from demonstrating that algorithms work to measuring **when, why and how
well** they work.

ATLAS already has a benchmark harness, scenarios and multiple random seeds.
Build on that foundation instead of creating a disconnected evaluation system.

The first study compares the same `loop` scenario and seeds with loop closure
enabled and disabled. Run and plot it with:

```bash
python scripts/loop_closure_study.py run --seeds 0 1 2 --output experiments/studies/loop_closure.json
python scripts/loop_closure_study.py plot experiments/studies/loop_closure.json --output experiments/plots/loop_closure_study.png
```

The JSON retains both individual results and scenario settings. This is a
small, paired experiment on one route, not evidence of statistical significance
or of online navigation improvement: the graph correction happens after driving.

**Measured 23 November 2026**, seeds 0–2, 800 particles and 60 beams:

| Loop closure | Trajectory RMS (m), mean ± sample std [min, max] | Map error (cells), mean ± sample std [min, max] |
|---|---|---|
| Disabled | 0.993 ± 0.023 [0.966, 1.008] | 257.3 ± 61.1 [213, 327] |
| Enabled | 0.188 ± 0.024 [0.161, 0.207] | 69.3 ± 9.6 [59, 78] |

All six runs reached the goal with zero collisions. In this scenario, enabling
loop closure reduced the mean reconstructed trajectory and observed-surface map
errors; these results do not establish performance on other routes. Raw results:
`experiments/studies/loop_closure.json`.

The second study tests localization with LiDAR range-noise standard deviations
of 0.02, 0.10 and 0.30 m, keeping the route, filter settings and seeds fixed.
The filter is not retuned for each noise level. Run and plot the recorded data:

```bash
python scripts/lidar_noise_study.py run --seeds 0 1 2 --output experiments/studies/lidar_noise.json
python scripts/lidar_noise_study.py plot experiments/studies/lidar_noise.json --output experiments/plots/lidar_noise_study.png
```

**Measured 23 November 2026**, seeds 0–2, 800 particles and 60 beams:

| LiDAR noise σ (m) | Filter position RMS (m), mean ± sample std [min, max] |
|---|---|
| 0.02 | 0.0845 ± 0.0004 [0.0842, 0.0849] |
| 0.10 | 0.2451 ± 0.0013 [0.2439, 0.2465] |
| 0.30 | 0.3860 ± 0.0088 [0.3767, 0.3942] |

All nine runs reached the goal without collisions, while localization error
increased with noise on this route. This small study does not establish a
general failure threshold or statistical significance. Individual metrics and
settings are retained in `experiments/studies/lidar_noise.json`.

### Phase 14 — Wheel-calibration error

How much does a fixed calibration error affect dead reckoning, and how much of
that drift remains after scan correction? The `localization` runner drives on
the particle-filter estimate and scores both estimators against simulator truth
at each step. For each seed, the measured wheel speeds use left/right factors
`1 − b` and `1 + b`; the simulator still drives the commanded wheels. The
planned route, LiDAR noise (σ = 0.02 m), particle count (800), beam budget
(60) and filter settings are held fixed. The actual trajectory can change
across bias levels because the controller uses the estimate.

| Per-wheel bias | Odometry position RMS (m) | Filter position RMS (m) | Paired difference (m) |
|---|---:|---:|---:|
| 0% | 0.000075 ± 0.000001 | 0.0823 ± 0.0010 | −0.0822 ± 0.0010 |
| 1% | 11.390 ± 0.013 | 0.0844 ± 0.0003 | 11.306 ± 0.013 |
| 3% | 21.796 ± 0.018 | 0.0902 ± 0.0007 | 21.706 ± 0.017 |

Values are means ± sample standard deviations across seeds 0–4. The difference
is odometry RMS minus filter RMS **within each drive**; positive values favour
the filter. All 15 runs reached the goal with zero collisions. At 1% bias the
per-seed metrics for seeds 0–2 match the nominal LiDAR study. See the
[raw results and ranges](../experiments/studies/wheel_scale.json).

With exact wheel calibration and a known initial pose, simulated dead reckoning
has almost no drift; noisy scan correction does not improve on it. At both
nonzero bias levels the filter keeps position RMS below 0.1 m on this route.
This comparison does not test whether an odometry-driven controller would
reach the goal, or establish a calibration tolerance for real robots.

Reproduce the 15 simulations, then plot from the saved JSON:

```bash
python scripts/wheel_scale_study.py run --seeds 0 1 2 3 4 --output experiments/studies/wheel_scale.json
MPLBACKEND=Agg python scripts/wheel_scale_study.py plot experiments/studies/wheel_scale.json --output experiments/plots/wheel_scale_study.png
```

### Phase 15 — Particle-filter beam budget

Does localization still support navigation when the filter scores fewer valid
returns from each scan? The 60-return reference uses Phase 14's 1% wheel bias
(left/right scales 0.99/1.01), 800 particles and 0.02 m range noise on the
`localization` route. Each budget uses the same five seeds (0–4). The simulated
LiDAR still casts 360 rays. Only the maximum number of valid hits scored in
each filter update changes; there is no sensor dropout or filter retuning.

| Return budget | Position RMS (m) | Heading RMS (°) | Position RMS increase vs 60 (m) | Goals | Collisions |
|---:|---:|---:|---:|---:|---:|
| 60 | 0.0844 ± 0.0003 | 0.183 ± 0.004 | reference | 5/5 | 0 |
| 20 | 0.0907 ± 0.0007 | 0.367 ± 0.011 | 0.0062 ± 0.0008 | 5/5 | 0 |
| 8 | 0.1061 ± 0.0055 | 1.018 ± 0.050 | 0.0216 ± 0.0052 | 5/5 | 0 |
| 2 | 0.9464 ± 0.1374 | 17.50 ± 1.70 | 0.8620 ± 0.1376 | 0/5 | 5 |

Values are means ± sample standard deviations. Differences are computed
against the same seed's 60-return run before averaging. All five reference
runs exactly reproduced the corresponding Phase 14 metrics. The controller
uses the estimate, so seed pairing does not hold the driven trajectory fixed.
RMS includes every executed step up to arrival, collision or timeout; the
two-return errors cover only the trajectory before and including the collision.

On this route, reducing the budget from 60 to 20 cost about 6 mm in position
RMS without a missed goal. At two returns the filter picks the first and last
valid hit in scan order; those bearings can be close across the 360° scan
boundary. Reducing the budget also reduces the number of likelihood terms in
each update. These runs do not isolate either effect or establish a minimum
safe beam count.

[Raw per-seed results, configuration and ranges](../experiments/studies/beam_budget.json).
Reproduce the simulations and plot from the recorded data:

```bash
python scripts/beam_budget_study.py run --seeds 0 1 2 3 4 --output experiments/studies/beam_budget.json
MPLBACKEND=Agg python scripts/beam_budget_study.py plot experiments/studies/beam_budget.json --output experiments/plots/beam_budget.png
```

### Phase 16 — Choosing two returns

Can changing the bearings scored by the filter rescue Phase 15's two-return
runs? `scan_order` (the default) picks the first and last valid hit when the
budget is two. The opt-in `widest_pair` policy picks the valid pair whose
angular difference is closest to 180°, using body-frame bearings only.
It sorts the bearings modulo 360° and checks the two neighbours of the
opposite direction among later bearings for each candidate. This costs
O(n log n) for n valid hits. With fewer than two hits both policies use all
available hits. The measurement update and scan residual use the same selector.

The `localization` runner used seeds 0–4, 800 particles, wheel scales
0.99/1.01, 0.02 m range noise, and unchanged likelihood scale. Only the
selection policy varied. All five `scan_order` runs exactly reproduced
Phase 15's two-return metric dictionaries.

| Selection | Position RMS (m) | Heading RMS (°) | Goals | Collisions |
|---|---:|---:|---:|---:|
| First and last valid hit | 0.946 ± 0.137 | 17.50 ± 1.70 | 0/5 | 5 |
| Widest pair | 0.925 ± 0.044 | 17.00 ± 0.72 | 0/5 | 5 |

Values are means ± sample standard deviations across seeds. The paired position
RMS difference (widest minus scan order) was −0.021 ± 0.123 m, ranging from
−0.173 to +0.138 m. Wider separation alone did not prevent a collision in
any drive. Two opposite returns need not constrain independent pose errors,
for example when they see parallel walls. The study did not measure which
walls caused the failures. Each controller follows its own estimated pose;
RMS is measured up to that drive's collision, not over identical trajectories.

[Raw paired metrics and settings](../experiments/studies/beam_pair.json):

```bash
python scripts/beam_pair_study.py run --seeds 0 1 2 3 4 --output experiments/studies/beam_pair.json
MPLBACKEND=Agg python scripts/beam_pair_study.py plot experiments/studies/beam_pair.json --output experiments/plots/beam_pair.png
```

### Phase 17 — Weighting two returns

Phase 16 held the filter's likelihood scale fixed. Does weighting the same
two hits more strongly change localization or navigation? The filter adds
`−scale / 2 × Σ(distance / measurement_sigma)²` to each particle's log weight.
At a two-hit budget, scales 0.8 and 6.0 give the sum the same total coefficient
as 8 and 60 equally weighted hits at the reference scale of 0.2, respectively,
if their mean squared residuals are equal. Multiplying a residual does not
supply the geometry of additional measurements.

Both Phase 16 selectors ran at all three scales: five seeds (0–4), 800
particles, wheel scales 0.99/1.01, LiDAR range noise 0.02 m. The ten runs at
0.2 exactly reproduced the previously recorded localization metrics. In this
study the runner additionally scores elapsed simulated time and distance from
the **true final pose** to the last planned waypoint. Neither drives control.

| Selector | Scale | Position RMS (m), mean ± sample SD | Heading RMS (°), mean | Simulated time (s), mean | Final true goal distance (m), mean | Reported arrivals | Collisions |
|---|---:|---:|---:|---:|---:|---:|---:|
| Scan order | 0.2 | 0.946 ± 0.137 | 17.50 | 20.43 | 15.31 | 0/5 | 5 |
| Scan order | 0.8 | 1.169 ± 0.229 | 19.26 | 19.16 | 15.57 | 0/5 | 5 |
| Scan order | 6.0 | 1.083 ± 0.169 | 17.26 | 19.64 | 15.43 | 0/5 | 5 |
| Widest pair | 0.2 | 0.925 ± 0.044 | 17.00 | 20.47 | 15.16 | 0/5 | 5 |
| Widest pair | 0.8 | 0.241 ± 0.241 | 4.24 | 26.74 | 13.56 | 0/5 | 5 |
| Widest pair | 6.0 | 0.128 ± 0.048 | 1.38 | 34.43 | 10.61 | 1/5 | 4 |

At 6.0 the widest-pair RMS dropped on all five seeds: paired change vs 0.2
was −0.797 ± 0.073 m. For scan order the mean change was +0.136 ± 0.280 m.
The only controller-reported arrival (widest pair, 6.0, seed 0) ended 0.218 m
from the goal. None of the 30 final true poses met the 0.10 m arrival
tolerance. These runs cover one route, with RMS measured until each drive
stopped. They support a selector-dependent effect on localization, not a
general tuning recommendation; the default remains 0.2.

[Per-seed results, paired differences and ranges](../experiments/studies/likelihood_scale.json).
Reproduce the experiment and plot final goal distance from recorded results:

```bash
python scripts/likelihood_scale_study.py run --seeds 0 1 2 3 4 --output experiments/studies/likelihood_scale.json
MPLBACKEND=Agg python scripts/likelihood_scale_study.py plot experiments/studies/likelihood_scale.json --output experiments/plots/likelihood_scale.png
```

### Phase 18 — Replaying one drive through every filter

Phase 17's filter settings changed the closed-loop route, because the
controller followed each filter's estimate. Its RMS differences therefore
mixed estimator quality with different trajectories, stopping times and
scoring horizons. Phase 18 holds the drive fixed: record one drive per seed
with the default filter, then replay its wheel commands and LiDAR scans
through every selector and scale. A replay never commands the robot again; it
only predicts, updates and scores the filter against the recorded true poses.
Every setting therefore sees identical observations over an identical horizon.

The recording source was `scan_order` at likelihood scale 0.2 (the current
default): five seeds (0–4), 800 particles, a two-hit budget, wheel scales
0.99/1.01 and 0.02 m LiDAR range noise. Each recording is the Phase 17
reference drive, so its live metrics match that table. Replaying the source
filter with the same seed reproduced filter position RMS, maximum position
error and heading RMS exactly on every seed: the largest absolute difference
was `0.0`. That checks that the saved wheel commands, scans and true poses are
sufficient to reconstruct the scored run.

| Seed | Steps | Duration (s) | Collisions | Live filter RMS (m) |
|---:|---:|---:|---:|---:|
| 0 | 402 | 20.10 | 1 | 1.011 |
| 1 | 388 | 19.40 | 1 | 1.094 |
| 2 | 439 | 21.95 | 1 | 0.727 |
| 3 | 410 | 20.50 | 1 | 0.923 |
| 4 | 404 | 20.20 | 1 | 0.977 |

The 30 replays used one recording per seed. Paired changes compare a setting
with the source replay for the same seed.

| Selector | Scale | Replayed position RMS (m), mean ± SD | Heading RMS (°), mean ± SD | Paired RMS change vs source (m), mean ± SD | Seeds improved |
|---|---:|---:|---:|---:|---:|
| Scan order | 0.2 | 0.946 ± 0.137 | 17.50 ± 1.70 | reference | 0/5 |
| Scan order | 0.8 | 1.249 ± 0.359 | 19.82 ± 3.72 | +0.303 ± 0.406 | 1/5 |
| Scan order | 6.0 | 1.007 ± 0.267 | 16.26 ± 3.29 | +0.060 ± 0.324 | 2/5 |
| Widest pair | 0.2 | 0.665 ± 0.271 | 13.11 ± 5.45 | −0.282 ± 0.339 | 4/5 |
| Widest pair | 0.8 | 0.134 ± 0.021 | 2.79 ± 0.25 | −0.813 ± 0.121 | 5/5 |
| Widest pair | 6.0 | 0.109 ± 0.023 | 2.24 ± 0.21 | −0.837 ± 0.120 | 5/5 |

On identical inputs the widest pair at scale 6.0 still had the lowest mean
replayed RMS: `0.109` against `0.946 m` for the source, a paired change of
`−0.837 ± 0.120 m` ranging from `−0.980` to `−0.650 m`. Even at the reference
scale the widest pair lowered RMS on four of five seeds (`0.665` mean, paired
`−0.282 ± 0.339 m`); seed 2 rose by `0.065 m`. Changing the scale with
scan-order selection had no consistent direction, improving on only one of
five and two of five seeds, respectively. The Phase 17 localization gain
therefore survives the removal of route divergence.

Live and replayed RMS still differ where the live filter steered a different
route. For example, the live widest-pair run at scale 0.2 scored `0.925 m`,
but replaying that filter on the source trajectory scored `0.665 m`. The
replay isolates estimation; it says nothing about navigation success. All five
recordings collided after 19.4–22.0 s and ended 14.9–15.5 m from the goal, so
the shared horizon is a failed drive rather than a completed route.

This comparison covers one route, one recording selector and six settings at a
two-return budget. Replayed filter metrics are measured against the recorded
true poses; because the replayed estimate does not influence motion, the study
cannot test whether a filter would reach the goal. The exact reference match
is a determinism check for this simulation, not an independent measurement.

[Recorded configurations, per-seed replays and paired differences](../experiments/studies/replay_localization.json).
Reproduce the five recordings, 30 replays and plot from the saved JSON:

```bash
python scripts/replay_localization_study.py run --seeds 0 1 2 3 4 --output experiments/studies/replay_localization.json
MPLBACKEND=Agg python scripts/replay_localization_study.py plot experiments/studies/replay_localization.json --output experiments/plots/replay_localization.png
```

### 5.1 Controlled comparisons

| Experiment | Comparison | Question |
|---|---|---|
| Localization | Odometry vs particle filter | How much does filtering reduce position error? |
| Mapping and localization | SLAM vs odometry-only trajectory | How does loop closure affect trajectory and map quality? |
| Planning | A* vs Dijkstra | What are the differences in path cost and computational effort? |
| Loop closure | Before vs after verified loop closure | How much are trajectory and map estimates corrected? |
| Exploration | Greedy next-best-view vs a future alternative | What is the cost of the current greedy strategy? |
| Sensor robustness | Several sensor-noise settings | How sensitive is the system to degraded observations? |
| Recovery | Normal operation vs injected localization failure | Can the robot detect and recover from failure? |

Only compare methods under equivalent worlds, start/goal conditions, sensor
assumptions and random seeds wherever possible.

### 5.2 Robustness experiments

Test how ATLAS behaves when its assumptions are challenged:

- Increased wheel-scale error.
- Increased range noise.
- Reduced or ambiguous observations.
- Incorrect or false loop-closure candidates.
- Long corridors and geometrically repetitive environments.
- Dead ends and narrow passages.
- Large unexplored areas.
- Localization uncertainty or deliberate pose disruption.

Do not hide failures. Record them, classify them and explain the conditions that
cause them.

### 5.3 Metrics to report

Use metrics that fit each experiment. Possible measures include:

- Final position error.
- Localization RMS error.
- Absolute Trajectory Error (ATE).
- Relative Pose Error (RPE), where appropriate.
- Cross-track error.
- Path length and path cost.
- Time to goal.
- Collision count.
- Map coverage.
- Free-space classification.
- Number of viewpoints visited.
- Number of replans.
- Recovery detection and reacquisition time.
- Runtime and memory use.

For repeated runs, report mean, standard deviation, minimum and maximum, number
of runs, random seeds and relevant scenario configuration.

Use confidence intervals when the sample size and experiment design make them
meaningful. Do not imply statistical significance from a small number of runs.

### 5.4 Ablation studies

An ablation study removes or disables one part of a system to understand its
contribution.

Possible ATLAS ablations:

- Disable loop closure and compare trajectory error.
- Disable online graph correction and compare continued-drive behaviour.
- Disable particle-filter correction and compare localization drift.
- Disable map repair and compare planning outcomes.
- Disable recovery logic and measure failure consequences.
- Disable exploration gain and compare the selected viewpoints.

Each ablation should answer a specific question. Keep the world, seed and other
settings controlled.

### Definition of done

- Experiments can be rerun from documented commands.
- Configurations and seeds are saved.
- Raw results are retained in JSON or CSV.
- Plots are generated from recorded results, not manually edited numbers.
- Conclusions distinguish measured results from interpretation.

---

## 6. Workstream C — ATLAS Research Dashboard

### Objective

Build a visual interface that exposes the actual state of the simulation and
makes the system understandable without requiring someone to inspect thousands
of lines of Python.

The dashboard should be connected to the real simulation and evaluation code.
It should not be a decorative mock-up.

### Suggested views

**Environment**

- Obstacles and boundaries.
- Occupancy grid.
- Unknown, free and occupied regions.
- Robot footprint and orientation.
- Ground-truth pose, only as an evaluation overlay.
- Estimated pose used by the navigation system.

**Localization**

- Estimated position and heading.
- Particle cloud.
- Particle weights or other useful filter diagnostics.
- Odometry estimate.
- Localization uncertainty, where available.
- Ground-truth error for evaluation only.

**SLAM**

- Estimated trajectory.
- Pose-graph nodes and constraints.
- Loop-closure events.
- Scan-matching results.
- Trajectory before and after optimization.
- Rebuilt map after graph correction.

**Planning and control**

- Global path.
- Current target or waypoint.
- Robot heading.
- Controller output.
- Cross-track error.
- Replanning events.
- Obstacle-related recovery state.

**Exploration**

- Known and unknown regions.
- Candidate viewpoints.
- Selected next-best viewpoint.
- Expected information gain, if exposed by the implementation.
- Visited viewpoints.
- Coverage progress.

**Evaluation**

- Goal completion.
- Collision count.
- Navigation time.
- Localization RMS.
- ATE.
- Map coverage.
- Replanning and recovery counts.
- Benchmark results across seeds.

### Interaction ideas

Consider adding:

- Start, pause and reset.
- Scenario selection.
- Random seed selection.
- Sensor-noise configuration.
- Algorithm toggles for controlled experiments.
- Step-by-step execution.
- Playback of saved runs.
- Side-by-side experiment comparison.

Only expose controls that can be connected reliably to the actual simulation. A
playback mode based on recorded runs is a useful first version if live control
would require too much refactoring.

### Implementation principle

Keep the simulation and algorithm code independent from the dashboard. Define a
clean data interface that publishes simulation state, events and metrics.

The dashboard should consume that interface rather than duplicating navigation
logic.

### Definition of done

A person should be able to watch a run and answer:

- Where does the robot believe it is?
- What does it know about the environment?
- What is it trying to do next?
- Why did it replan?
- What changed after loop closure?
- How well did the run perform?

---

## 7. Workstream D — Mathematical and technical depth

### Objective

Make the mathematics a first-class part of the project, not an appendix that few
people read.

The existing `docs/mathematics.md` is a major asset. Improve its connection to
the implementation and experiments.

For each major subsystem, document:

1. The problem being solved.
2. The mathematical model.
3. The equations and notation.
4. Assumptions and simplifications.
5. Why the method was selected.
6. How it is implemented in ATLAS.
7. How it is tested.
8. Which experiment evaluates it.
9. What the results show.
10. Known limitations and failure cases.

Suggested technical chapters: coordinate frames, differential-drive kinematics,
numerical integration, occupancy grids, Dijkstra and A*, PID, LiDAR, odometry,
particle filters, sensor models, recovery, log-odds mapping, scan matching, pose
graphs, loop closure, online correction, exploration and evaluation methodology.

### Example: explaining Phase 11

Rather than writing only "implemented pose-graph SLAM", explain:

- Why wheel odometry accumulates error.
- What a loop closure represents.
- How scan matching estimates a relative pose.
- How pose-graph nodes and constraints represent the trajectory.
- How graph optimization adjusts pose estimates.
- Why the map must be updated after the trajectory changes.

Then present the reported result:

| Measurement | Reported value |
|---|---:|
| Odometry ATE | 0.993 m |
| SLAM ATE | 0.188 m |
| Reported ATE reduction | 81.1% |

State the scenario, seed count, metric definition and evaluation procedure beside
the result before using it as a general performance claim.

---

## 8. Workstream E — Limitations and future research

### Objective

Turn known limitations into a transparent research backlog.

The project identifies several designed-in limits:

- Greedy exploration rather than a full viewpoint tour.
- Coarse exploration lattice.
- Optimistic planning through unknown space.
- Fixed map extent.
- No dynamic obstacles in the relevant exploration setup.
- No sensor dropout in the relevant exploration setup.

### Potential future directions

**Robust repeated online SLAM**

- Repeated loop closures during a run.
- False-positive loop-closure handling.
- Robust optimization kernels.
- Rejection or rollback of harmful corrections.
- Consistency after multiple graph optimizations.

**Coverage planning**

- Viewpoint-tour planning.
- Finer viewpoint lattices.
- Travel cost versus information gain.
- Repeated or redundant observations.
- Coverage efficiency.

**Calibrated sensor models**

- Range-dependent measurement uncertainty.
- Incidence-angle effects.
- Occupancy update calibration.
- Moving-object handling.
- Sensor dropout and invalid measurements.

**Broader benchmarking**

- More environment geometries.
- More start/goal configurations.
- More random seeds.
- Standardized pass/fail thresholds.
- Runtime and memory scaling.
- Reproducible regression benchmarks.

Do not start all these directions at once. Select them based on what the
experiments reveal.

---

## 9. Workstream F — Professional project communication

### Objective

Make the project easy to discover, understand, run and evaluate.

Create five connected deliverables:

| Deliverable | Purpose |
|---|---|
| ATLAS Core | The actual robotics implementation |
| ATLAS Research Dashboard | Visual explanation and experiment inspection |
| ATLAS Benchmark Suite | Quantitative evaluation and reproducibility |
| ATLAS Technical Report | Mathematical and engineering explanation |
| ATLAS Demonstration Film | A concise visual story of the system |

### GitHub repository

Make these easy to find:

- What ATLAS is.
- What problem it investigates.
- Architecture diagram.
- Phase roadmap and current status.
- Installation instructions.
- Quick-start demo.
- Benchmark commands.
- Representative results.
- Known limitations.
- Links to the technical report and demonstration.

Keep the README concise enough to scan. Put detailed mathematics and experiment
methodology in dedicated documents.

### Technical report

Possible structure:

1. Abstract.
2. Problem statement and motivation.
3. System architecture.
4. Mathematical foundations.
5. Algorithms and implementation.
6. Experimental setup.
7. Evaluation metrics.
8. Results.
9. Ablation studies.
10. Failure analysis.
11. Limitations.
12. Future work.
13. Conclusion.
14. Reproducibility instructions.

Every reported result should be traceable to a command, configuration, seed and
output file.

### Demonstration film

Target length: approximately 3–4 minutes.

Suggested narrative:

1. **The problem:** a mobile robot must navigate while its knowledge of the
   environment and its own position is uncertain.
2. **Motion and control:** show the robot moving and following a planned path.
3. **Localization:** show odometry drift and the particle-filter estimate.
4. **Mapping and SLAM:** show the map being built, a loop closure and trajectory
   correction.
5. **Online correction:** show graph optimization during motion and continued
   navigation.
6. **Exploration:** start from an empty prior and show viewpoint selection and
   map growth.
7. **Evidence:** present measured results, benchmark conditions and limitations.

Avoid editing the film in a way that hides failures or makes the system appear
more autonomous than it is.

---

## 10. Suggested execution order

This is a proposed sequence, not a fixed deadline.

| Order | Work | Why it comes here |
|---|---|---|
| 1 | Resolve Phase 10 verification debt | Establish a trustworthy baseline (complete 19 Nov 2026) |
| 2 | Audit the current repository and documentation | Ensure public claims match the code |
| 3 | Strengthen existing benchmark experiments | Produce stronger evidence |
| 4 | Run selected ablation and robustness studies | Understand subsystem contributions and failure modes |
| 5 | Build the first Research Dashboard | Make the system observable |
| 6 | Improve the mathematics and technical report | Explain the engineering rigorously |
| 7 | Produce the demonstration film | Communicate the complete story |
| 8 | Reassess future algorithm research | Let evidence determine the next phase |
| 9 | Consider physical robot implementation | Add real-world validation when resources and design are ready |

The physical robot is a potential future extension. It is not a prerequisite for
making the current simulation a strong flagship project.

---

## 11. Definition of a successful flagship project

ATLAS will be in a strong position to serve as a flagship undergraduate
engineering project when:

- The implementation is reproducible from a clean checkout.
- The verification status of every phase is honest and clear.
- Important algorithms have controlled experiments.
- Performance claims are supported by saved data.
- Ablation studies explain the contribution of major subsystems.
- Known limitations and failure cases are documented.
- The dashboard exposes the actual simulation state.
- The mathematics is connected to the source code and experiments.
- A new person can run a demonstration using the documentation.
- The technical report and demonstration film tell the same evidence-based
  story.

This is a quality target, not a claim that all items are already complete.

---

## 12. How to explain ATLAS to other people

### Short explanation

ATLAS is a Python-based autonomous robotics research platform. It simulates how
a mobile robot can move, localize itself, build maps, plan routes, recover from
failures and explore unknown environments. Its algorithms are evaluated through
tests and repeatable benchmarks.

### Technical explanation

I developed ATLAS as a modular 2D mobile-robot autonomy stack. It combines
differential-drive kinematics, occupancy-grid planning, PID path following,
simulated LiDAR, particle-filter localization, dynamic obstacle recovery, map
repair, pose-graph SLAM, online trajectory correction and next-best-view
exploration. The project emphasizes measurable evaluation, reproducible
experiments and explicit documentation of limitations.

### The larger ambition

The goal is to investigate how the components of an autonomous robot work
together when both the robot's position and its understanding of the environment
are uncertain.

---

## 13. Guiding principles

1. **Evidence over claims.** Every important performance statement should have a
   reproducible experiment behind it.
2. **Depth over phase count.** Do not add features only to make the roadmap
   longer.
3. **Honest limitations.** Document what the system cannot yet do.
4. **Separation of concerns.** Keep algorithms, simulation, evaluation and
   visualisation cleanly separated.
5. **Reproducibility.** Save configurations, seeds, raw results and commands.
6. **Mathematical understanding.** Be able to explain why an algorithm works,
   not just how to call it.
7. **Visible engineering.** Make internal system behaviour observable.
8. **Clear communication.** Help a new reader understand the problem,
   architecture, evidence and limitations.

---

## Final direction

**Do not make ATLAS bigger just for the sake of making it bigger. Make it
deeper, more rigorous, more observable and easier to understand.**

The strongest version of ATLAS is not necessarily the one with the most
algorithms. It is the one where the implementation, mathematics, experiments,
visual demonstrations and technical explanations all support one coherent
engineering story.

That is how ATLAS can become a flagship project that can be discussed
meaningfully with professors, researchers, internship interviewers and graduate
admissions committees.
