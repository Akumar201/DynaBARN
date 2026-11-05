# DynaBARN II: findings for the paper

Running notes, newest evidence added as it arrives. Each finding says how solid it is.
Raw data lives in `dyna_barn/ros2_pkg/ws/src/dynabarn2/data` and `.../results`.

## 1. Port of the original 60 worlds

- **Waypoints recovered exactly.** 600 compiled Gazebo 9 plugins (the only copy of the
  trajectories) were loaded and their keyframes intercepted: 6,270 keyframes, no failures.
- **Motion matches Gazebo Classic.** Compared with Gazebo 9's own `PoseAnimation` on 1.5 million
  positions across all 60 worlds: mean error at most 5 mm per world, most under 0.1 mm; 99.9th
  percentile at most 3.8 mm; largest error away from loop jumps 4.3 cm.
  Data: `data/classic_vs_harmonic_motion.csv`.
  - Limits: the Classic side is the real animation code, not a full Gazebo 9 run. Paths only, not
    robot contact behaviour. `err_max_m` is metres in 20 worlds: a single sample at a loop jump,
    one step apart between simulators, equal to the jump distance. Exclude these when reporting.
- **Deliberate change:** obstacles sit on the ground rather than half buried. Same footprint.

## 2. Problems found in the original benchmark

All reproduced from the original data or code. Worth confirming with Prof. Xiao before publishing.

- **Speed spikes.** The spline ignores timing on short segments. Peak obstacle speed is 3.9–71.9
  m/s per world; 35 of 60 exceed 10 m/s. Generator settings were about 0.5–1.5 m/s.
- **Overshoot.** Paths leave the arena, reaching 11.18 m, past the walls at 10 m.
- **Unavoidable hits.** In world 40 an obstacle touches the parked robot at 3.5 s, before any
  planner starts. ROS 1 ignored touches before the robot moved.
- **Negative path length in the score.** For dynamic worlds, ROS 1 `run.py` computes
  `GOAL_POSITION[0] - INIT_POSITION[0]` = -31 and the ROS 2 runner -20, both negative. Optimal time
  goes negative, so the score is meaningless; clip's lower bound exceeds its upper. We use the true
  20 m. Published DynaBARN scores from that code may be affected: check before claiming this.
- **Inconsistent clip in the ROS 2 repo.** The runner uses 2x-8x optimal time, as does ROS 1;
  `report_test.py` uses 4x-8x.
- **Obstacles pass through each other** in the original worlds.

## 3. DynaBARN II worlds

- **Set:** 60 worlds, 5 families (crowd, lanes, traffic, stop-and-go, mixed) × 3 levels × 4 worlds.
- **Real bounces, repeatable runs.** Obstacles are simulated offline as colliding discs and saved as
  waypoints. In Gazebo the closest pair in a hard crowd world came within 0.8 mm without
  overlapping, over 30 s, one world checked.
- **Bounded speed.** Peak 3.4 m/s against 71.9 m/s originally; the peaks are brief bounces.
- **All solvable.** Every world passes a space-time search for a route to the goal at 1 m/s.
- **Difficulty rises in every family.** `blind_collision` (chance a non-reacting straight-line robot
  is hit): easy 0.45, medium 0.82, hard 0.97. Mixed easy, at 0.74, is harder than the other easy
  cells. `earliest_arrival` barely varies (about 19-21 s), so it does not separate difficulty.
- **Design lesson:** with an even lane count the robot's route ran down a gap between lanes, so
  hard lanes scored easier than medium (0.25 vs 0.63). An odd count fixed it.

## 4. Infrastructure lessons, useful for a methods section

- `TouchPlugin` fires once per obstacle and then disables itself, so a runner that ignores early
  touches silently misses collisions. Our test caught this; the runner now keeps every touch.
- Nav2's controller needs `odom_topic` set for Clearpath's robot. Without it the controller reads
  speed 0, capping MPPI at about 0.2 m/s. This invalidated a first batch of 120 trials, discarded.
- Collision check: on the 60 original worlds all 94 collisions were within 0.32 m of a real obstacle.

## 5. Baselines

Setup: 1,440 trials. Nav2 MPPI and DWB, Jackal J100, 60 worlds, 3 repeats, 4 sets of runs. Worlds are
the unit of analysis, and intervals are 95% bootstraps over worlds. 22 trials that were missing or
never started were rerun on the same build, so no planner result was dropped. To reproduce:
`tools/analyze_results.py <package dir>`. Merged data is in `results/<tag>.csv`.

| Worlds | Obstacles | DWB success (95% CI) | MPPI success (95% CI) |
| --- | --- | --- | --- |
| Original 60 | fixed | 0.22 (0.13-0.31) | 0.21 (0.13-0.29) |
| DynaBARN II | fixed | 0.35 (0.26-0.45) | 0.39 (0.29-0.49) |
| DynaBARN II | yield | 0.71 (0.60-0.80) | 0.75 (0.65-0.84) |
| DynaBARN II | block | 0.18 (0.10-0.27) | 0.17 (0.09-0.26) |

Success by DynaBARN II level, DWB / MPPI:

| Level | Fixed | Yield | Block |
| --- | --- | --- | --- |
| Easy | 0.60 / 0.68 | 0.90 / 0.90 | 0.32 / 0.40 |
| Medium | 0.30 / 0.37 | 0.77 / 0.85 | 0.23 / 0.12 |
| Hard | 0.15 / 0.12 | 0.45 / 0.50 | 0.00 / 0.00 |

What the data supports:

- **DynaBARN II levels work.** Success falls from easy to hard for both planners in all three modes
  (hard block is at the floor, 0.00).
- **The original worlds are harder to use.** The hard dense group (40-59) sits at 0.02-0.03, so it
  cannot separate planners. 6 trials were hit before the robot moved. DynaBARN II had none.
- **Obstacle response changes scores a lot.** Paired by world against fixed motion, yield adds +0.36
  for both planners (DWB +0.272 to +0.439, MPPI +0.267 to +0.456). Block costs -0.17 for DWB (-0.250
  to -0.083) and -0.22 for MPPI (-0.300 to -0.139). Worlds moved up/down: yield 37/0 and 35/1, block
  4/19 and 1/25.

What the data does not support:

- **Planner ranking did not flip.** DWB minus MPPI is not separable from zero in any mode: fixed
  -0.039 (-0.117 to +0.039), yield -0.044 (-0.139 to +0.044), block +0.011 (-0.072 to +0.089). One
  level cell stands out, medium block at +0.117 (+0.033 to +0.200), but it is 1 of 12 uncorrected
  comparisons, so treat it as a lead to retest and not as a result.
- **World reordering is not shown.** World success correlates at 0.57-0.67 between modes, and one
  repeat correlates at 0.60-0.64 with the other two in the same mode. The two are not exact equivalents,
  because cross-mode pairs average 3 repeats on each side, so some real reordering is possible, but
  3 repeats cannot show it.
- **blind_collision is a weak predictor.** Spearman against success is -0.27 to -0.42 across modes.
  The levels work, but the metric should not be presented as a difficulty measure.

Validity limits:

- **Collisions.** All 503 collisions in fixed motion ended within 0.315 m of an obstacle, which fits
  the Jackal's half diagonal of about 0.35 m. `check_collisions.py` uses stored paths, so it cannot
  confirm reactive hits. Those gaps are bounded as expected (at most 0.23 m in yield, 1.47 m in block,
  with offsets capped at 2 m), which is a sanity check only.
- **Reactive rules are our own.** Radius 2.5 m, speed 1.0 m/s and offset cap 2 m were picked once and
  not swept or fitted to any data. Yield and block are bounding cases, not models of real agents.
- **Reactive mode drops two guarantees.** Bounced waypoints and the space-time solvability check
  apply to fixed motion only. A reactive obstacle can overlap another obstacle, and block worlds
  are not shown to be solvable. Hard block is 0.00 for both planners and may be impossible.
- **Planners are similar.** Both are classical Nav2 controllers on near-default tuning, so they are
  weak probes for a ranking flip.

## 5a. Start-time noise in the benchmark

Obstacles start at sim time 0, but the robot starts when Nav2 is ready. Across the fixed-motion
DynaBARN II trials the robot first moved between 2.6 and 10.6 s of sim time (SD 1.7 s). At about 1 m/s
each repeat meets obstacles several metres from where the last one did, so a repeat of a world is not
the same scene, and 39 of 120 world/planner pairs had mixed outcomes over 3 repeats. Any benchmark
that starts obstacles before the planner is ready has this problem, and it is worth checking whether
the original setup in Gazebo Classic does.

Fix: `sync_start:=true` holds each obstacle at its first waypoint until the robot has moved 0.1 m. In a
first test (8 worlds, 2 planners, 2 repeats) repeats mostly ended within 0.1 s of each other and 1 of
16 pairs had mixed outcomes. That is a small sample with 2 repeats, not 3, so full runs will replace it.
Results in section 5 predate this fix.

## 6. Novelty for IROS/ICRA

Based on abstracts and search results, not full papers. Treat as leads, and confirm with Prof. Xiao.

- **Difficulty metric alone is not novel.** Shi et al. (arXiv 2404.14848) propose survivability for
  dynamic worlds, with Spearman above 0.9, in a simulator without perception or control errors.
- **Reactive agents alone are not novel.** HuNavSim and others already model them.
- **Replay vs reactive has precedent elsewhere.** In driving, rankings can flip between replayed and
  reactive traffic (Hagedorn et al., arXiv 2510.14677; SimNet, arXiv 2105.12332). In social
  navigation, PopNavShift (arXiv 2609.21838) shows the pedestrian model matters.
- **The apparent gap:** no study found that tests this for lidar-based metric ground navigation such
  as BARN. This is the most promising angle, but absence from a search is not proof.
- **Our result so far:** obstacle response moved scores strongly, but DWB and MPPI did not swap places
  (section 5). "Replay flips rankings" is not supported by this data. The defensible claim today is
  smaller: replay scores depend heavily on an unstated assumption about how obstacles respond.
- **To test the stronger claim:** add planners that behave differently (learned, or one that relies
  on obstacles moving out of its way), sweep the response rules, and use more repeats.
- **Challenge context:** DynaBARN results did not count toward the 2025 ranking, and sources conflict
  on 2026. A sound, comparable dynamic benchmark may be a real need for the challenge.
