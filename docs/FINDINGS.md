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

These runs predate the start-time fix in section 5a. Section 5b supersedes them.

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
the same scene. Check whether the original setup in Gazebo Classic has the same problem: this port
inherited the sim-time start, but it has not been measured on Classic.

Fix: `sync_start:=true` holds each obstacle at its first waypoint until the robot has moved 0.1 m.

DWB and MPPI over 3 repeats. A mixed pair is a world/planner pair whose repeats disagree on outcome.
Spread is the median finish-time range in pairs that agree.

| Runs | Mixed, unsynced | Mixed, synced | Spread, unsynced (s) | Spread, synced (s) |
| --- | --- | --- | --- | --- |
| Original worlds, fixed | 29/120 | 4/118 | 2.22 | 0.06 |
| DynaBARN II, fixed | 39/120 | 22/120 | 0.62 | 0.08 |
| DynaBARN II, block | 15/120 | 5/120 | 0.68 | 0.06 |
| DynaBARN II, yield | 30/120 | 36/120 | 0.48 | 0.23 |

Sync helps most on fixed motion and block. It does not help yield, where small timing differences
still change how robot and obstacles interact. Pairs that remain mixed in fixed motion were not traced.

## 5b. Synchronised experiment

Setup: 2,880 planned Nav2 trials with `sync_start:=true`, 2,874 valid after one refill round of void
or missing runs. Planners DWB, MPPI and RPP, which stops for obstacles rather than steering around them.
60 worlds; 3 repeats for fixed, yield 1.0 and block 1.0 and for the original worlds; 1 repeat for the
0.5 and 2.0 m/s response speeds. Reproduce with `tools/analyze_sync.py <package dir>`. These results
replace section 5 for anything planner- or world-related. Section 5 stays as the unsynced record.

| Mode | DWB (95% CI) | MPPI (95% CI) | RPP (95% CI) |
| --- | --- | --- | --- |
| Fixed | 0.33 (0.22-0.44) | 0.37 (0.26-0.49) | 0.38 (0.27-0.48) |
| Yield 0.5 m/s | 0.43 (0.32-0.57) | 0.58 (0.45-0.70) | 0.58 (0.45-0.70) |
| Yield 1.0 m/s | 0.59 (0.49-0.68) | 0.74 (0.63-0.84) | 0.68 (0.57-0.78) |
| Yield 2.0 m/s | 0.91 (0.82-0.98) | 0.95 (0.88-1.00) | 0.93 (0.86-0.98) |
| Block 0.5 m/s | 0.22 (0.12-0.33) | 0.25 (0.13-0.37) | 0.27 (0.15-0.38) |
| Block 1.0 m/s | 0.09 (0.03-0.16) | 0.09 (0.03-0.17) | 0.12 (0.05-0.20) |
| Block 2.0 m/s | 0.02 (0.00-0.05) | 0.03 (0.00-0.08) | 0.03 (0.00-0.08) |
| Original worlds, fixed | 0.25 (0.14-0.36) | 0.21 (0.11-0.32) | 0.22 (0.12-0.33) |

Findings:

- **Block does not show reordering (corrected).** Spearman against fixed motion is 0.48-0.50, but
  `tools/tie_check.py` gets the same value (null means 0.46-0.54) when block keeps fixed motion's world
  order exactly and only compresses success towards zero. No world passed only under block for any
  planner, and only 1 world-planner pair improved. Block at 1.0 m/s looks uniformly harder, not
  reordered. An earlier version of these notes claimed otherwise, and that claim is withdrawn.
- **Yield may reorder worlds, not settled.** Spearman (0.47-0.60) is below the same null (0.55-0.74,
  p <= 0.003), and 5-16 worlds per planner go from 0/3 to 3/3 passes. Within-yield repeat agreement is
  low for DWB (0.51) and RPP (0.64), so only MPPI (0.80) is clean, and drift stalls add noise here.
- **Response speed is a strong dose-response.** Success rises with yield speed and falls with block
  speed, for every planner. A replay score therefore has no single meaning without a stated obstacle
  response. All these speeds are our choices, not fitted to real agents.
- **No full ranking flip.** All planners are tied in fixed motion and in block. The clearest gap is DWB
  behind MPPI under yield: -0.150 (-0.261 to -0.044) at 1.0 m/s and -0.150 (-0.283 to -0.033) at
  0.5 m/s, gone at 2.0 m/s. It is uncorrected, and it shrinks when drift stalls (below) count as
  arrivals: -0.100 (-0.194 to 0.000) and -0.117 (-0.233 to 0.000). Treat it as suggestive only.
- **blind_collision stays weak,** -0.07 to -0.35 across modes and planners.

Validity:

- **Collisions.** 751 synced fixed-motion collisions all ended within 0.354 m of an obstacle, none at a
  wall. Reactive hits have no equivalent check, because stored paths do not give reactive positions.
- **Odometry drift stalls.** 46 of 49 synced timeouts ended within 3 m of the goal. In logged reruns
  Nav2 reported success in the odom frame while the robot was 1.3-1.6 m from the goal in truth, outside
  the 1 m radius, so it sat until timeout. These are localisation failures, not failures to get
  through obstacles. They are concentrated in yield 1.0 (27), and the unsynced DWB and MPPI runs had
  none on DynaBARN II. Why synced runs have them was not explained. 3 timeouts were not near the goal.
  The table counts stalls as failures, so success is an underestimate where stalls are common.
- **Block solvability is still unproven.** Block at 2.0 m/s is close to 0 and may be near-impossible.
- **RPP tuning.** RPP settings were picked for this robot and speed and not tuned against the worlds.

## 5c. Prediction planners and a ranking reversal

Two planners were added, using a lidar tracker that needs no simulator truth
(`scripts/obstacle_predictor.py`). On 4 worlds with `mppi_pred`, 98-100% of sampled moving tracks matched
a true obstacle, with median velocity error of 0.15-0.19 m/s. The evaluation has not been repeated at scale.

- `mppi_pred` marks swept future positions in the local costmap. It has no notion of time.
- `mppi_critic` adds `src/PredictionCritic.cc`, an MPPI critic that compares each trajectory at step k
  with each track's constant-velocity position k steps ahead. Settings are first-pick values, untuned.

All runs use `sync_start:=true` and 3 repeats on 60 worlds, 180 trials per planner and mode. After refill,
`mppi_critic` has 171 valid yield and 176 valid block trials, and `mppi_pred` has 176, 172 and 178
(fixed, yield, block). Reproduce gaps with `tools/compare_planners.py <package dir> <planner>`.

Success:

| Obstacles | DWB | MPPI | RPP | mppi_pred | mppi_critic |
| --- | --- | --- | --- | --- | --- |
| Fixed | 0.33 | 0.37 | 0.38 | 0.39 | 0.48 |
| Yield 1.0 | 0.59 | 0.74 | 0.68 | 0.75 | 0.69 |
| Block 1.0 | 0.09 | 0.09 | 0.12 | 0.11 | 0.28 |

BARN score:

| Obstacles | DWB | MPPI | RPP | mppi_pred | mppi_critic |
| --- | --- | --- | --- | --- | --- |
| Fixed | 0.160 | 0.170 | 0.180 | 0.165 | 0.183 |
| Yield 1.0 | 0.286 | 0.328 | 0.322 | 0.296 | 0.253 |
| Block 1.0 | 0.044 | 0.043 | 0.055 | 0.046 | 0.111 |

`mppi_critic` minus the other planner, paired by world (95% CI):

| Obstacles | Success vs MPPI | Score vs MPPI | Score vs RPP |
| --- | --- | --- | --- |
| Fixed | +0.106 (-0.006 to +0.217) | +0.013 (-0.037 to +0.062) | +0.003 (-0.046 to +0.052) |
| Yield 1.0 | -0.053 (-0.169 to +0.058) | -0.075 (-0.126 to -0.024) | -0.069 (-0.118 to -0.019) |
| Block 1.0 | +0.186 (+0.096 to +0.282) | +0.067 (+0.033 to +0.105) | +0.054 (+0.019 to +0.092) |

Findings:

- **The ranking reverses with obstacle response, on the BARN score.** `mppi_critic` ties MPPI and RPP on
  replay, is worse than both under yield, and better than both under block. Replay alone hides both.
- **The change in its advantage is significant.** Its success gain over MPPI falls by -0.158 (-0.275 to
  -0.042) from fixed to yield, and by -0.153 (-0.272 to -0.039) when drift stalls count as arrivals, so
  stalls do not explain it. Its score gain falls by -0.088 (-0.137 to -0.041) and rises by +0.054 (+0.003
  to +0.104) for block.
- **Timed prediction is what matters.** `mppi_pred` shows no gain, while `mppi_critic` uses the same
  tracker. Its fixed-motion success gain is suggestive: the CI touches zero as run, and clears it
  (+0.022 to +0.256) with stalls counted.
- **Prediction costs time.** Median success time is 24.4 s against 20.5 s for MPPI on fixed motion, which
  is why its success lead does not carry over to score.
- **Hard worlds do not improve:** 0.07 against 0.12 on fixed, and 0.34 against 0.58 under yield.

Why yield hurts is a hypothesis, not a result. `mppi_critic`'s yield collisions come much later (median
34 s against 11 s for MPPI), which fits a robot that hesitates around obstacles that are moving away, then
gets caught. This needs a direct test, for instance logging speed and critic cost through a trial.

Limits:

- Many comparisons were made and intervals are uncorrected, though the yield drop in success was the
  stated test before the run. The score reversal was noticed afterwards, so it needs confirming.
- One prediction design, one tuning and one set of reactive rules. Yield and block are bounding cases.
- All planners are compared on one set of worlds, with no separate holdout.
- Block solvability remains unproven.

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
- **Our result so far (section 5c):** a time-aware prediction planner ties the baselines on replay by BARN
  score, loses to them when obstacles yield, and beats them when obstacles block. This is the strongest
  candidate for the novelty claim: ranking depends on obstacle response, and replay shows neither effect.
  It differs from the first guess, since reactive motion in general does not defeat prediction.
- **Earlier work (section 5b):** the classical planners alone showed no flip, and response speed moves
  scores steadily. World reordering is possible under yield, and floor effects explain block.
- **A second, independent contribution:** the start-time fix (section 5a) cuts mixed outcomes on the
  original worlds from 29/120 to 4/118. It is cheap to adopt and may matter to the challenge itself.
- **To make the claim stronger:** confirm the reversal on new worlds and response speeds, explain why
  yield hurts, use response models grounded in real agents, and add a learned planner.
- **Challenge context:** DynaBARN results did not count toward the 2025 ranking, and sources conflict
  on 2026. A sound, comparable dynamic benchmark may be a real need for the challenge.

## 5d. Held-out confirmation (pre-registered 2026-11-08, before any trial)

New world set `worlds/dynabarn2_holdout` (generator seed 2027, 60 worlds, same families and levels; never run before).
Runs: synced start, reactive speed 1.0 m/s, planners mppi, rpp, mppi_critic, 2 repeats, block then yield.
Settings frozen: the critic parameters of section 5c, unchanged.

Predictions, by BARN score with worlds as the unit and 95% bootstrap intervals (tools/compare_planners.py):
1. Block: mppi_critic beats both MPPI and RPP (interval above zero).
2. Yield: mppi_critic is worse than both MPPI and RPP (interval below zero).
3. The critic's gain over MPPI is larger under block than under yield (interval of the difference above zero).
Any prediction that fails is reported as failed.

### 5d results (2026-11-08, 710 of 720 trials; one block and nine yield trials missing, 10 no_start excluded)

BARN score, mppi_critic minus baseline, worlds as unit, 95% bootstrap:

| Mode | vs MPPI | vs RPP |
| --- | --- | --- |
| Block | +0.002 (-0.037 to +0.037) | +0.016 (-0.009 to +0.043) |
| Yield | -0.050 (-0.097 to -0.006) | -0.023 (-0.073 to +0.031) |

Block gain minus yield gain (vs MPPI): +0.052 (-0.002 to +0.107).

1. Block, beats both: **failed.** No detectable gain over either baseline.
2. Yield, worse than both: **failed as stated.** Worse than MPPI (interval below zero), not distinguishable from RPP.
3. Larger gain under block than yield: **failed narrowly** (interval touches zero); direction as predicted.

Success, block: MPPI 0.31, critic 0.35 (+0.04, -0.05 to +0.13); yield: MPPI 0.72, critic 0.74.
The critic is still slower (median success 25.1 s vs 20.5 s under yield), which drives the yield score loss.

Caveat on comparability: under block, MPPI succeeds 0.31 on held-out worlds against 0.09 on the original 60, so the
held-out worlds may be easier under block, or the block effect differs between world sets. A fixed-motion run on the
held-out worlds is needed to tell. Either way, the section 5c block advantage did not replicate, and the reversal
cannot be claimed. What survives on both world sets: the critic loses score to MPPI under yield, mainly through time.

### 5d follow-up: fixed motion on the held-out worlds (2026-11-09, 357 of 360 trials)

Success / BARN score: MPPI 0.42 / 0.203, RPP 0.42 / 0.199, mppi_critic 0.52 / 0.215.
Critic score minus MPPI +0.004 (-0.040 to +0.048), minus RPP +0.008 (-0.031 to +0.051): tied, as on the original set.
The critic's success edge on fixed motion (+0.10) repeats on both world sets; its score does not, because it is slower.

Why block looked easy on held-out worlds: they are only slightly easier with fixed motion (MPPI 0.42 vs 0.37), but block
costs much less there. Block minus fixed success, MPPI: original -0.28 (-0.38 to -0.18), held-out -0.12 (-0.20 to -0.04);
RPP: -0.26 vs -0.14. So the size of the block effect depends on the world set, which is itself worth reporting: reactive
difficulty is not predictable from fixed-motion difficulty. Which world features drive it is not yet known.

## 5e. What makes blocking obstacles hard (2026-11-12)

**Not the software.** MPPI under block on the original 60 worlds, rerun today: 0.09 success (55 trials) against 0.10 in
the original run. Regenerating the original worlds with today's generator reproduces them byte for byte.

**Not the manifest features.** Both sets have near-identical manifest statistics (obstacle counts, speeds,
blind_collision, earliest arrival, family and level mix). Features that track the block drop in one set track it the
other way in the other set. The held-out set's smaller drop shows in every level and most families.

**The manifest metrics describe the wrong start.** `blind_collision` averages over 40 robot departure times. With the
synchronised start the robot always meets the obstacles at trajectory time 0. `tools/sync_features.py` replays each
world from time 0 against a straight-line robot, with obstacles fixed, yielding or blocking (ObstacleMover's rule).

Spearman with per-world success (MPPI and RPP averaged), original | held-out | pooled (95% CI over worlds):

| Feature | Fixed success | Block success | Block drop (worlds with fixed > 0) |
| --- | --- | --- | --- |
| blind_collision (manifest) | -0.36 / -0.59 / -0.47 | -0.12 / -0.63 / -0.41 (-0.56 to -0.24) | +0.12 / -0.45 / -0.19 |
| hit_fixed_1.0 (synced, fixed) | -0.57 / -0.68 / -0.63 | -0.39 / -0.77 / -0.63 | +0.07 / -0.63 / -0.34 |
| hit_block_1.0 (synced, block) | -0.30 / -0.70 / -0.53 | **-0.48 / -0.85 / -0.75 (-0.86 to -0.58)** | **-0.45 / -0.77 / -0.68 (-0.81 to -0.52)** |

`hit_block_1.0` = a robot driving straight at 1 m/s from time 0 is hit when obstacles block. It explains the gap
between the sets: 4/60 original worlds are block-safe (not hit) against 14/60 held-out worlds; block-safe worlds
succeed 0.94 under block, the rest 0.07. Predicted set block success from these two means: 0.125 original (actual
0.106), 0.272 held-out (actual 0.292). Within non-safe worlds a smaller gap remains (drop -0.28 vs -0.17).
Caveat: block-safe worlds are also easy with fixed motion (0.96); the metric mainly separates trivially open worlds.
Found after looking at the data, so it is confirmed below on a third world set.

### 5f. Pre-registered confirmation (written 2026-11-12, before any trial)

World set `worlds/dynabarn2_confirm` (generator seed 2028, 60 worlds, never run). `hit_block_1.0` computed before
running (`worlds/dynabarn2_confirm/sync_features.csv`): 11 block-safe worlds.
Runs: synced start, MPPI and RPP, fixed and block (1.0 m/s), 2 repeats. Success averaged over MPPI and RPP per world.

1. Spearman of `hit_block_1.0` with block success is negative, 95% interval below zero, and more negative than
   `blind_collision`'s.
2. Block-safe worlds: mean block success at least 0.80. Other worlds: at most 0.20.
3. Set block success within 0.08 of the predicted 0.228 (stratum means 0.944 / 0.067 from sections 5d-5e).
