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

First pass in `results/`; repeats and reactive runs in progress.

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
- **Challenge context:** DynaBARN results did not count toward the 2025 ranking, and sources conflict
  on 2026. A sound, comparable dynamic benchmark may be a real need for the challenge.
