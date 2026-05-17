# 🤖 Robot Controller Evaluation Criteria

This document defines the metrics used to evaluate and compare the performance of robot controllers (e.g. Braitenberg-based `basic` vs learned `SAC`-based controller).

All metrics are computed by the `Evaluator` class (`evaluate_controller.py`) and saved as a `.json` file at the end of each simulation episode.

----------

## ⏱️ 1. Elapsed Time (`elapsed_time`)
- **What**: Total duration of the episode in seconds.
- **Why**: A faster completion indicates a more efficient controller.
- **How**: Accumulated at each simulation step (`timestep/1000` seconds per step).
- **Goal**: Lower is better, provided the goal is reached.

----------

## 📏 2. Total Distance Travelled (`total_distance_travelled`)
- **What**: Total path length covered by the robot during the episode, in meters.
- **Why**: Two robots may reach the goal in the same time but via very different paths. A shorter path indicates smarter navigation.
- **How**: Sum of Euclidean distances between consecutive GPS positions at each step.
- **Goal**: Lower is better.

----------

## 🎯 3. Goal Reached (`goal_reached`)
- **What**: Boolean indicating whether the robot successfully reached the goal position.
- **Why**: The primary success criterion. All other metrics are only meaningful in context of this flag.
- **How**: Set to `True` when `distance_to_goal < goal_distance_tolerance` (default: 25mm).
- **Goal**: `True`.

----------

## 📍 4. Best Distance to Goal (`best_distance_to_goal`)
- **What**: The closest the robot ever got to the goal during the episode, in meters.
- **Why**: Useful for evaluating episodes where the goal was not reached. A robot that got within 5cm is meaningfully better than one that never progressed.
- **How**: Running minimum of `distance_to_goal` across all steps.
- **Goal**: Lower is better; ideally below `goal_distance_tolerance`.

----------

## 💥 5. Collision Score (`collision_score`)
- **What**: Sum of the maximum infrared sensor reading recorded at each simulation step.
- **Why**: A continuous, threshold-free proxy for obstacle proximity throughout the episode. Unlike a simple collision count, it reflects both the frequency and intensity of near-obstacle situations.
- **How**: At each step, `max(ps0, ..., ps7)` is recorded and summed over the full episode.
- **Note**: This metric is sensor-based and does not capture collisions in the robot's blind spots (limited sensor coverage). See `predicted_collision_count` for a complementary kinematic-based estimate.
- **Goal**: Lower is better.

----------

## 🔒 6. Total Collision Time (`total_collision_time`)
- **What**: Total time (in seconds) during which the robot was detected as being in a collision state.
- **Why**: Distinguishes between a robot that briefly grazes a wall versus one that stays stuck against an obstacle for several seconds.
- **How**: Kinematic collision detection. At each step, the predicted position (from motor speeds and orientation via the differential drive model) is compared to the GPS-measured position. If the discrepancy exceeds `0.2 × r × ω_max × Δt`, a collision state is active.
- **Note**: This method is immune to sensor blind spots but may produce false positives due to discretization errors, especially at high speeds or sharp turns.
- **Goal**: Lower is better.

----------

## 🔢 7. Predicted Collision Count (`predicted_collision_count`)
- **What**: Number of distinct collision events detected during the episode.
- **Why**: Complements `total_collision_time`. A robot with 10 short collisions behaves differently from one with 1 long collision.
- **How**: Incremented each time the robot transitions from a non-collision state to a collision state (rising edge detection via `collision_active` flag), using the same kinematic method as above.
- **Goal**: Lower is better.

----------

## 🧭 8. Path Efficiency (`path_efficiency`)
- **What**: Ratio of the straight-line initial distance to the goal over the total distance actually travelled, expressed as a percentage.
- **Why**: Measures how "smart" the navigation was. An efficiency of 100% means the robot took a perfectly straight path.
- **How**: `path_efficiency = (initial_goal_distance / total_distance_travelled) × 100`
- **Note**: Can exceed 100% only if the robot somehow travels less than the straight-line distance (not physically possible here). Values well below 100% indicate detours caused by obstacle avoidance.
- **Goal**: Higher is better.

----------

## 📋 Summary Table

|           Metric           |   Unit   |   Goal   |
|----------------------------|----------|----------|
| `elapsed_time`             | seconds  | ↓ lower  |
| `total_distance_travelled` | meters   | ↓ lower  |
| `goal_reached`             | boolean  |   True   |
| `best_distance_to_goal`    | meters   | ↓ lower  |
| `collision_score`          | _______  | ↓ lower  |  [sensor based]
| `total_collision_time`     | seconds  | ↓ lower  |  [kinematic based]
| `predicted_collision_count`| count    | ↓ lower  |  [kinematic-based]
| `path_efficiency`          | %        | ↑ higher |

----------

## ⚠️ Limitations & Notes
- **Collision detection is dual**,it is sensor-based (`collision_score`) and kinematic-based (`total_collision_time`, `predicted_collision_count`). Neither is perfect in isolation: the sensor-based method misses blind-spot collisions, while the kinematic method may produce false positives at high speeds or during sharp turns. Both are applied identically to all controllers, ensuring fair comparison.
- **Episode termination** is triggered by either `goal_reached = True` or `no_progression_time > threshold` (no improvement in `best_distance_to_goal` for a configurable duration, set in the controller). This replaces a fixed global timeout, allowing short episodes when the robot is stuck early and longer episodes when it is still making progress.
- **Consistency across runs** is not measured within a single episode. To evaluate robustness, multiple episodes should be run and standard deviation computed across the saved `.json` files.