# 🤖 Robot Controller Evaluation Criteria

This document defines the metrics used to evaluate and compare the performance of robot controllers, such as a basic Braitenberg-based controller versus a learned AI-based controller.

----------

## 🔢 1. Total Distance Traveled
- **Why**: Two robots may reach the goal in the same time, but one could have taken a more optimal path.
- **How to measure**: Accumulate the distance between GPS positions at each simulation step.
- **Goal**: Lower is better.

----------

## 🔁 2. Stability of Behaviour
- **Why**: Erratic motion with frequent direction or speed changes indicates instability or lack of control.
- **How to measure**:
  - Count significant changes in motor speeds (e.g., speed deltas above a threshold).
  - Count oscillatory behaviors (e.g., repeated forward-backward or side-to-side patterns).
- **Goal**: Fewer changes or oscillations indicate smoother and more stable motion.

----------

## 🧠 3. Adaptivity & Unblocking Capability
- **Why**: A good controller must recover from partial entrapment or unexpected obstacle configurations.
- **How to measure**:
  - Success rate in cluttered or adversarial environments.
  - Number of times the robot gets stuck for > *X* seconds.
  - Detection of local loops or repetitive trajectories.
- **Goal**: High success rate, low time stuck, and minimal looping.

----------

## 💡 4. Path Efficiency ("Smartness")
- **Why**: Intelligent agents should avoid obstacles proactively and follow efficient paths.
- **How to measure**:
  - Percentage of progress made along the ideal straight line path to the goal.
  - Ratio of actual path length to ideal path length.
  - Efficiency in unseen/random environments.
- **Goal**: High straight-line progress and low path-to-ideal-path ratio.

----------

## 🔀 5. Consistency Across Tests
- **Why**: Reliable controllers should yield similar results across runs.
- **How to measure**:
  - Compute standard deviation across *n* trials for:
    - Total time
    - Distance traveled
    - Number of collisions
    - Success/failure
- **Goal**: Low standard deviation indicates robust and reliable performance.

----------

## 📊 Additional Metrics
- **Collisions with obstacles**: Count or penalize for safety/reliability.
- **(Optional) Energy usage**: Relevant if energy consumption is modeled or simulated.

----------