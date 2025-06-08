import numpy as np
import json

class Evaluator:
    def __init__(self, controller="basic", goal_position=(0, 0), goal_tolerance=0.005, stuck_threshold=2.5,
                 speed_delta_threshold=1.0, collision_threshold=300):
        # Init from args
        self.controller = controller
        self.goal = np.array(goal_position)
        self.goal_tolerance = goal_tolerance
        self.stuck_threshold = stuck_threshold              # seconds
        self.speed_delta_threshold = speed_delta_threshold  # rad/s
        self.collision_threshold = collision_threshold
        # Internal state
        self.positions = []
        self.prev_speed = None
        self.last_position = None
        self.elapsed_time = 0.0
        self.collision_active = False
        # Output values
        self.total_distance = 0.0
        self.goal_reached = False
        self.stability_issues = 0
        self.stuck_counter = 0
        self.collision_counter = 0

    def update(self, position, motor_speeds, timestep, proximity_values):
        self.elapsed_time += timestep / 1000.0  # ms -> seconds
        pos = np.array(position[:2])
        self.positions.append(pos)
        # Goal detection
        if not self.goal_reached and np.linalg.norm(pos - self.goal) <= self.goal_tolerance:
            self.goal_reached = True
        # Distance tracking
        if self.last_position is not None:
            dist = np.linalg.norm(pos - self.last_position)
            self.total_distance += dist
        self.last_position = pos
        # Stability check
        if self.prev_speed is not None:
            dl = abs(motor_speeds[0] - self.prev_speed[0])
            dr = abs(motor_speeds[1] - self.prev_speed[1])
            if dl > self.speed_delta_threshold or dr > self.speed_delta_threshold:
                self.stability_issues += 1
        self.prev_speed = motor_speeds
        # Stuck detection
        if len(self.positions) >= int(self.stuck_threshold * 1000 / timestep):
            recent_positions = self.positions[-int(self.stuck_threshold * 1000 / timestep):]
            movement = np.linalg.norm(recent_positions[-1] - recent_positions[0])
            if movement < 0.005:
                self.stuck_counter += 1
        # Collision detection
        if max(proximity_values) > self.collision_threshold:
            if not self.collision_active:
                self.collision_counter += 1
                self.collision_active = True
        else:
            self.collision_active = False

    def compute_metrics(self):
        if not self.positions:
            return {}
        start = self.positions[0]
        ideal_distance = np.linalg.norm(self.goal - start)
        path_efficiency = 100 * (ideal_distance / self.total_distance if self.total_distance > 0 and self.goal_reached else 0.0)
        return {
            "total_distance": round(self.total_distance, 2),
            "duration": round(self.elapsed_time, 3),
            "path_efficiency": round(path_efficiency, 2),
            "goal_reached": self.goal_reached,
            "stability_issues": self.stability_issues,
            "stuck_count": self.stuck_counter,
            "collisions": self.collision_counter
        }

    def save(self):
        data = self.compute_metrics()
        save_path = f"../eval/{self.controller}_{data.get('path_efficiency'):.2f}.json"
        with open(save_path, "w") as f:
            json.dump(data, f, indent=2)