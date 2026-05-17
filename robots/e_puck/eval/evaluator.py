import numpy as np
import os
import json

class Evaluator:
    def __init__(self, controller="basic", timestep=32, maximum_motor_speed = 6.28, wheel_radius = 0.0205,
                 goal_distance_tolerance=0.01, initial_goal_distance=0.0):
        # Global variables
        self.controller = controller
        self.timestep = timestep
        self.maximum_motor_speed = maximum_motor_speed
        self.wheel_radius = wheel_radius
        self.initial_goal_distance = initial_goal_distance
        self.goal_distance_tolerance = goal_distance_tolerance
        # Evaluation variables
        self.elapsed_time = 0.0
        self.total_distance_travelled = 0.0
        self.goal_reached = False
        self.best_distance_to_goal = float("inf")
        self.total_collision_time = 0.0
        self.predicted_collision_count = 0
        # Internal variables
        self.no_progression_time = 0.0
        self.max_sensor_values = []
        self.collision_active = False
        self.prev_position = None

    def reset(self):
        # Evaluation variables
        self.elapsed_time = 0.0
        self.total_distance_travelled = 0.0
        self.goal_reached = False
        self.best_distance_to_goal = float("inf")
        self.total_collision_time = 0.0
        self.predicted_collision_count = 0
        # Internal variables
        self.no_progression_time = 0.0
        self.max_sensor_values = []
        self.collision_active = False
        self.prev_position = None

    def update(self, position, orientation, distance_to_goal, sensor_values, motor_speed):
        self.elapsed_time += self.timestep/1000 # from ms to s
        # Travelled distance tracking
        if self.prev_position is not None:
            delta_position = np.linalg.norm(position - self.prev_position) # used for tracking several things
            self.total_distance_travelled += delta_position
        # Objective reached tracking
        if distance_to_goal < self.goal_distance_tolerance:
            self.goal_reached = True
        # Best distance to goal & no progression time tracking
        if self.best_distance_to_goal > distance_to_goal:
            self.best_distance_to_goal = distance_to_goal
            self.no_progression_time = 0.0
        else:
            self.no_progression_time += self.timestep/1000   # from ms to s
        # Tracking the closest obstacle
        self.max_sensor_values.append(max(sensor_values))
        # Collision count & Stuck time tracking
        if self.prev_position is not None:
            # (pred_x, pred_y) ​= (prev_x+v*cos(θ)*Δt, prev_y​+v*sin(θ)*Δt), with v = r*0.5*(​ωl+ωr)
            predicted_position = np.array((self.prev_position[0]+self.wheel_radius*0.5*(np.sum(motor_speed))*np.cos(orientation)*self.timestep/1000, 
                                           self.prev_position[1]+self.wheel_radius*0.5*(np.sum(motor_speed))*np.sin(orientation)*self.timestep/1000))
            # d > [0.1,0.25]*d_max ? with d_max = r*​ω_max*Δt & [0.1,0.25] the collision sensibility rate
            if np.linalg.norm(position-predicted_position) > 0.2*self.wheel_radius*self.maximum_motor_speed*(self.timestep/1000):
                self.total_collision_time += self.timestep/1000    # from ms to s
                if not self.collision_active:
                    #print("Collision Detected !")
                    self.predicted_collision_count += 1
                    self.collision_active = True
            else:
                self.collision_active = False
        # Update previous value variables
        self.prev_position = position
    
    def compute_metrics(self):
        path_efficiency = 0.0
        if self.total_distance_travelled > 0.0:
            path_efficiency = self.initial_goal_distance / self.total_distance_travelled
        collision_score = sum(self.max_sensor_values)
        return {
            "elapsed_time": round(self.elapsed_time, 3),
            "total_distance_travelled": round(self.total_distance_travelled, 3),
            "goal_reached": self.goal_reached,
            "best_distance_to_goal": round(self.best_distance_to_goal, 3),
            "collision_score": round(collision_score),
            "total_collision_time": round(self.total_collision_time, 3),
            "predicted_collision_count": self.predicted_collision_count,
            "path_efficiency": round(100*path_efficiency, 2)
        }

    def save(self):
        data = self.compute_metrics()
        folder = "../../eval"
        prefix = self.controller
        max_index = -1
        for filename in os.listdir(folder):
            if filename.startswith(prefix + "_") and filename.endswith(".json"):
                number_part = filename[len(prefix) + 1 : -5]
                if number_part.isdigit():
                    max_index = max(max_index, int(number_part))
        with open(f"{folder}/{prefix}_{max_index+1}.json", "w") as f:
            json.dump(data, f, indent=2)