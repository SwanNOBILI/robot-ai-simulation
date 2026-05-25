from controller import Robot
import numpy as np
import os
from robots.e_puck.controllers.global_var import *
from robots.e_puck.eval.evaluator import Evaluator



# Fixed Variables
if os.environ.get("MODE", "eval") == "train": raise RuntimeError("\"basic\" controller cannot be run in train mode !")
PI = np.pi
TUNED_BRAITENBERG_COEFFICIENTS = [
    [-0.942, 0.22],     # ps0 -> turn strongly to the left
    [-0.63,  0.10],     # ps1 -> turn consequently to the left
    [-0.5,   0.06],     # ps2 -> turn slightly to the left
    [ 1.3,   1.3],      # ps3 -> go strongly forward
    [ 1.3,   1.3],      # ps4 -> go strongly forward
    [ 0.06, -0.5 ],     # ps5 -> turn slightly to the right
    [ 0.10, -0.63],     # ps6 -> turn consequently to the right
    [ 0.22, -0.942]     # ps7 -> turn strongly to the right
]



# Initialization
robot = Robot()
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)

# Get the Robot goal (from supervisor)
goal_position = None
while goal_position is None:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        gx, gy = map(float, message.strip().split())
        goal_position = np.array([gx, gy])
        receiver.nextPacket()
    robot.step(TIMESTEP)    # robot.step(TIMESTEP) advances the simulation, so that we can get the message

# Get and enable 8 distance sensors
sensors = []
for s in range(NUM_DISTANCE_SENSORS):
    name = f"{SENSOR_PREFIX}{s}"
    sensor = robot.getDevice(name)
    sensor.enable(TIMESTEP)
    sensors.append(sensor)

# GPS & Compass
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(TIMESTEP)
compass.enable(TIMESTEP)

# Get motors and set to velocity control mode
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)     # set the left_motor Velocity to 0.0
right_motor.setVelocity(0.0)    # set the right_motor Velocity to 0.0
robot.step(TIMESTEP)



# Smooth velocity change for the Motors
def get_smooth_velocity(current, target, step=0.5):
    return_value = None
    if abs(current - target) < step:    # if current_velocity == (target_velocity +/- step)
        return_value = target
    elif target > current:              # if target_velocity > current_velocity
        return_value = current + step
    else:                               # target_velocity < current_velocity
        return_value = current - step
    # Speed Limitations (give values in the range of the motors)
    return np.clip(return_value, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

# Adapted Wheight depending on proximity
def proximity_weight(sensor_value, scale=70):
    # Sensor value isn't linear -> A sensor value of 500 is not twice as "dangerous" as one of 250
    """ if scale = 70
    | Input (`sensor_value`) | Output (`proximity_weight`) |
    | ---------------------- | --------------------------- |
    | 0                      | = 0.0                       |
    | 200                    | ≈ 6.4                       |
    | 400                    | ≈ 43.3                      |
    | 500                    | ≈ 106.1                     |
    """
    return np.exp(sensor_value / scale) - 1

# Get the Environment useful values
def get_observations():
    sensor_values = [sensor.getValue() for sensor in sensors]
    current_position = np.array(gps.getValues()[:2])
    position_to_goal_diff = goal_position - current_position
    distance_to_goal = np.linalg.norm(position_to_goal_diff)
    north = compass.getValues()
    current_angle = (np.arctan2(north[0], north[1])) % (2*PI)
    goal_angle = np.arctan2(position_to_goal_diff[1], position_to_goal_diff[0])
    angle_to_goal = np.arctan2(np.sin(goal_angle-current_angle), np.cos(goal_angle-current_angle))
    return sensor_values, current_position, current_angle, distance_to_goal, angle_to_goal

# Initialization
initial_goal_distance = np.linalg.norm(goal_position - np.array(gps.getValues()[:2]))
best_distance_to_goal = initial_goal_distance
left_motor_speed = 0.0; right_motor_speed = 0.0; goal_reached = False; no_progression_time = 0.0
evaluator = Evaluator(controller="basic", timestep=TIMESTEP, maximum_motor_speed=MAX_MOTOR_SPEED, wheel_radius=WHEEL_RADIUS)

# Main Loop
while not goal_reached and no_progression_time < NO_PROGRESSION_TIME_LIMIT:
    # Go to the next simulation step
    robot.step(TIMESTEP)
    # Obstacles avoidance using Braitenberg motion
    sensor_values, current_position, current_angle, distance_to_goal, angle_to_goal = get_observations()
    forward_speed = 0.0; rotation_speed = 0.0
    for s in range(NUM_DISTANCE_SENSORS):
        weight = proximity_weight(sensor_values[s])
        left_wheel_speed = TUNED_BRAITENBERG_COEFFICIENTS[s][0] * weight
        right_wheel_speed = TUNED_BRAITENBERG_COEFFICIENTS[s][1] * weight
        forward_speed += 0.5*(left_wheel_speed + right_wheel_speed)
        rotation_speed += 0.5*(right_wheel_speed - left_wheel_speed)
    # Orientation correction towards the goal
    correction_ratio = 0.75     # manage the speed of refocusing towards the target (0.0 means not refocusing towards target)
    rotation_speed += correction_ratio*angle_to_goal
    # Final brake if the Robot is close to the goal, brake_threshold = k1*goal_tolerance + k2*linear_speed
    # with linear_speed = r*0.5*(|​ωl|+|​ωr|)
    current_linear_speed = WHEEL_RADIUS*0.5*(abs(left_motor_speed) + abs(right_motor_speed))
    brake_threshold = 5*GOAL_DISTANCE_TOLERANCE + 0.5*current_linear_speed
    if distance_to_goal < brake_threshold:  # when close enough to the goal
        brake_factor = np.clip((distance_to_goal-GOAL_DISTANCE_TOLERANCE)/(brake_threshold-GOAL_DISTANCE_TOLERANCE),
                               0.0, 1.0)
        forward_speed *= brake_factor
        rotation_speed *= (1.0 + 0.5*brake_factor)   # Rotation speed will be twice (+1) faster than Forward speed
    # Get a smooth velocity (from 'current_motor_speeds' to 'desired_motor_speeds')
    left_motor_speed = get_smooth_velocity(left_motor_speed, forward_speed-rotation_speed)
    right_motor_speed = get_smooth_velocity(right_motor_speed, forward_speed+rotation_speed)
    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)
    # End Loop conditions
    if distance_to_goal <= GOAL_DISTANCE_TOLERANCE: goal_reached = True
    if best_distance_to_goal > distance_to_goal:
        best_distance_to_goal = distance_to_goal
        no_progression_time = 0.0
    else: no_progression_time += TIMESTEP/1000    # from ms to s
    # Evaluate the useful values
    evaluator.update(current_position, current_angle, best_distance_to_goal, sensor_values, (left_motor_speed, right_motor_speed), 
                     goal_reached)

# Save evaluation data
evaluator.save(evaluator.compute_metrics(initial_goal_distance))



# When the goal is reached, stop moving
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Send the information that the Controller is finished (robot isn't moving anymore) (into the channel 1, see in .wbt file)
emitter.send("finished")