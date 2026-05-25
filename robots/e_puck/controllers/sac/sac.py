from controller import Robot
import os
import numpy as np
import time
from robots.e_puck.controllers.global_var import *
from ai.sac_agent import SAC_Agent
from robots.e_puck.eval.evaluator import Evaluator


# Initialization
MODE = os.environ.get("MODE", "train")
PI = np.pi
robot = Robot()
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)

# Get the Robot goal (from supervisor)
def get_goal_position():
    goal_position = None
    while goal_position is None:
        if receiver.getQueueLength() > 0:
            message = receiver.getString()
            gx, gy = map(float, message.strip().split())
            goal_position = np.array([gx, gy])
            receiver.nextPacket()
        robot.step(TIMESTEP)    # robot.step(TIMESTEP) advances the simulation, so that we can get the message
    return goal_position

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



# Get the Environment useful values
def get_observations(goal_position):
    sensor_values = [sensor.getValue() for sensor in sensors]
    current_position = np.array(gps.getValues()[:2])
    position_to_goal_diff = goal_position - current_position
    distance_to_goal = np.linalg.norm(position_to_goal_diff)
    north = compass.getValues()
    current_angle = (np.arctan2(north[0], north[1])) % (2*PI)
    goal_angle = np.arctan2(position_to_goal_diff[1], position_to_goal_diff[0])
    angle_to_goal = np.arctan2(np.sin(goal_angle-current_angle), np.cos(goal_angle-current_angle))
    return sensor_values, current_position, current_angle, distance_to_goal, angle_to_goal

# Reward computation
def get_simple_reward(prev_distance_to_goal, distance_to_goal, sensor_values, goal_reached):
    return 50*(prev_distance_to_goal-distance_to_goal) - 5*max(sensor_values) - 15 + 100*goal_reached # -15 -> time penalty, maybe too high

#def get_complex_reward(prev_distance_to_goal, distance_to_goal, max_sensor_values, goal_reached, total_distance_travelled, 
#                       best_distance_to_goal, no_progression_time, total_collision_time, predicted_collision_count)

# Agent object
agent = SAC_Agent(state_size=12, action_size=2, max_action=MAX_MOTOR_SPEED)

# Choose between two possible MODE
if MODE == "train":
    def format_time(seconds):
        seconds = int(seconds); hours = seconds//3600; minutes = (seconds%3600)//60; secs = seconds%60
        if hours > 0: return f"{hours}h {minutes}m {secs}s"
        elif minutes > 0: return f"{minutes}m {secs}s"
        else: return f"{secs}s"
    # Initialization
    if os.path.isfile(os.path.join(MODELS_FILE_PATH, "actor.pt")):  # load a previous model if already existing
        agent.load(MODELS_FILE_PATH)
    total_episodes = int(os.environ.get("TOTAL_EPISODES", "1"))
    start_time = time.perf_counter()
    for episode in range(total_episodes):
        print("---------------------------------------------------------------------")
        print(f"Episode {episode}/{total_episodes} [{round(100*episode/total_episodes, 2)}%]")
        episode_start = time.perf_counter()
        # New initialization for each Episode
        goal_reached = False; no_progression_time = 0.0; left_motor_speed = 0.0; right_motor_speed = 0.0
        prev_state = None
        goal_position = get_goal_position()
        initial_goal_distance = np.linalg.norm(goal_position - np.array(gps.getValues()[:2]))
        best_distance_to_goal = initial_goal_distance; prev_distance_to_goal = initial_goal_distance
        # TO DELETE
        step_count = 0
        # Main Loop
        while not goal_reached and no_progression_time < NO_PROGRESSION_TIME_LIMIT:
            # Go to the next simulation step
            robot.step(TIMESTEP)
            # Use the SAC Agent to get an action depending on the current state
            sensor_values, current_position, current_angle, distance_to_goal, angle_to_goal = get_observations(goal_position)
            state = np.array([distance_to_goal, angle_to_goal, *sensor_values, left_motor_speed, right_motor_speed])    # size(st) = 12
            action = agent.select_action(state)                                                                         # size(at) = 2
            left_motor_speed = float(action[0])
            right_motor_speed = float(action[1])
            left_motor.setVelocity(left_motor_speed)
            right_motor.setVelocity(right_motor_speed)
            # End Loop conditions
            if distance_to_goal <= GOAL_DISTANCE_TOLERANCE: goal_reached = True
            if best_distance_to_goal > distance_to_goal:
                best_distance_to_goal = distance_to_goal
                no_progression_time = 0.0
            else: no_progression_time += TIMESTEP/1000    # from ms to s
            # Compute Reward & Store into ReplayBuffer
            reward = get_simple_reward(prev_distance_to_goal, distance_to_goal, sensor_values, goal_reached)
            done = float(goal_reached or no_progression_time >= NO_PROGRESSION_TIME_LIMIT)
            if prev_state is not None:
                agent.replay_buffer.add(prev_state, action, reward, state, done)
            # Training the Agent
            if len(agent.replay_buffer) >= agent.batch_size and step_count % 10 == 0:
                #t0 = time.perf_counter()
                agent.train_step()
                #train_time = time.perf_counter() - t0
                #print(f"train_step: {train_time*1000:.1f}ms")
            step_count += 1
            # Update the previous variables
            prev_distance_to_goal = distance_to_goal
            prev_state = state
        # Update visual information
        episode_time = time.perf_counter()-episode_start; elapsed_time = time.perf_counter()-start_time
        avg_time_per_episode = elapsed_time/(episode+1); remaining_time = avg_time_per_episode*(total_episodes-episode-1)
        print(f"Episode time: {format_time(episode_time)}  |  Elapsed time: {format_time(elapsed_time)}  |  Remaining time: {format_time(remaining_time)}")
        # TO DELETE:
        print(f"Steps: {step_count} | Goal reached: {goal_reached}")
        # End of the Simulation
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        emitter.send("finished")
    # Save the model
    agent.save(MODELS_FILE_PATH)

elif MODE == "eval":
    # Initialization
    goal_position = get_goal_position()
    initial_goal_distance = np.linalg.norm(goal_position - np.array(gps.getValues()[:2]))
    best_distance_to_goal = initial_goal_distance; prev_distance_to_goal = initial_goal_distance
    left_motor_speed = 0.0; right_motor_speed = 0.0; goal_reached = False; no_progression_time = 0.0
    evaluator = Evaluator(controller="sac", timestep=TIMESTEP, maximum_motor_speed=MAX_MOTOR_SPEED, wheel_radius=WHEEL_RADIUS)
    # Load the SAC Agent previously trained
    agent.load(MODELS_FILE_PATH)
    # Main Loop
    while not goal_reached and no_progression_time < NO_PROGRESSION_TIME_LIMIT:
        # Go to the next simulation step
        robot.step(TIMESTEP)
        # Use the SAC Agent to get an action depending on the current state
        sensor_values, current_position, current_angle, distance_to_goal, angle_to_goal = get_observations(goal_position)
        state = np.array([distance_to_goal, angle_to_goal, *sensor_values, left_motor_speed, right_motor_speed])    # size(st) = 12
        action = agent.select_action(state)                                                                         # size(at) = 2
        left_motor_speed = float(action[0])
        right_motor_speed = float(action[1])
        left_motor.setVelocity(left_motor_speed)
        right_motor.setVelocity(right_motor_speed)
        # End Loop conditions
        if distance_to_goal <= GOAL_DISTANCE_TOLERANCE: goal_reached = True
        if best_distance_to_goal > distance_to_goal:
            best_distance_to_goal = distance_to_goal
            no_progression_time = 0.0
        else: no_progression_time += TIMESTEP/1000    # from ms to s
        # Update the previous variables
        prev_distance_to_goal = distance_to_goal
    # Save evaluation data
    evaluator.save(evaluator.compute_metrics(initial_goal_distance))
    # End of the Simulation
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    emitter.send("finished")
else:
    print(f"ERROR Wrong mode: {MODE}")