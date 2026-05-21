from controller import Robot
import numpy as np
from robots.e_puck.controllers.global_var import NUM_DISTANCE_SENSORS, SENSOR_PREFIX, TIMESTEP



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
    robot.step()    # robot.step() advances the simulation, so that we can get the message

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
robot.step()



# Utility functions
def get_simple_reward(prev_distance_to_goal, distance_to_goal, sensor_values, goal_reached):
    return 50*(prev_distance_to_goal-distance_to_goal) - 5*max(sensor_values) - 15 + 100*goal_reached   # -15 -> time penalty, maybe too high

#def get_complex_reward(prev_distance_to_goal, distance_to_goal, max_sensor_values, goal_reached, total_distance_travelled, 
#                       best_distance_to_goal, no_progression_time, total_collision_time, predicted_collision_count)

# Main Loop






# When the goal is reached, stop moving
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Send the information that the Controller is finished (robot isn't moving anymore) (into the channel 1, see in .wbt file)
emitter.send("finished")