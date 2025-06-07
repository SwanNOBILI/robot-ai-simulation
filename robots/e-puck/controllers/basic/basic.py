from controller import Robot
import numpy as np

# FIXED VARIABLE
PI = np.pi
SENSOR_PREFIX = "ps"
NUM_DISTANCE_SENSORS = 8
BASE_SPEED = 5.0                # rad/s
ROTATION_SPEED = BASE_SPEED / 4
TUNED_BRAITENBERG_COEFFICIENTS = [
    [-0.942, 0.22],             # ps0 -> turn strongly to the left
    [-0.63,  0.10],             # ps1 -> turn strongly to the left
    [-0.5,   0.06],             # ps2 -> turn strongly to the left
    [ 1.3,   1.3],              # ps3 -> go strongly forward
    [ 1.3,   1.3],              # ps4 -> go strongly forward
    [ 0.06, -0.5 ],             # ps5 -> turn strongly to the right
    [ 0.10, -0.63],             # ps6 -> turn strongly to the right
    [ 0.22, -0.942]             # ps7 -> turn strongly to the right
]


# Initialization
robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

# Get the Robot goal (from supervisor)
goal_position = None
while goal_position is None:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        gx, gy = map(float, message.strip().split())
        goal_position = [gx, gy]
        receiver.nextPacket()
    robot.step(timestep)    # robot.step(timestep) advances the simulation, so that we can get the message

# Get and enable 8 distance sensors
sensors = []
for s in range(NUM_DISTANCE_SENSORS):
    name = f"{SENSOR_PREFIX}{s}"
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

# GPS & Compass
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(timestep)
compass.enable(timestep)

# Get motors and set to velocity control mode
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)     # set the left_motor Velocity to 0.0
right_motor.setVelocity(0.0)    # set the right_motor Velocity to 0.0



# Smooth velocity change for the Motors
def get_smooth_velocity(current, target, step=0.5):
    if abs(current - target) < step:    # if current_velocity == (target_velocity +/- step)
        return target
    return current + step if target > current else current - step

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

# Initialization
robot.step()
initial_distance = np.linalg.norm(np.array(goal_position) - np.array(gps.getValues()[:2]))
left_motor_speed = 0.0                      # current left_motor_speed
right_motor_speed = 0.0                     # current right_motor_speed
norm_position_diff = float("inf")           # current norm(position)
norm_position_threshold = 0.02              # norm(position) threshold
angle_threshold = np.deg2rad(1)             # angle (°) threshold
phase = "rotate"
i = 0                                       # loop count
idx_debugs = 30                             # used for debug

# Main loop
while not norm_position_diff < norm_position_threshold:
    # Go to the next simulation step
    robot.step()

    # Get current needed values
    current_position = gps.getValues()
    north = compass.getValues()     # in this world.wbt, "y" axis points to north (and not "x" axis)
    current_angle = (np.arctan2(north[0], north[1])) % (2 * PI)

    # Compute Robot objective
    position_diff = np.array(goal_position) - np.array(current_position[:2])    # no height consideration
    norm_position_diff = np.linalg.norm(position_diff)
    goal_angle = np.arctan2(position_diff[1], position_diff[0])
    angle_diff = np.arctan2(np.sin(goal_angle-current_angle), np.cos(goal_angle-current_angle))
    
    # Desired motor speeds (depending on Robot situation -> needs Rotation / going forward [considering Obstacles])
    if phase == "rotate":
        if abs(angle_diff) > angle_threshold:
            # Keep rotating
            if angle_diff > 0:
                desired_left_motor_speed = -ROTATION_SPEED
                desired_right_motor_speed = ROTATION_SPEED
            else:
                desired_left_motor_speed = ROTATION_SPEED
                desired_right_motor_speed = -ROTATION_SPEED
        else:
            # Stop turning (and make the Robot stationary)
            phase = "forward"
            desired_left_motor_speed = 0.0
            desired_right_motor_speed = 0.0
    elif phase == "forward":
        # Obstacles avoidance using Braitenberg logic
        sensor_values = [sensor.getValue() for sensor in sensors]
        desired_left_motor_speed = 0.0
        desired_right_motor_speed = 0.0
        # Braitenberg motion
        for s in range(NUM_DISTANCE_SENSORS):
            weight = proximity_weight(sensor_values[s])
            desired_left_motor_speed += TUNED_BRAITENBERG_COEFFICIENTS[s][0] * weight
            desired_right_motor_speed += TUNED_BRAITENBERG_COEFFICIENTS[s][1] * weight
        # Speed limitations
        desired_left_motor_speed = np.clip(desired_left_motor_speed, -BASE_SPEED, BASE_SPEED)
        desired_right_motor_speed = np.clip(desired_right_motor_speed, -BASE_SPEED, BASE_SPEED)
        # Orientation correction toward the goal
        correction_ratio = 0.75 # manage the speed of refocusing towards the target (0.0 means not refocusing towards target)
        correction = np.clip(correction_ratio*angle_diff, -ROTATION_SPEED, ROTATION_SPEED)
        desired_left_motor_speed -= correction
        desired_right_motor_speed += correction

    # Get a smooth velocity
    left_motor_speed = get_smooth_velocity(left_motor_speed, desired_left_motor_speed)
    right_motor_speed = get_smooth_velocity(right_motor_speed, desired_right_motor_speed)

    # Set motors velocity
    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)

    # Debug
    if i % idx_debugs == 0:
        #print(f"Current Motors Speed: ({left_motor_speed:5.2f},{right_motor_speed:5.2f}) | Desired Motors Speed: ({desired_left_motor_speed:5.2f}, {desired_right_motor_speed:5.2f})")
        print(f"Progression `{100*norm_position_diff/initial_distance:6.2f}%` -> ΔP = ({position_diff[0]:6.3f},{position_diff[1]:6.3f})")
        print("-----------------------------------------------------------------------------")

    # Increase loop count
    i+=1



# When the goal is reached, stop moving
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Send the information that the Controller is finished (robot isn't moving anymore) (into the channel 1, see in .wbt file)
emitter.send("finished")