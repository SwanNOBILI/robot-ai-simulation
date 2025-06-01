from controller import Robot

# FIXED VARIABLE
SENSOR_PREFIX = "ps"
NUM_DISTANCE_SENSORS = 8

# Initialization
robot = Robot()
emitter = robot.getDevice("emitter")
timestep = int(robot.getBasicTimeStep())

# Get and enable 8 distance sensors
sensor_names = [f"{SENSOR_PREFIX}{i}" for i in range(NUM_DISTANCE_SENSORS)]     # ps0, ps1, ..., ps7
sensors = [robot.getDevice(name) for name in sensor_names]
for sensor in sensors:
    sensor.enable(timestep)

# Get motors and set to velocity control mode
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))



# Initial Velocity
left_motor.setVelocity(5.0)
right_motor.setVelocity(5.0)

# Main loop
while robot.step(timestep) != -1:
    values = [sensor.getValue() for sensor in sensors]
    #print(f"Sensor values {values}\n")

    # If close to collision
    if any(val > 500 for val in values):
        break

# Final Velocity
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Send the information that the Controller is finished (robot isn't moving anymore) (into the channel 1, see in .wbt file)
emitter.send("finished")