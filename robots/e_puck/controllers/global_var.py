import os

# FIXED VARIABLES
SENSOR_PREFIX = "ps"
NUM_DISTANCE_SENSORS = 8
WHEEL_RADIUS = 0.0205   # value given in the e-puck documentation
MAX_MOTOR_SPEED = 6.28  # value given in the e-puck documentation
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))  # Root of the project

# ADJUSTABLE VARIABLES
MODELS_FILE_PATH = os.path.join(_ROOT, "ai/models") + os.sep
TIMESTEP = 32
GOAL_DISTANCE_TOLERANCE = 0.01
NO_PROGRESSION_TIME_LIMIT = 15.0