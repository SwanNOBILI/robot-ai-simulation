from controller import Supervisor
import random
import math
import numpy as np


# World parameters
box_size = 0.1
min_distance = 2 * box_size     # between objects
min_boxes = 4
max_boxes = 10

# Supervisor Initialization
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice("supervisor_emitter")
receiver = supervisor.getDevice("supervisor_receiver")
receiver.enable(timestep)
root = supervisor.getRoot()
children = root.getField("children")
arena_node = supervisor.getFromDef("ARENA")
arena_size = arena_node.getField("floorSize").getSFVec2f()



# Get a random position in the Arena
def get_random_position():
    return [random.uniform(-arena_size[0]/2 + min_distance, arena_size[0]/2 - min_distance), 
            random.uniform(-arena_size[1]/2 + min_distance, arena_size[1]/2 - min_distance)
           ]

# Generate a non-overlapping position
def generate_non_overlapping_position(existing_positions):
    # Area bounds (assuming square arena)
    half_width = arena_size[0] / 2 - min_distance
    half_height = arena_size[1] / 2 - min_distance
    # Path between goal position & start position
    path = np.array(goal_position) - np.array(start_position)
    direction = path / np.linalg.norm(path)
    perpendicular = np.array([-direction[1], direction[0]]) # perpendicular direction (to add lateral noise)
    for _ in range(1000):   # max attempts to create a new position
        t = random.uniform(min_distance, 1-min_distance)    # avoid placing right at start/goal
        lateral_offset = random.gauss(0, 0.15)              # std-dev of spread
        center = np.array(start_position) + t*path
        new_position = center + lateral_offset*perpendicular
        # If the position is located in the area (+/- min_distance)
        if (-half_width <= new_position[0] <= half_width) and (-half_height <= new_position[1] <= half_height):
            # If the position isn't near another box (+/- min_distance)
            if all(math.sqrt((new_position[0]-ex)**2 + (new_position[1]-ey)**2) > min_distance for ex, ey in existing_positions):
                return new_position
    return None

#
def place_marker(position, color_rgb, transparency=0.5):
    r, g, b = color_rgb
    return f'''
Transform {{
  translation {position[0]} {position[1]} {0}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {r} {g} {b}
          transparency {transparency}
        }}
      }}
      geometry Box {{
        size 0.1 0.1 0.001  # flat rectangle
      }}
    }}
  ]
}}
'''

# E-puck spawn
start_position = get_random_position()
epuck_angle = random.uniform(0, 2 * math.pi)
epuck_node = supervisor.getFromDef("EPUCK")
if epuck_node:
    translation_field = epuck_node.getField("translation")
    translation_field.setSFVec3f([start_position[0], start_position[1], 0.0])
    rotation_field = epuck_node.getField("rotation")
    rotation_field.setSFRotation([0, 0, 1, epuck_angle])
    print(f"E-puck starts at ({start_position[0]:.2f}, {start_position[1]:.2f} 0.0) with angle {epuck_angle:.2f} rad")
print(f"The Robot start position is {start_position}")
children.importMFNodeFromString(-1, place_marker(start_position, (0.3, 0.3, 1)))    # light blue

# Robot goal
goal_position = get_random_position()
emitter.send(f"{goal_position[0]} {goal_position[1]}")
print(f"The Robot goal position is {goal_position}")
children.importMFNodeFromString(-1, place_marker(goal_position, (1, 0.3, 0.3)))     # light red



# Boxes spawn
positions = [start_position]    # e-puck & boxes positions
num_boxes = random.randint(min_boxes, max_boxes)
print("########################################################################################")
for i in range(num_boxes):
    box_position = generate_non_overlapping_position(positions)
    box_angle = random.uniform(0, 2 * math.pi)
    if box_position is None:
        continue
    positions.append(box_position)
    children.importMFNodeFromString(-1, f"myWoodenBox {{ translation {box_position[0]} {box_position[1]} {box_size/2} rotation 0 0 1 {box_angle} size {box_size} {box_size} {box_size} }}")
    print(f"Placed box at ({box_position[0]:.2f}, {box_position[1]:.2f} {box_size/2}) with angle {box_angle:.2f} rad")
print("########################################################################################")



# Rotate a vector from the Robot perspective to the world perspective
def rotate_vector_by_quat(v, q):
    # q * v * q^-1
    q_conj = [q[0], -q[1], -q[2], -q[3]]    # q^-1 = q_conj because q is unitary
    v_quat = [0] + v    # quaternion representation of "v" (w=0) q=(w,x,y,z)
    return quat_mult(quat_mult(q, v_quat), q_conj)[1:]  # remove w component from the result

# Quaternion (matrixes) multiplication
def quat_mult(q1, q2):
    a1, b1, c1, d1 = q1  # w1, x1, y1, z1
    a2, b2, c2, d2 = q2  # w2, x2, y2, z2
    return [
        a1*a2 - b1*b2 - c1*c2 - d1*d2,  # w_out
        a1*b2 + b1*a2 + c1*d2 - d1*c2,  # x_out
        a1*c2 - b1*d2 + c1*a2 + d1*b2,  # y_out
        a1*d2 + b1*c2 - c1*b2 + d1*a2   # z_out
    ]

# Convert (axis, theta) to quaternion matrix
def axis_angle_to_quat(axis, theta):
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)
    return [np.cos(theta/2)] + list(np.sin(theta/2) * axis)

# Convert quaternion matrix to (axis, theta)
def quat_to_axis_angle(q):
    a, b, c, d = q
    theta = 2 * np.arccos(a)
    sin_half_theta = np.sin(theta/2)
    if sin_half_theta < 1e-6:
        return [0, 1, 0, 0]
    return [b/np.sin(theta/2), c/np.sin(theta/2), d/np.sin(theta/2), theta]

# Initialization
last_q_init = None
angle_threshold = np.deg2rad(5)  # ° threshold

# Supervisor working Loop
while supervisor.step(timestep) != -1:
    # Stop the Supervisor when the Controller is finished (Robot not moving anymore)
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        if message == "finished":
            break
        receiver.nextPacket()

    # Get current E-puck position and heading
    epuck_translation = epuck_node.getField("translation").getSFVec3f()
    epuck_rotation = epuck_node.getField("rotation").getSFRotation()
    q_init = axis_angle_to_quat(epuck_rotation[:3], epuck_rotation[3])

    ########################################################################
    ### If we were using a smoother Robot motion we could skip this part ###
    # Skip camera update if robot hasn't turned significantly (useful because of Braitenberg motion)
    if last_q_init is not None:
        # Angle between two quaternions (formula: acos(2*dot(q1, q2)^2 - 1))
        dot_product = sum(a*b for a, b in zip(q_init, last_q_init))
        dot_product = np.clip(dot_product, -1.0, 1.0)   # np.clip(a, a_min, a_max) = np.minimum(a_max, np.maximum(a, a_min))
        delta_theta = 2 * np.arccos(abs(dot_product))
        if delta_theta < angle_threshold:
            q_init = last_q_init    # Keep old orientation to suppress shaking
        else:
            last_q_init = q_init    # Update memory
    else:
        last_q_init = q_init        # Initialize for first iteration
    ########################################################################

    # Camera Position
    offset_forward = 0.2; offset_up = 0.2
    forward_vec = rotate_vector_by_quat([1, 0, 0], q_init)
    camera_position = [
        epuck_translation[0] + offset_forward * forward_vec[0],
        epuck_translation[1] + offset_forward * forward_vec[1],
        epuck_translation[2] + offset_up
    ]

    # Camera orientation
    # Rotation 1: Pi around the "z" axis (axle=[0, 0, 1] & angle = Pi) -> camera in front of the Robot
    q1 = axis_angle_to_quat([0, 0, 1], np.pi)
    # Rotation 2: Angle between Camera & Robot around the "y" axis (axle=[0, 1, 0] & angle = -arctan(dz/dx))
    q2 = axis_angle_to_quat([0, 1, 0], -np.arctan2(offset_up, offset_forward))
    # Combined Rotation (Initial Rotation * Rotation 2 * Rotation 1) -> Initial Rotation to be in the Robot referencial before doing other Rotation(s)
    q_combined = quat_mult(q_init, quat_mult(q2, q1))
    # Convert final quaternion to axis-angle (Webots expects this format)
    camera_orientation = quat_to_axis_angle(q_combined)

    # Make the Webots Viewpoint focused on this Camera view
    viewpoint = supervisor.getFromDef("VIEW")
    if viewpoint:
        viewpoint.getField("position").setSFVec3f(camera_position)
        viewpoint.getField("orientation").setSFRotation(camera_orientation)