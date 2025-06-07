from controller import Supervisor
import random
import math
import numpy as np

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

# World parameters
minimal_box_size = 0.035                                    # height of the E-puck robot (sensors need to see boxes)
maximal_box_size = 0.1
min_distance = maximal_box_size                             # between objects
min_boxes = 5
max_boxes = 10
min_start_goal_distance = np.linalg.norm(arena_size) * 0.3  # goal seperated at least of 30% of arena size from start



# Get a random position in the Arena
def get_random_position_with_margin(margin):
    return [random.uniform(-arena_size[0]/2 + margin, arena_size[0]/2 - margin), 
            random.uniform(-arena_size[1]/2 + margin, arena_size[1]/2 - margin)]

def get_start_and_goal_position(minimal_goal_distance, margin, max_attempts=1000):
    for _ in range(max_attempts):
        start_position = get_random_position_with_margin(margin)
        goal_position = get_random_position_with_margin(margin)
        dist = np.linalg.norm(np.array(goal_position) - np.array(start_position))
        if dist >= minimal_goal_distance:
            return start_position, goal_position
    # fallback if can't find a good position
    return get_random_position_with_margin(margin), get_random_position_with_margin(margin)

# Get a random box sizes
def get_random_box_sizes():
    box_x_size = random.uniform(minimal_box_size, maximal_box_size)
    box_y_size = random.uniform(minimal_box_size, maximal_box_size)
    box_z_size = random.uniform(minimal_box_size, maximal_box_size)
    return [box_x_size, box_y_size, box_z_size]

# Generate non-overlapping position for the Boxes
def generate_non_overlapping_position_near_path(existing_boxes, new_box_size, max_attempts=10000):
    half_width = arena_size[0] / 2 - maximal_box_size
    half_height = arena_size[1] / 2 - maximal_box_size
    path_vec = np.array(goal_position) - np.array(start_position)
    path_length = np.linalg.norm(path_vec)
    direction = path_vec / path_length
    perpendicular = np.array([-direction[1], direction[0]])
    sigma_lateral = 0.04
    sx_new, sy_new, _ = new_box_size
    for _ in range(max_attempts):
        t = random.uniform(0, 1)
        d = random.gauss(0, sigma_lateral)
        base_pos = np.array(start_position) + t * path_vec
        new_position = base_pos + d * perpendicular
        if not (-half_width <= new_position[0] <= half_width and -half_height <= new_position[1] <= half_height):
            continue
        collision = False
        for ex, ey, esize in existing_boxes:
            sx_i, sy_i, _ = esize
            dx = abs(new_position[0] - ex)
            dy = abs(new_position[1] - ey)
            min_dist_x = (sx_new + sx_i) / 2 + min_distance
            min_dist_y = (sy_new + sy_i) / 2 + min_distance
            if dx < min_dist_x and dy < min_dist_y:
                collision = True
                break
        if not collision:
            return new_position.tolist()
    return None

# Place semi-transparent rectangles on the ground for the Start (blue) & Goal (red) Robot positions
def place_marker(position, color_rgb, transparency=0.5):
    r, g, b = color_rgb
    return f"""
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
        size {maximal_box_size} {maximal_box_size} 0.001  # flat rectangle
      }}
    }}
  ]
}}
"""



# E-puck spawn & goal
start_position, goal_position = get_start_and_goal_position(min_start_goal_distance, maximal_box_size)
epuck_angle = random.uniform(0, 2 * math.pi)
# Set Robot into its start position and initial angle
epuck_node = supervisor.getFromDef("EPUCK")
if epuck_node:
    translation_field = epuck_node.getField("translation")
    translation_field.setSFVec3f([start_position[0], start_position[1], 0.0])
    rotation_field = epuck_node.getField("rotation")
    rotation_field.setSFRotation([0, 0, 1, epuck_angle])
    print(f"E-puck starts at ({start_position[0]:.2f}, {start_position[1]:.2f} 0.0) with angle {epuck_angle:.2f} rad")
children.importMFNodeFromString(-1, place_marker(start_position, (0.3, 0.3, 1)))    # place a light blue marker
# Robot goal
emitter.send(f"{goal_position[0]} {goal_position[1]}")
print(f"The Robot goal position is {goal_position}")
children.importMFNodeFromString(-1, place_marker(goal_position, (1, 0.3, 0.3)))     # place a light red marker



# Boxes spawn
positions = [(start_position[0], start_position[1], [0, 0, 0]), (goal_position[0], goal_position[1], [0, 0, 0])]
num_boxes = random.randint(min_boxes, max_boxes)    # Number of boxes to place
print("########################################################################################")
for i in range(num_boxes):
    box_size = get_random_box_sizes()
    box_position = generate_non_overlapping_position_near_path(positions, box_size)
    if box_position is None:
        continue
    box_angle = random.uniform(0, 2 * math.pi)
    positions.append((box_position[0], box_position[1], box_size))
    children.importMFNodeFromString(-1, f"myWoodenBox {{ translation {box_position[0]} {box_position[1]} {box_size[2]/2} rotation 0 0 1 {box_angle} size {box_size[0]} {box_size[1]} {box_size[2]} }}")
    print(f"Placed box at ({box_position[0]:.2f}, {box_position[1]:.2f} {box_size[2]/2:.3f}) with angle {box_angle:.2f} rad and size {box_size}")
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