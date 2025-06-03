from controller import Supervisor
import random
import math
import numpy as np


# Initialization
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
receiver = supervisor.getDevice("supervisor_receiver")
receiver.enable(timestep)



# Generate a non-overlapping position
def generate_non_overlapping_position(existing_positions):
    for _ in range(1000):  # max attempts
        x = random.uniform(-arena_size/2 + box_size, arena_size/2 - box_size)
        y = random.uniform(-arena_size/2 + box_size, arena_size/2 - box_size)
        if all(((x - ex)**2 + (y - ey)**2) ** 0.5 > min_distance for ex, ey in existing_positions):
            return (x, y)
    return None



# Load parameters
arena_size = 2.0  # from RectangleArena (2x2)
box_size = 0.1
min_distance = 5 * box_size  # between boxes
min_boxes = 3
max_boxes = 8



# E-puck spawn
positions = []  # e-puck & boxes positions
epuck_position = generate_non_overlapping_position(positions)
#epuck_position = (0.0, 0.0)
positions.append(epuck_position)
epuck_angle = random.uniform(0, 2 * math.pi)
#epuck_angle = 0.0

epuck_node = supervisor.getFromDef("EPUCK")
if epuck_node:
    translation_field = epuck_node.getField("translation")
    translation_field.setSFVec3f([epuck_position[0], epuck_position[1], 0.0])
    rotation_field = epuck_node.getField("rotation")
    rotation_field.setSFRotation([0, 0, 1, epuck_angle])
    print(f"E-puck starts at ({epuck_position[0]:.2f}, {epuck_position[1]:.2f} 0.0) with angle {epuck_angle:.2f} rad")



# Boxes spawn
num_boxes = random.randint(min_boxes, max_boxes)
root = supervisor.getRoot()
children = root.getField("children")

for i in range(num_boxes):
    box_position = generate_non_overlapping_position(positions)
    box_angle = random.uniform(0, 2 * math.pi)
    if box_position is None:
        continue
    positions.append(box_position)
    children.importMFNodeFromString(-1, f"WoodenBox {{ translation {box_position[0]} {box_position[1]} {box_size/2} rotation 0 0 1 {box_angle} size {box_size} {box_size} {box_size} }}")
    print(f"Placed box at ({box_position[0]:.2f}, {box_position[1]:.2f} {box_size/2}) with angle {box_angle:.2f} rad")
print("#####################################################################################")



# 
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

    viewpoint = supervisor.getFromDef("VIEW")
    if viewpoint:
        viewpoint.getField("position").setSFVec3f(camera_position)
        viewpoint.getField("orientation").setSFRotation(camera_orientation)