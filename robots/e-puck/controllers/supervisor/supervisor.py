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
#epuck_position = generate_non_overlapping_position(positions)
epuck_position = (0.0, 0.0)
positions.append(epuck_position)
#epuck_angle = random.uniform(0, 2 * math.pi)
epuck_angle = 0.0

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



# Supervisor working Loop
# Camera offsets
distance = 1.0                          # Camera Distance
height = 0.5                            # Camera Height
angle = math.atan2(height, distance)    # Camera Angle = arctan(height/distance)

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
    epuck_angle = epuck_rotation[3]   # e-puck does a rotation only around the "z" axis (translation: "x" & "y" -> rotation: "z")

    x = epuck_translation[0] - distance * math.cos(epuck_angle)
    y = epuck_translation[1] - distance * math.sin(epuck_angle)
    z = epuck_translation[2] + height
    camera_position = [x, y, z]
    camera_orientation = [0, 1, 0, angle]


    viewpoint = supervisor.getFromDef("VIEW")
    if viewpoint:
        viewpoint.getField("position").setSFVec3f(camera_position)
        viewpoint.getField("orientation").setSFRotation(camera_orientation)