import numpy as np
import math


'''
    Transform the roll pitch yaw to rotation matrix
'''
def RPY_to_Rotation(RPY_list):
    roll, pitch, yaw = RPY_list[0], RPY_list[1], RPY_list[2]
    yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R = yawMatrix * pitchMatrix * rollMatrix
    return R

# the cubes do not collide with each other in one axis only when the max of one cube is smaller than another cube
# under the same axis
def collide_check(ref_min, ref_max, _min, _max):
    if ref_min>_max or ref_max<_min:
        return False
    return True

# Detect collision in each dimension
def collision_detect(ref_corner, cuboid_corner):
    x_min = np.min(cuboid_corner[:, 0])
    x_max = np.max(cuboid_corner[:, 0])
    y_min = np.min(cuboid_corner[:, 1])
    y_max = np.max(cuboid_corner[:, 1])
    z_min = np.min(cuboid_corner[:, 2])
    z_max = np.max(cuboid_corner[:, 2])

    xref_min = np.min(ref_corner[:, 0])
    xref_max = np.max(ref_corner[:, 0])
    yref_min = np.min(ref_corner[:, 1])
    yref_max = np.max(ref_corner[:, 1])
    zref_min = np.min(ref_corner[:, 2])
    zref_max = np.max(ref_corner[:, 2])

    x_collide = collide_check(xref_min, xref_max, x_min, x_max)
    y_collide = collide_check(yref_min, yref_max, y_min, y_max)
    z_collide = collide_check(zref_min, zref_max, z_min, z_max)

    return (x_collide and y_collide and z_collide)

def Check_Collision(cuboid_ref, cuboid):
    T_matrix = np.array([[1,1,1],[1,-1,1],[-1,-1,1],[-1,1,1],[1,1,-1],[1,-1,-1],[-1,-1,-1],[-1,1,-1]])
    Projection_matrix = np.array([[1,0,0],[0,1,0],[0,0,1]])

    # Calculate rotation matrix for both cubes in respect to the base frame
    Rotation_ref = RPY_to_Rotation(cuboid_ref["Orientation"])
    Rotation_cub = RPY_to_Rotation(cuboid["Orientation"])

    # Rotate each corner point relative to cube's center (do not consider the position relative to base frame's origin)
    cuboid_corner_initial = np.array(
        [cuboid["Dimension"][0] / 2, cuboid["Dimension"][1] / 2, cuboid["Dimension"][2] / 2])
    cuboid_corner_dimension = np.tile(cuboid_corner_initial, (8, 1))
    cuboid_corner = cuboid_corner_dimension * T_matrix


    # Rotate each corner point relative to cube's center (do not consider the position relative to base frame's origin)
    ref_corner_initial = np.array(
        [cuboid_ref["Dimension"][0] / 2, cuboid_ref["Dimension"][1] / 2, cuboid_ref["Dimension"][2] / 2])
    ref_corner_dimension = np.tile(ref_corner_initial, (8, 1))
    ref_corner = ref_corner_dimension * T_matrix

    # Add origin to get the absolute cordinates of each corner point
    ref_corners = ref_corner @ Rotation_ref + np.array(cuboid_ref["Origin"])
    cub_corners = cuboid_corner @ Rotation_cub + np.array(cuboid["Origin"])
    # Uncomment below to plot current position of two cubes

    #Project each cube onto the normals of reference cube
    Projection_matrix_ref = Projection_matrix @ Rotation_ref
    cuboid_corner_new = cub_corners @ Projection_matrix_ref.T
    ref_corner_new = ref_corners @ Projection_matrix_ref.T
    Collision_or_not_ref = collision_detect(ref_corner_new,cuboid_corner_new) # check collision
    # Uncomment below to plot current position of two cubes (in cordinates of the reference cube)

    #Project each cube onto the normals of reference cube
    Projection_matrix_cub = Projection_matrix @ Rotation_cub
    cuboid_corner_new = cub_corners @ Projection_matrix_cub.T
    ref_corner_new = ref_corners @ Projection_matrix_cub.T
    Collision_or_not_cub = collision_detect(ref_corner_new,cuboid_corner_new)
    # Uncomment below to plot current position of two cubes (in cordinates of the reference cube)

    return Collision_or_not_ref and Collision_or_not_cub


def collosion_detect(cuboid_1,cuboid_2):
    result = Check_Collision(cuboid_1, cuboid_2)   #  In reference of cuboid1
    return result

# def chech_two_cuboid():
#     cuboid_1 = {"Origin": [0, 0, 0], "Orientation": [0, 0, 0], "Dimension": [2, 2, 2]}
#     cuboid_2 = {"Origin": [1.7, 1.7, 0], "Orientation": [0, 0, np.pi/4], "Dimension": [2, 2, 2]}
#     print(collosion_detect(cuboid_1,cuboid_2))



