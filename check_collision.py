import numpy as np
# from SAT import collosion_detect

# True if collides
# check if the bounding box collides or not

def check_collision(robot):
    global obstacle_cuboid_spec
    for i in range(len(robot)):
        for j in range(len(obstacle_cuboid_spec)):
            if collosion_detect(i,j):
                return True

    return False