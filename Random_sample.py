import numpy as np
from Robot_construct import *
from check_collision import *

def Random_sample():

    sample_joint_config = (np.random.rand(5)-0.5) * np.pi
    robot = Robot_construct(sample_joint_config)
    if check_collision(robot):
        Random_sample()
    else:
        return sample_joint_config

