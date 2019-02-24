import numpy as np
from Robot_construct import *

def local_planner(X, Y):
    delta = X-Y
    n_samples = np.ceil(np.sum(abs(delta)) / 0.03).astype(int)
    for i in range(n_samples):
        sample = Y + (i/n_samples)*delta
        

    return