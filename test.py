import numpy as np
from calculate_dist import *

X= np.array([0,0,0,0,0])
Y = np.array([[2,2,2,2,2],[1,1,1,1,1]])
delta = X-Y
dd = np.sum(abs(delta))

print(dd)
n_samples = np.ceil(np.sum(abs(delta)) / 0.017).astype(int)

print(2/5)

print(n_samples)
