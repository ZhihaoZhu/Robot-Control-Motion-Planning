import numpy as np
from calculate_dist import *

X= np.array([0,0,0,0,0])
Y = np.array([[2,2,2,2,2],[1,1,1,1,1]])

dist = calculate_dist(X,Y)

d = np.argsort(dist)
dist = list(dist)
dist.sort()

print(dist)
print(d)