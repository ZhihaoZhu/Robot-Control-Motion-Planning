import numpy as np
import scipy.spatial.distance as ssd

def calculate_dist(X, sample_list):
    dist = np.zeros(sample_list.shape[0])
    for i in range(sample_list.shape[0]):
        dist[i] = ssd.euclidean(X, sample_list[i])
    return dist