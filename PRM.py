import numpy as np
from Random_sample import *
from calculate_dist import *
from local_planner import *

def PRM(n_samples, K):
    sample = Random_sample()
    samples = np.tile(sample, (n_samples,1))
    edges = np.zeros((n_samples*K,2))
    edge_length = np.zeros((n_samples*K, 1))
    n_edges = 0

    for i in range(1,n_samples):
        X = Random_sample()
        dist = calculate_dist(X, samples[:i,:])
        dist_index = np.argsort(dist)
        dist = list(dist)
        dist.sort()

        for ii in range(min(K,len(dist))):
            j = dist_index[ii]
            if local_planner(X,samples[j,:]):
                n_edges += 1
                edges[n_edges,:] = [i, j]
                edge_length[n_edges] = dist[ii]

    return samples, edges[:n_edges,:], edge_length[:n_edges]



