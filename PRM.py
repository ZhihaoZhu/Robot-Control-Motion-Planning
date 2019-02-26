import numpy as np
from Random_sample import *
from calculate_dist import *
from local_planner import *

def PRM(n_samples, K):
    deg_to_rad = np.pi / 180.
    start = np.array([-80. * deg_to_rad,0,0,0,0])
    end = np.array([0, 60*deg_to_rad, -75*deg_to_rad, -75*deg_to_rad, 0])
    samples = [start,end]
    edges = {}
    edge_length = {}

    for i in range(2,n_samples):
        X = Random_sample()
        samples.append(X)
        dist = calculate_dist(X, samples[:i,:])
        dist_index = np.argsort(dist)
        dist = list(dist)
        dist.sort()
        edges[X] = []
        for ii in range(min(K,len(dist))):
            j = dist_index[ii]
            if local_planner(X,samples[j,:]):
                edges[X].append(samples[j,:])
                edges[samples[j,:]].append(X)
                edge_length[(X,samples[j,:])] = dist[ii]
                edge_length[(samples[j,:],X)] = dist[ii]


    return samples, edges, edge_length



