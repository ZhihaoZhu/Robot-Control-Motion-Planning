import numpy as np
from Random_sample import *

def PRM(n_samples, K):
    sample = Random_sample()
    samples = np.tile(sample, (n_samples,1))
    edges = np.zeros((n_samples*K,2))
    edge_length = np.zeros((n_samples*K, 1))
    n_edges = 0

    for i in range(1,n_samples):
        X = Random_sample()
        dist = calculate_dist(X, samples[:i,:])
        
