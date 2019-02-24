import numpy as np

sample_joint_config = (np.random.rand(5) - 0.5) * np.pi
samples = np.tile(sample_joint_config, (4, 1))

print(samples)