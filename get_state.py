import math

import numpy as np


def get_degree(index):
    file_path = f'./pcd/pcd{index}.npy'
    points = np.load(file_path)

    sampled_data = points
    sampled_data = sampled_data[(-0.1 < sampled_data[:, 0])]
    sampled_data = sampled_data[(sampled_data[:, 0] < 0.1)]
    sampled_data = sampled_data[(-0 < sampled_data[:, 1])]
    sampled_data = sampled_data[(sampled_data[:, 1] < 0.2)]
    # print(points)
    if len(sampled_data) == 0:
        return 90
    max_values = np.max(sampled_data, axis=0)
    min_values = np.min(sampled_data, axis=0)
    diff = max_values[2] - min_values[2]
    c = math.sqrt(diff * diff + 0.04)
    # print(c)
    ard = math.acos(1/c*0.2)
    # print(ard)
    if max_values[2] > 2:
        return 90
    deg = math.degrees(ard)
    if deg < 20:
        return 90

    return deg

def get_distance(index):
    file_path = f'./pcd/pcd{index}.npy'
    points = np.load(file_path)

    sampled_data = points
    sampled_data = sampled_data[(-0.1 < sampled_data[:, 0])]
    sampled_data = sampled_data[(sampled_data[:, 0] < 0.1)]
    sampled_data = sampled_data[(-0 < sampled_data[:, 1])]
    sampled_data = sampled_data[(sampled_data[:, 1] < 0.2)]

    mean_distance = np.mean(sampled_data[:, 2])

    return mean_distance