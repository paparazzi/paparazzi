import os

from Edge_detect import *
from load_data import load_set, RGB_to_BGR, BGR_to_GRAY
from tqdm import tqdm as tqdm
import numpy as np

if __name__ == "__main__":

    path = "cyberzoo_manual_flight_data_set/flight_test/Obstacles/orange/"
    dataset = load_set(path, "RGB")
    dataset = RGB_to_BGR(dataset)
    dataset_gray = BGR_to_GRAY(dataset)

    nr_points_quad = []

    for key, value in tqdm(dataset.items()):

        corners = edge_detect_FAST(value, dataset_gray.get(key), max_points=100, threshold=70)
        nr_points_quad.append([len(corners[(corners[:, 0] <= 240/2) & (corners[:, 1] <= 520/2)]),
                               len(corners[(corners[:, 0] > 240/2) & (corners[:, 1] <= 520/2)]),
                               len(corners[(corners[:, 0] <= 240/2) & (corners[:, 1] > 520/2)]),
                               len(corners[(corners[:, 0] > 240/2) & (corners[:, 1] > 520/2)])])

    print(np.mean(np.asarray(nr_points_quad), axis=0))
    print(np.min(np.asarray(nr_points_quad), axis=0))

    arr = np.asarray(nr_points_quad)

    print(len(arr[:, 0][arr[:, 0] == 0]) / len(os.listdir(path)), len(arr[:, 1][arr[:, 1] == 0]) / len(os.listdir(path)),
          len(arr[:, 2][arr[:, 2] == 0]) / len(os.listdir(path)), len(arr[:, 3][arr[:, 3] == 0]) / len(os.listdir(path)))

    chance_zero = 0

    for index in range(len(arr)):
        if np.any(arr[index, :] < 3):
            chance_zero += 1

    chance_zero = chance_zero / len(os.listdir(path))

    print(chance_zero)


