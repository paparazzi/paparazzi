"""
Computational effort and RMSE determination for divergence calculations based on number of points
Feasibility of calculating the divergence for local areas within the image
"""

import os
from Edge_detect import *
from load_data import load_set, RGB_to_BGR, BGR_to_GRAY
from tqdm import tqdm as tqdm
import numpy as np
from determine_optic_flow import determine_optic_flow
from matplotlib import pyplot as plt
import seaborn as sns

# Plot style
sns.set_style("whitegrid")
sns.set_context("paper", font_scale=1.6)

# Loading and pre-processing data set
path = "cyberzoo_manual_flight_data_set/flight_test/Obstacles/orange/"
dataset = load_set(path, "RGB")
dataset = RGB_to_BGR(dataset)
dataset_gray = BGR_to_GRAY(dataset)

# Initialize variables
nr_points_quad = []
index = 0
prev = None
window_x, window_y = 50, 50

div_baseline_right = {}
div_baseline_left = {}
div_baseline = {}
baseline_time = []

# Iterate over data set
for key, value in tqdm(dataset.items()):

    #
    corners = edge_detect_FAST(value, dataset_gray.get(key), max_points=100, threshold=70)
    nr_points_quad.append([len(corners[(corners[:, 1] <= 520/2)]),
                           len(corners[(corners[:, 1] > 520/2)])])

    new = value

    if index > 1:
        time, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, corners=None, graphics=False, max_points=150)
        div_baseline[index - 1] = [vectors, corners]
        baseline_time.append(time)

        if index > 2:
            # distance in previous image
            dy = div_baseline[index - 2][1][:, 0] - div_baseline[index - 1][1][:, 0]
            dx = div_baseline[index - 2][1][:, 1] - div_baseline[index - 1][1][:, 1]
            distance_1 = np.sqrt(dx**2 + dy**2)

            # Distance in current image
            dy = div_baseline[index - 2][1][:, 0] + div_baseline[index - 2][0][:, 0] - div_baseline[index - 1][1][:, 0] - div_baseline[index - 1][0][:, 0]
            dx = div_baseline[index - 2][1][:, 1] + div_baseline[index - 2][0][:, 1] - div_baseline[index - 1][1][:, 1] - div_baseline[index - 1][0][:, 1]
            distance_2 = np.sqrt(dx**2 + dy**2)

            distance_2[distance_1 < 1e-5] = 1
            distance_1[distance_1 < 1e-5] = 1

            div_sum_left = (distance_2[(corners[:, 1] <= 520/2)] - distance_1[(corners[:, 1] <= 520/2)]) / distance_1[(corners[:, 1] <= 520/2)]
            div_sum_right = (distance_2[(corners[:, 1] > 520/2)] - distance_1[(corners[:, 1] > 520/2)]) / distance_1[(corners[:, 1] > 520/2)]

            if len(div_sum_left) > 0:
                div_baseline_left[index - 2] = np.sum(div_sum_left) / len(div_sum_left)
            else:
                div_baseline_left[index - 2] = 0

            if len(div_sum_right) > 0:
                div_baseline_right[index - 2] = np.sum(div_sum_right) / len(div_sum_right)
            else:
                div_baseline_right[index - 2] = 0

    prev = value
    index += 1

baseline_time = np.mean(np.array(baseline_time))

print(np.mean(np.asarray(nr_points_quad), axis=0))
print(np.min(np.asarray(nr_points_quad), axis=0))

arr = np.asarray(nr_points_quad)

print(len(arr[:, 0][arr[:, 0] == 0]) / len(os.listdir(path)), len(arr[:, 1][arr[:, 1] == 0]) / len(os.listdir(path)))

chance_zero = 0

for index in range(len(arr)):
    if np.any(arr[index, :] < 3):
        chance_zero += 1

chance_zero = chance_zero / len(os.listdir(path))

print(chance_zero)

prev = 0
div_left_avg_error = []
div_right_avg_error = []
iter_time_total = []

lower_bound = 10
upper_bound = 75

for max_points in range(lower_bound, upper_bound):

    index = 0
    new = 0
    div_iter = {}
    div_iter_left = {}
    div_iter_right = {}
    iter_time = []

    for key, value in tqdm(dataset.items()):

        new = value

        if index > 1:
            time, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, corners=None, graphics=False, max_points=max_points)
            div_iter[index - 1] = [vectors, corners]
            iter_time.append(time)

        if index > 2:
            # distance in previous image
            dy = div_iter[index - 2][1][:, 0] - div_iter[index - 1][1][:, 0]
            dx = div_iter[index - 2][1][:, 1] - div_iter[index - 1][1][:, 1]
            distance_1 = np.sqrt(dx**2 + dy**2)

            # Distance in current image
            dy = div_iter[index - 2][1][:, 0] + div_iter[index - 2][0][:, 0] - div_iter[index - 1][1][:, 0] - div_iter[index - 1][0][:, 0]
            dx = div_iter[index - 2][1][:, 1] + div_iter[index - 2][0][:, 1] - div_iter[index - 1][1][:, 1] - div_iter[index - 1][0][:, 1]
            distance_2 = np.sqrt(dx**2 + dy**2)

            distance_2[distance_1 < 1e-5] = 1
            distance_1[distance_1 < 1e-5] = 1

            div_sum_left = (distance_2[(corners[:, 1] <= 520/2)] - distance_1[(corners[:, 1] <= 520/2)]) / distance_1[(corners[:, 1] <= 520/2)]
            div_sum_right = (distance_2[(corners[:, 1] > 520/2)] - distance_1[(corners[:, 1] > 520/2)]) / distance_1[(corners[:, 1] > 520/2)]

            if len(div_sum_left) > 0:
                div_iter_left[index - 2] = np.sum(div_sum_left) / len(div_sum_left)
            else:
                div_iter_left[index - 2] = 0

            if len(div_sum_right) > 0:
                div_iter_right[index - 2] = np.sum(div_sum_right) / len(div_sum_right)
            else:
                div_iter_right[index - 2] = 0


        prev = value
        index += 1

    div_errors_left = np.sqrt((np.array([v for k, v in div_iter_left.items()]) - np.array([v for k, v in div_baseline_left.items()]))**2)
    div_left_avg_error.append([max_points, np.average(div_errors_left)])

    div_errors_right = np.sqrt((np.array([v for k, v in div_iter_right.items()]) - np.array([v for k, v in div_baseline_right.items()]))**2)
    div_right_avg_error.append([max_points, np.average(div_errors_right)])

    iter_time_total.append(np.mean(np.array(iter_time)))

div_left_avg_error = np.array(div_left_avg_error)
div_right_avg_error = np.array(div_right_avg_error)
iter_time_total = np.array(iter_time_total)

# Plot divergence
plt.plot(div_left_avg_error[:, 0], div_left_avg_error[:, 1], label="left", linewidth=3)
plt.plot(div_right_avg_error[:, 0], div_right_avg_error[:, 1], label="right", linewidth=3)
plt.legend()
plt.xlabel(r"Number of points")
plt.ylabel(r"RMSE")
plt.title(r"RMSE of divergence as a function of points")
plt.tight_layout()
plt.savefig("Plots/points_error.pdf")
plt.close()

# Plot time
plt.plot([i for i in range(lower_bound, upper_bound)], iter_time_total/baseline_time, linewidth=3)
plt.xlabel(r"Number of points")
plt.ylabel(r"Relative time")
plt.title(r"Relative time to compute optic flow")
plt.tight_layout()
plt.savefig("Plots/points_comptime.pdf")
plt.close()
