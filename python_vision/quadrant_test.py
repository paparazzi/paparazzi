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
import time

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

# Initializing lists and dictionaries
div_baseline_right = {}
div_baseline_left = {}
div_baseline = {}
baseline_time = []

# Iterate over data set to come up with the baseline divergence
for key, value in tqdm(dataset.items()):

    # Determine how many corners are in the left and the right side of the picture
    corners = edge_detect_FAST(value, dataset_gray.get(key), max_points=100, threshold=100)
    nr_points_quad.append([len(corners[(corners[:, 1] <= 520/2)]),
                           len(corners[(corners[:, 1] > 520/2)])])

    # Update new picture
    new = value

    if index > 1:

        # Compute the optical flow for each picture in the dataset
        # Use baseline of 150 points as the ground truth

        # start = time.clock()
        time_of, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, corners=None, graphics=False, max_points=150)
        baseline_time.append(time_of)
        div_baseline[index - 1] = [vectors, corners]

        if index > 2:
            # distance in previous image
            dy = div_baseline[index - 2][1][:, 0] - div_baseline[index - 1][1][:, 0]
            dx = div_baseline[index - 2][1][:, 1] - div_baseline[index - 1][1][:, 1]
            distance_1 = np.sqrt(dx**2 + dy**2)

            # Distance in current image
            dy = div_baseline[index - 2][1][:, 0] + div_baseline[index - 2][0][:, 0] - div_baseline[index - 1][1][:, 0] - div_baseline[index - 1][0][:, 0]
            dx = div_baseline[index - 2][1][:, 1] + div_baseline[index - 2][0][:, 1] - div_baseline[index - 1][1][:, 1] - div_baseline[index - 1][0][:, 1]
            distance_2 = np.sqrt(dx**2 + dy**2)

            # If the distance is less than 1e-5 set both previous and current distance to 1
            # such that divergence becomes equal to 0
            distance_2[distance_1 < 1e-5] = 1
            distance_1[distance_1 < 1e-5] = 1

            # Compute the divergence for the left and right side of the picture
            div_sum_left = (distance_2[(corners[:, 1] <= 520/2)] - distance_1[(corners[:, 1] <= 520/2)]) / distance_1[(corners[:, 1] <= 520/2)]
            div_sum_right = (distance_2[(corners[:, 1] > 520/2)] - distance_1[(corners[:, 1] > 520/2)]) / distance_1[(corners[:, 1] > 520/2)]

            # Append the average divergence for the left side
            if len(div_sum_left) > 0:
                div_baseline_left[index - 2] = np.sum(div_sum_left) / len(div_sum_left)
            else:
                div_baseline_left[index - 2] = 0

            # Append the average divergence for the right side
            if len(div_sum_right) > 0:
                div_baseline_right[index - 2] = np.sum(div_sum_right) / len(div_sum_right)
            else:
                div_baseline_right[index - 2] = 0

            # qbaseline_time.append(time.clock() - start)

    prev = value
    index += 1

baseline_time = np.mean(np.array(baseline_time))

# Compute the chance of no points appearing in the left/right side of the frame
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

# Re-initializing variables
prev = 0
div_left_avg_error = []
div_right_avg_error = []
iter_time_total = []

# Setting lower and upper bound for the maximum number of tracked points in the frame
lower_bound = 10
upper_bound = 75

for max_points in range(lower_bound, upper_bound):

    # Initializing variables, lists and dicts for each iteration
    index = 0
    new = 0
    div_iter = {}
    div_iter_left = {}
    div_iter_right = {}
    iter_time = []

    for key, value in tqdm(dataset.items()):

        new = value

        if index > 1:

            # Calculate the optical flow in each frame

            # start = time.clock()
            time_of, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, corners=None, graphics=False, max_points=max_points)
            iter_time.append(time_of)
            div_iter[index - 1] = [vectors, corners]

        if index > 2:
            # distance in previous image
            dy = div_iter[index - 2][1][:, 0] - div_iter[index - 1][1][:, 0]
            dx = div_iter[index - 2][1][:, 1] - div_iter[index - 1][1][:, 1]
            distance_1 = np.sqrt(dx**2 + dy**2)

            # Distance in current image
            dy = div_iter[index - 2][1][:, 0] + div_iter[index - 2][0][:, 0] - div_iter[index - 1][1][:, 0] - div_iter[index - 1][0][:, 0]
            dx = div_iter[index - 2][1][:, 1] + div_iter[index - 2][0][:, 1] - div_iter[index - 1][1][:, 1] - div_iter[index - 1][0][:, 1]
            distance_2 = np.sqrt(dx**2 + dy**2)

            # If the distance is below 1e-5 set both distances to 1, such that divergence equals 0
            distance_2[distance_1 < 1e-5] = 1
            distance_1[distance_1 < 1e-5] = 1

            # Calculate divergence in the left and right half plane
            div_sum_left = (distance_2[(corners[:, 1] <= 520/2)] - distance_1[(corners[:, 1] <= 520/2)]) / distance_1[(corners[:, 1] <= 520/2)]
            div_sum_right = (distance_2[(corners[:, 1] > 520/2)] - distance_1[(corners[:, 1] > 520/2)]) / distance_1[(corners[:, 1] > 520/2)]

            # Append the average divergence for the left side
            if len(div_sum_left) > 0:
                div_iter_left[index - 2] = np.sum(div_sum_left) / len(div_sum_left)
            else:
                div_iter_left[index - 2] = 0

            # Append the average divergence for the right side
            if len(div_sum_right) > 0:
                div_iter_right[index - 2] = np.sum(div_sum_right) / len(div_sum_right)
            else:
                div_iter_right[index - 2] = 0

            # iter_time.append(time.clock() - start)

        prev = value
        index += 1

    # Calculate the RMSE of divergence for the left and right side
    div_errors_left = np.sqrt((np.array([v for k, v in div_iter_left.items()]) - np.array([v for k, v in div_baseline_left.items()]))**2)
    div_left_avg_error.append([max_points, np.average(div_errors_left)])

    div_errors_right = np.sqrt((np.array([v for k, v in div_iter_right.items()]) - np.array([v for k, v in div_baseline_right.items()]))**2)
    div_right_avg_error.append([max_points, np.average(div_errors_right)])

    # Append the average runtime over the dataset
    iter_time_total.append(np.mean(np.array(iter_time)))

# Convert lists to arrays for easier plotting
div_left_avg_error = np.array(div_left_avg_error)
div_right_avg_error = np.array(div_right_avg_error)
iter_time_total = np.array(iter_time_total)

if not os.path.isdir("Plots"):
    os.makedirs("Plots")

# Plot divergence
fig, ax = plt.subplots(2, 1)
ax[0].plot(div_left_avg_error[:, 0], div_left_avg_error[:, 1], label="left", linewidth=3)
ax[0].plot(div_right_avg_error[:, 0], div_right_avg_error[:, 1], label="right", linewidth=3)
ax[0].legend()
ax[0].set_xlabel(r"Number of points")
ax[0].set_ylabel(r"RMSE")

# Plot time
ax[1].plot([i for i in range(lower_bound, upper_bound)], iter_time_total/baseline_time, linewidth=3)
ax[1].set_xlabel(r"Number of points")
ax[1].set_ylabel(r"Relative time")
plt.tight_layout()
plt.savefig("Plots/points_comptime_subplots.pdf")
plt.close()
