"""
Computational effort and RMSE determination for optical flow calculations based on varying window size
"""

from determine_optic_flow import determine_optic_flow
from load_data import load_set, RGB_to_BGR
from tqdm import tqdm as tqdm
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns

# Set plotting style
sns.set_style("whitegrid")
sns.set_context("paper", font_scale=1.6)

# Load images and pre-process data set
path = "cyberzoo_manual_flight_data_set/flight_test/Obstacles/orange/"
dataset = load_set(path, "RGB")
dataset = RGB_to_BGR(dataset)

# Initialize some variables
index = 0
prev = 0
baseline = {}

# Iterate over all images
for key, value in tqdm(dataset.items()):

    new = value

    if index >= 1:
        # Calculate optic flow and store time to execute, optic flow vectors and used corners in dict
        # A window size of 50x50 is used as a "ground truth" to compare the smaller window size results with later
        time, vector, corners = determine_optic_flow(new, prev,  50, 50, corners=None, graphics=False)
        baseline[index - 1] = [time, vector, corners]

    prev = value
    index += 1

# Determine the maximum time to execute from the baseline data
time_base_max = np.max(np.array([v[0] for k, v in baseline.items()]))

# Re-initialize variables
prev = 0
time_vec_max = [[50, 50, time_base_max]]
error_vec = []
time_vec_square = []
error_vec_square = []

# Lower and upper bound for the window size
lower_bound = 15
upper_bound = 50

# Iterate over dataset given different combinations of window sizes
for window_x in tqdm(range(lower_bound, upper_bound)):
    for window_y in tqdm(range(lower_bound, upper_bound)):

        # Initialize variables
        errors = {}
        time_vec_iter = []
        index = 0

        for key, value in dataset.items():

            new = value

            if index >= 1:
                # To keep the tracked features consistent the corners as established in the baseline dataset are re-used
                # The RMSE and computational time are saved for later comparison
                time, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, baseline[index - 1][2], graphics=False)

                # RMSE error between baseline and current window size
                errors[index - 1] = np.sqrt((vectors - baseline[index - 1][1])**2)
                time_vec_iter.append(time)

            prev = value
            index += 1

        # Store the maximum computation time and average error for each window size combination
        time_vec_max.append([window_x, window_y, max(time_vec_iter)])
        error_vec.append([window_x, window_y, np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[0]), np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[1])])

        # Single out square windows
        if window_y == window_x:
            time_vec_square.append(max(time_vec_iter))
            error_vec_square.append([np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[0]), np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[1])])

# Convert to numpy arrays for easier slicing
time_vec_max = np.array(time_vec_max)
error_vec = np.array(error_vec)
time_vec_square = np.array(time_vec_square)
error_vec_square = np.array(error_vec_square)

# Plotting for square windows only
plt.plot([i for i in range(lower_bound, upper_bound)], error_vec_square[:, 0], label="X", linewidth=3)
plt.plot([i for i in range(lower_bound, upper_bound)], error_vec_square[:, 1], label="Y", linewidth=3)
plt.legend()
plt.xlabel("Window size")
plt.ylabel("RMSE")
plt.title("RMSE of optical flow as a function of window size")
plt.tight_layout()
plt.savefig("Plots/window_error.pdf")
plt.close()

plt.plot([i for i in range(lower_bound, upper_bound)], time_vec_square/time_base_max, linewidth=3)
plt.xlabel("Window size")
plt.ylabel("Relative time")
plt.title("Computational time as a function of window size")
plt.tight_layout()
plt.savefig("Plots/window_comptime.pdf")
plt.close()

# Plotting also for asymmetric windows
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(time_vec_max[:, 0], time_vec_max[:, 1], time_vec_max[:, 2])
ax.set_title("time vs window size")
plt.show()

fig2 = plt.figure()
ax2 = fig2.add_subplot(projection='3d')
ax2.scatter(error_vec[:, 0], error_vec[:, 1], error_vec[:, 2])
ax2.set_title("Errors in x")
plt.show()

fig3 = plt.figure()
ax3 = fig3.add_subplot(projection='3d')
ax3.scatter(error_vec[:, 0], error_vec[:, 1], error_vec[:, 3])
ax3.set_title("Errors in y")
plt.show()
