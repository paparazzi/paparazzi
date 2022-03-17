from determine_optic_flow import determine_optic_flow
from load_data import load_set, RGB_to_BGR
from tqdm import tqdm as tqdm
from matplotlib import pyplot as plt
import numpy as np

if __name__ == "__main__":

    path = "cyberzoo_manual_flight_data_set/flight_test/Obstacles/flag/"
    dataset = load_set(path, "RGB")
    dataset = RGB_to_BGR(dataset)

    index = 0
    prev = 0

    baseline = {}

    for key, value in tqdm(dataset.items()):

        new = value

        # print(new, prev)

        if index >= 1:
            time, vector, corners = determine_optic_flow(new, prev,  50, 50, corners=None, graphics=False)
            baseline[index - 1] = [time, vector, corners]

        prev = value
        index += 1

    time_base_max = np.max(np.array([v[0] for k, v in baseline.items()]))
    comparison = {}

    # index = 0
    prev = 0
    time_vec_max = [[50, 50, time_base_max]]
    error_vec = []

    for window_x in tqdm(range(10, 13)):
        for window_y in tqdm(range(10, 13)):

            errors = {}
            time_vec_iter = []
            index = 0

            for key, value in dataset.items():

                new = value

                if index >= 1:
                    time, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, baseline[index - 1][2], graphics=False)
                    errors_iter = np.sqrt((vectors - baseline[index - 1][1])**2)
                    comparison[index - 1] = [window_x, window_y, time, vectors]
                    errors[index - 1] = errors_iter
                    time_vec_iter.append(time)

                prev = value
                index += 1

            time_vec_max.append([window_x, window_y, max(time_vec_iter)])
            error_vec.append([window_x, window_y, np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[0]), np.average(np.average(np.array([v[3] for k, v in errors.items()]), axis=0)[1])])

    time_vec_max = np.array(time_vec_max)
    error_vec = np.array(error_vec)

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

    # print(baseline[0][1][:, 0] - comparison[0][3][:, 0])
    # print(comparison[0][3])
    #
    # plt.plot([i for i in range(len(baseline[0][1][:, 0]))], np.sqrt((baseline[0][1][:, 0] - comparison[0][3][:, 0])**2))
    # plt.show()

    # Plot time to execute optical flow

    # plt.plot([i for i in range(len(comparison.items()))], (np.array([v[0] for k, v in baseline.items()])))
    # plt.plot([i for i in range(len(comparison.items()))], np.array([v[2] for k, v in comparison.items()]))
    # plt.show()

