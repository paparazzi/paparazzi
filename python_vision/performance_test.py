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

    comparison = {}

    index = 0
    prev = 0

    for window_x in tqdm(range(5, 6)):
        for window_y in tqdm(range(5, 6)):

            for key, value in dataset.items():

                new = value

                # print(new, prev)

                if index >= 1:
                    time, vectors, corners = determine_optic_flow(new, prev, window_x, window_y, baseline[index - 1][2], graphics=False)
                    comparison[index - 1] = [window_x, window_y, time, vectors]

                prev = value
                index += 1

    # print(len(baseline))
    # print(len(comparison))
    # print(395/98)


    # print(baseline[0][1][:, 0] - comparison[0][3][:, 0])
    # print(comparison[0][3])
    #
    plt.plot([i for i in range(len(baseline[0][1][:, 0]))], np.sqrt((baseline[0][1][:, 0] - comparison[0][3][:, 0])**2))
    plt.show()

    # Plot time to execute optical flow

    time_base = np.array([v[2] for k, v in comparison.items()])
    print(time_base)

    plt.plot([i for i in range(len(comparison.items()))], (np.array([v[0] for k, v in baseline.items()])))
    plt.plot([i for i in range(len(comparison.items()))], np.array([v[2] for k, v in comparison.items()]))
    plt.show()

