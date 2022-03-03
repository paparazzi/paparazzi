import cv2
import os
from tqdm import tqdm as tqdm
import numpy as np


# This function loads one image from a specified directory
def load_img(img_path, color_format):
    img = cv2.imread(img_path)

    if color_format.upper() == "YUV":
        return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    elif color_format.upper() == "RGB":
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    else:
        raise Exception("Invalid color format, supported types are RGB and YUV")


# This function loads all images from a specified directory
def load_set(set_path, color_format):

    list_img = os.listdir(set_path)
    # images = {"img": "img_data"}
    images = {}

    for dirs in tqdm(list_img):
        images[dirs] = load_img(set_path+dirs, color_format)

    return images


# Function to filter the entire data set given as a dict (WIP)
def filter_YUV(images, y_low, y_high, u_low, u_high, v_low, v_high):
    if type(images) is dict:
        return {k: img_filter(v, y_low, y_high, u_low, u_high, v_low, v_high) for k, v in images.items()}
    else:
        return img_filter(images, y_low, y_high, u_low, u_high, v_low, v_high)


# Defines the filter to be used in filter_YUV
def img_filter(img, y_low, y_high, u_low, u_high, v_low, v_high):
    Filtered = np.zeros([img.shape[0], img.shape[1]])
    Filtered[(img[:, :, 0] <= y_high) & (img[:, :, 0] >= y_low) & (img[:, :, 1] <= u_high) & (img[:, :, 1] >= u_low)
             & (img[:, :, 2] <= v_high) & (img[:, :, 2] >= v_low)] = 1
    return Filtered


# This function rescales images (WIP)
def img_rescale(img, resize_factor):

    if type(img) is dict:
        return {k: cv2.resize(v, (int(v.shape[1] / resize_factor), int(v.shape[0] / resize_factor))) for k, v in img.items()}
    else:
        return cv2.resize(img, (int(img.shape[1] / resize_factor), int(img.shape[0] / resize_factor)))


# This function will load images from the webcam
def cam_feed(nr_frames):
    pass


if __name__ == "__main__":

    from determine_optic_flow import determine_optic_flow
    import matplotlib.pyplot as plt

    """
    path = "AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/"
    imgs = os.listdir(path)

    test = load_set(path, "YUV")

    # print(test.items())

    # print(len(test[imgs[0]]))

    rescale = img_rescale(test, 4)
    # print(rescale[imgs[0]].shape[0])

    res_filt = filter_YUV(rescale, 50, 200, 120, 130, 120, 130)
    # print(testFilter)

    print(res_filt[imgs[0]])

    plt.imshow(res_filt[imgs[0]])
    plt.show()
    """

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    ret, frame = 0, 0

    while True:
        ret_prev, frame_prev = ret, frame
        ret, frame = cap.read()
        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        # cv2.imshow('Input', frame)

        if not ret_prev == 0:
            determine_optic_flow(frame, frame_prev)

        c = cv2.waitKey(1)
        if c == 27:
            break

    cap.release()
    cv2.destroyAllWindows()