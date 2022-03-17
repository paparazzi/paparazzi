import cv2
import os
from tqdm import tqdm as tqdm
import numpy as np
import matplotlib.pyplot as plt


# This function loads one image from a specified directory
def load_img(img_path, color_format):
    img = cv2.imread(img_path)

    if color_format.upper() == "YUV":
        return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    elif color_format.upper() == "RGB":
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif color_format.upper() == "BGR":
        return img
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


# Function to filter
def filter_YUV(images, y_low, y_high, u_low, u_high, v_low, v_high):
    if type(images) is dict:
        return {k: img_filter(v, y_low, y_high, u_low, u_high, v_low, v_high) for k, v in images.items()}
    else:
        return img_filter(images, y_low, y_high, u_low, u_high, v_low, v_high)


# Function to filter the entire data set given as a dict (WIP)
def RGB_to_BGR(images):
    if type(images) is dict:
        return {k: cv2.cvtColor(v, cv2.COLOR_RGB2BGR) for k, v in images.items()}
    else:
        return cv2.cvtColor(images, cv2.COLOR_RGB2BGR)


# Function to filter the entire data set given as a dict (WIP)
def BGR_to_GRAY(images):
    if type(images) is dict:
        return {k: cv2.cvtColor(v, cv2.COLOR_BGR2GRAY) for k, v in images.items()}
    else:
        return cv2.cvtColor(images, cv2.COLOR_BGR2GRAY)


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


if __name__ == "__main__":

    from determine_optic_flow import determine_optic_flow
    import time

    path = "cyberzoo_manual_flight_data_set/flight_test/random_flying/"
    dataset = load_set(path, "RGB")
    dataset = RGB_to_BGR(dataset)
    index = 0
    prev = 0

    for key, value in tqdm(dataset.items()):

        new = value

        # print(new, prev)

        if index >= 1:
            determine_optic_flow(new, prev)

        prev = value
        index += 1

    # cap = cv2.VideoCapture(0)
    #
    # if not cap.isOpened():
    #     raise IOError("Cannot open webcam")
    #
    # ret, frame = 0, 0
    #
    # while True:
    #     ret_prev, frame_prev = ret, frame
    #     ret, frame = cap.read()
    #     # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    #     # cv2.imshow('Input', frame)
    #
    #     if not ret_prev == 0:
    #         determine_optic_flow(frame, frame_prev)
    #
    #     c = cv2.waitKey(1)
    #     if c == 27:
    #         break
    #
    # cap.release()
    # cv2.destroyAllWindows()
    #


