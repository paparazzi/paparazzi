"""
Data loading and processing functions
"""

import cv2
import os
from tqdm import tqdm as tqdm
import numpy as np


# This function loads one image from a specified directory and returns it with a specific color format
def load_img(img_path, color_format):

    """
    :param img_path: directory of image to be loaded
    :param color_format: color format to be outputted

    :return: opencv2 type image
    """

    img = cv2.imread(img_path)

    if color_format.upper() == "YUV":
        return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    elif color_format.upper() == "RGB":
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif color_format.upper() == "BGR":
        return img
    else:
        raise Exception("Invalid color format, supported types are RGB, BGR and YUV")


# This function loads all images from a specified directory
def load_set(set_path, color_format):

    """
    :param set_path: directory containing only images
    :param color_format: color format to be outputted

    :return: Dictionary with image name as keys and opencv images as items
    """

    list_img = os.listdir(set_path)
    images = {}

    for dirs in tqdm(list_img):

        # Check if the file is an image
        if dirs.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp')):
            images[dirs] = load_img(set_path+dirs, color_format)
        else:
            continue

    return images


# Function to filter YUV type images from the data set
def filter_YUV(images, y_low, y_high, u_low, u_high, v_low, v_high):

    """
    :param images: Input images to be filtered
    :param y_low: lower threshold for y
    :param y_high: upper threshold for y
    :param u_low: lower threshold for u
    :param u_high: upper threshold for u
    :param v_low: lower threshold for v
    :param v_high: upper threshold for v

    :return: Single image or dict of images
    """

    if type(images) is dict:
        return {k: img_filter(v, y_low, y_high, u_low, u_high, v_low, v_high) for k, v in images.items()}
    else:
        return img_filter(images, y_low, y_high, u_low, u_high, v_low, v_high)


# Function to filter the entire data set given as a dict
def RGB_to_BGR(images):

    """
    :param images: input image

    :return: Single image or dict of BGR images
    """

    if type(images) is dict:
        return {k: cv2.cvtColor(v, cv2.COLOR_RGB2BGR) for k, v in images.items()}
    else:
        return cv2.cvtColor(images, cv2.COLOR_RGB2BGR)


# Function to filter the entire data set given as a dict (WIP)
def BGR_to_GRAY(images):

    """
    :param images: input image

    :return: Single image or dict of gray scale images
    """

    if type(images) is dict:
        return {k: cv2.cvtColor(v, cv2.COLOR_BGR2GRAY) for k, v in images.items()}
    else:
        return cv2.cvtColor(images, cv2.COLOR_BGR2GRAY)


# Defines the filter to be used in filter_YUV
def img_filter(img, y_low, y_high, u_low, u_high, v_low, v_high):

    """
    :param img: Input image to be filtered
    :param y_low: lower threshold for y
    :param y_high: upper threshold for y
    :param u_low: lower threshold for u
    :param u_high: upper threshold for u
    :param v_low: lower threshold for v
    :param v_high: upper threshold for v

    :return: Single image YUV filtered image
    """

    Filtered = np.zeros([img.shape[0], img.shape[1]])
    Filtered[(img[:, :, 0] <= y_high) & (img[:, :, 0] >= y_low) & (img[:, :, 1] <= u_high) & (img[:, :, 1] >= u_low)
             & (img[:, :, 2] <= v_high) & (img[:, :, 2] >= v_low)] = 1
    return Filtered


# This function rescales images (WIP)
def img_rescale(img, resize_factor):

    """
    :param img: input image
    :param resize_factor: factor by which the dimensions of the picture will be reduced

    :return: Single image or dict of rescaled images
    """

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


