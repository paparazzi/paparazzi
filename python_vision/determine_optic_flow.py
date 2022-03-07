# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 17:01:16 2022

Determine optic flow given two images

@author: guido
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from Edge_detect import *



def determine_optic_flow(filename_1, filename_2, method="ShiTomasi", max_points=100, graphics=True):

    BGR_1 = cv2.imread(filename_1);
    # BGR_1 = filename_1 # load the BGR color image:
    gray_1 = cv2.cvtColor(BGR_1, cv2.COLOR_BGR2GRAY)  # convert the image to gray scale
    BGR_2 = cv2.imread(filename_2);
    # BGR_2 = filename_2     # load the BGR color image:
    gray_2 = cv2.cvtColor(BGR_2, cv2.COLOR_BGR2GRAY) # convert the image to gray scale:

    # (1) Detect features:
    if method == 'Harris':
        corners, inds = edge_detect_Harris(image=BGR_1, image_gray=gray_1,blockSize=2,ksize=3,k=0.04,threshold_factor=0.01)

    elif method == 'FAST':

        corners = edge_detect_FAST(image=BGR_1, image_gray=gray_1, threshold=70, max_points=max_points)

    elif method == 'ShiTomasi':

        corners = edge_detect_ShiTomasi(image=BGR_1, image_gray=gray_1, maxCorners=100, qualityLevel=0.01, minDistance=10)

    elif method == "Tomasso":
        edges = find_edge_OF(BGR_1)
        inds = np.where(edges == 255)
        corners = np.stack((inds[1].T, inds[0].T), axis=1)
        corners = corners.astype("float32")


    # (2) Track the features to the next frame:

    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # calculate optical flow
    corners_new, status, error_match = cv2.calcOpticalFlowPyrLK(gray_1, gray_2, corners, None, **lk_params)

    # filter the points by their status:
    # corners = corners[status == 1, :];
    # corners_new = corners_new[status == 1, :];

    flow_vectors = corners_new - corners

    if (graphics):
        im = (0.5 * BGR_1.copy().astype(float) + 0.5 * BGR_2.copy().astype(float)) / 255.0
        n_corners = len(corners)
        color = (0.0, 1.0, 0.0)
        for p in range(n_corners):
            cv2.arrowedLine(im, (int(corners[p, 0]), int(corners[p, 1])),
                            (int(corners_new[p, 0]), int(corners_new[p, 1])), color, thickness=2, tipLength=0.5)

        cv2.imshow('Flow', im);
        plt.figure();
        plt.imshow(im);
        plt.title('Optical flow');


if __name__ == "__main__":
    # Determine optic flow on the images provided on the repository:
    determine_optic_flow('test_image_1.jpg', 'test_image_2.jpg')
