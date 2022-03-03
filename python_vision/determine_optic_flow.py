# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 17:01:16 2022

Determine optic flow given two images

@author: guido
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt


def determine_optic_flow(filename_1, filename_2, method="Tomasso", max_points=100, graphics=False):
    # load the BGR color image:
    # BGR_1 = cv2.imread(filename_1);
    BGR_1 = filename_1
    # convert the image to gray scale:
    gray_1 = cv2.cvtColor(BGR_1, cv2.COLOR_BGR2GRAY)
    # load the BGR color image:
    # BGR_2 = cv2.imread(filename_2);
    BGR_2 = filename_2
    # convert the image to gray scale:
    gray_2 = cv2.cvtColor(BGR_2, cv2.COLOR_BGR2GRAY)

    # (1) Detect features:
    if (method == 'Harris'):
        gray_Harris = np.float32(gray_1)
        # https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345
        # blockSize	Neighborhood size (see the details on cornerEigenValsAndVecs).
        # ksize	Aperture parameter for the Sobel operator.
        # k Harris detector free parameter.
        blockSize = 2
        ksize = 3
        k = 0.04
        # threshold_factor (multiplied with max value in image)
        threshold_factor = 0.01
        #  calculate the Harris value everywhere in the image:
        dst = cv2.cornerHarris(gray_Harris, blockSize, ksize, k)
        # dilate the values:
        # dst = cv2.dilate(dst, None)
        # Threshold the Harris values and set the corresponding pixels red
        BGR_1[dst > threshold_factor * dst.max()] = [0, 0, 255]
        # cv2.imshow('dst', BGR_1)

        inds = np.where(dst > threshold_factor * dst.max())
        n_points = len(inds[0])
        # corners = np.float32(np.zeros([n_points, 2]));

        corners = np.stack((inds[1].T, inds[0].T), axis=1)

    elif (method == 'FAST'):
        # Initiate FAST object with default values
        # https://docs.opencv.org/3.4/df/d74/classcv_1_1FastFeatureDetector.html
        threshold = 70
        nonmaxSuppression = True
        type_detector = cv2.FAST_FEATURE_DETECTOR_TYPE_9_16
        fast = cv2.FastFeatureDetector_create(threshold, nonmaxSuppression, type_detector)
        # find and draw the keypoints
        kp = fast.detect(gray_1, None)
        img2 = cv2.drawKeypoints(BGR_1, kp, None, color=(255, 0, 0))
        cv2.imshow('dst', img2)
        print("Total Keypoints with nonmaxSuppression: {}".format(len(kp)))
        # downselect the points:
        kp = np.random.choice(kp, size=max_points)
        n_points = len(kp)

        # convert the points to a 2D numpy array:
        corners = np.stack((kp.pt[0].T, kp.pt[1].T), axis=1)

        # corners = np.float32(np.zeros([n_points, 2]));
        # for i in range(n_points):
        #     corners[i, 0] = kp[i].pt[0];
        #     corners[i, 1] = kp[i].pt[1];

    elif (method == 'ShiTomasi'):
        # https://docs.opencv.org/3.4/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541
        # maxCorners	Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned. maxCorners <= 0 implies that no limit on the maximum is set and all detected corners are returned.
        # qualityLevel	Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue (see cornerMinEigenVal ) or the Harris function response (see cornerHarris ). The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.
        # minDistance	Minimum possible Euclidean distance between the returned corners.
        maxCorners = 100
        qualityLevel = 0.01
        minDistance = 10
        C = cv2.goodFeaturesToTrack(gray_1, maxCorners, qualityLevel, minDistance)

        n_points = C.shape[0]
        corners = np.float32(np.zeros([n_points, 2]))
        for i in range(n_points):
            corners[i, :] = C[i][0]

        C = np.int0(C)
        for i in C:
            x, y = i.ravel()
            cv2.circle(BGR_1, (x, y), 3, 255, -1)
        cv2.imshow('dst', BGR_1)

    elif (method=="Tomasso"):
        pass
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
    determine_optic_flow('bebop_flowers_1.jpg', 'bebop_flowers_2.jpg')
