import cv2
import numpy as np


def find_edge_OF(image, gray_scale=False, blur=False, minTH=0, maxTH=200):
    # img = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img = image
    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        # cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH)  # Canny Edge Detection

    return edges


def edge_detect_Harris(image, image_gray, blockSize, ksize, k, threshold_factor):
    gray_Harris = np.float32(image_gray)
    # https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345
    #  calculate the Harris value everywhere in the image:
    dst = cv2.cornerHarris(gray_Harris, blockSize, ksize, k)
    # dilate the values:
    # dst = cv2.dilate(dst, None)
    # Threshold the Harris values and set the corresponding pixels red
    image[dst > threshold_factor * dst.max()] = [0, 0, 255]
    # cv2.imshow('dst', BGR_1)

    inds = np.where(dst > threshold_factor * dst.max())
    n_points = len(inds[0])
    # corners = np.float32(np.zeros([n_points, 2]));

    corners = np.stack((inds[1].T, inds[0].T), axis=1)

    return corners, inds


def edge_detect_FAST(image, image_gray, threshold,  max_points , nonmaxSuppression=True):

    # Initiate FAST object with default values
    # https://docs.opencv.org/3.4/df/d74/classcv_1_1FastFeatureDetector.html

    type_detector = cv2.FAST_FEATURE_DETECTOR_TYPE_9_16
    fast = cv2.FastFeatureDetector_create(threshold, nonmaxSuppression, type_detector)
    # find and draw the keypoints
    kp = fast.detect(image_gray, None)
    img2 = cv2.drawKeypoints(image, kp, None, color=(255, 0, 0))
    cv2.imshow('dst', img2)
    # print("Total Keypoints with nonmaxSuppression: {}".format(len(kp)))
    # downselect the points:
    kp = np.random.choice(kp, size=max_points)
    n_points = len(kp)

    # convert the points to a 2D numpy array:
    corners = np.stack((kp.pt[0].T, kp.pt[1].T), axis=1)
    corners = corners[0:99]

    return corners


def edge_detect_ShiTomasi(image, image_gray, maxCorners, qualityLevel, minDistance):

    # https://docs.opencv.org/3.4/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541
    # maxCorners	Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned. maxCorners <= 0 implies that no limit on the maximum is set and all detected corners are returned.
    # qualityLevel	Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue (see cornerMinEigenVal ) or the Harris function response (see cornerHarris ). The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.
    # minDistance	Minimum possible Euclidean distance between the returned corners.

    C = cv2.goodFeaturesToTrack(image_gray, maxCorners, qualityLevel, minDistance)

    n_points = C.shape[0]
    corners = np.float32(np.zeros([n_points, 2]))
    for i in range(n_points):
        corners[i, :] = C[i][0]

    C = np.int0(C)
    for i in C:
        x, y = i.ravel()
        cv2.circle(image, (x, y), 3, 255, -1)
    cv2.imshow('dst', image)

    return corners

