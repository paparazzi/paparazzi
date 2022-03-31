# -*- coding: utf-8 -*-
"""
Created on Mon Feb 28 15:06:12 2022

@author: tomma
"""

import cv2
import numpy as np
import os
import time

def load_images_from_folder(folder):
    images = []
    for filename in sorted(os.listdir(folder)):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images


def find_edge(image,resize_factor=1,gray_scale=False,blur=False,minTH = 0, maxTH = 200,DRAW_BORDER = True,DBR = 10):
    img = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Display original image
    # cv2.imshow('Original', img)
    image_copy = img.copy()

    # Resize
    img = cv2.resize(img, (int(img.shape[1] / resize_factor), int(img.shape[0] / resize_factor)))

    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        #cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH)  # Canny Edge Detection

    if DRAW_BORDER == True:
        for i in range(0+DBR,img.shape[0]-DBR):
            edges[i][0] = 255
            edges[i][img.shape[1]-1] = 255

        for i in range(0+DBR, img.shape[1]-DBR):
            edges[0][i] = 255
            edges[img.shape[0]-1][i] = 255
    # Display Canny Edge Detection Image
    # cv2.imshow('Canny Edge Detection', edges)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    img, contours, hierarchy = cv2.findContours(image=edges, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

    # draw contours on the original image
    cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2,lineType=cv2.LINE_AA)

    if len(contours) == 0:
        return image_copy

    # find the biggest countour (c) by the area
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)

    # draw the biggest contour (c) in green
    cv2.rectangle(image_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)

    return image_copy

def find_edge_V1(image,resize_factor=1,size_crit_1=0.01,size_crit_2=0.1,gray_scale=False,blur=False,minTH = 0, maxTH = 200, DRAW_BORDER = True,DBR = 2, THICKER_LINES = True, thickness =1 ):
    img = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Display original image
    # cv2.imshow('Original', img)
    image_copy = img.copy()

    # Resize
    img = cv2.resize(img, (int(img.shape[1] / resize_factor), int(img.shape[0] / resize_factor)))

    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        # cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH, apertureSize = 5)  # Canny Edge Detection

    if THICKER_LINES == True:
        edges_mod = np.zeros(np.shape(edges))
        TMat = np.ones((thickness * 2 + 1, thickness * 2 + 1)) * 255
        for i in range(thickness,img.shape[0]-thickness):
            for j in range(thickness, img.shape[1] - thickness):
                if edges[i][j] == 255:
                    edges_mod[i-thickness:i+thickness+1,j-thickness:j+thickness+1] = TMat
        edges =edges_mod.astype(np.uint8)

    if DRAW_BORDER == True:
        for i in range(0 + DBR, img.shape[0] - DBR):
            edges[i][0] = 255
            edges[i][img.shape[1] - 1] = 255

        for i in range(0 + DBR, img.shape[1] - DBR):
            edges[0][i] = 255
            edges[img.shape[0] - 1][i] = 255


    # Display Canny Edge Detection Image
    cv2.imshow('Canny Edge Detection', edges)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    img, contours, hierarchy = cv2.findContours(image=edges, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE )#

    if len(contours) == 0:
        return image_copy

    # Select biggest contours
    biggest_contours = [contour for contour in contours if (cv2.contourArea(contour)>size_crit_1*img.shape[1]*img.shape[0] and cv2.contourArea(contour)<size_crit_2*img.shape[1]*img.shape[0])]

    # Calculate center of contour
    centroid_im = image_copy
    centroids = []
    for c in biggest_contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])  # cX
            cY = int(M["m01"] / M["m00"])  # cY
        else:
            cX, cY = 0, 0

        centroids.append([cX, cY])
        cv2.circle(centroid_im, (cX, cY), 5, (255, 255, 255), -1)

    # draw contours on the original image
    cv2.drawContours(image=centroid_im, contours=biggest_contours, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

    return centroid_im

def find_edge_V2(img,resize_factor=1,gray_scale=False,blur=False,minTH = 100, maxTH = 200 ):

    # Display original image
    # cv2.imshow('Original', img)
    # image_copy = img.copy()

    # Resize
    img = cv2.resize(img, (int(img.shape[1] / resize_factor), int(img.shape[0] / resize_factor)))

    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        #cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH)  # Canny Edge Detection

    # Display Canny Edge Detection Image
    # cv2.imshow('Canny Edge Detection', edges)

    # Find edge locations (sweep)
    gain = np.zeros(img.shape[0])
    for i in range(0,img.shape[0]):
        counter = 0
        for j in range(0,img.shape[1]):
            if edges[i][j] == 255 and counter == 0:
                counter = 1
                gain[i] = j

    return gain

# For optic flow
def find_edge_OF(image,gray_scale=False,blur=False,minTH = 0, maxTH = 200 ):
    img = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        #cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH)  # Canny Edge Detection

    return edges



# Read the original image
img = cv2.imread('29148598.jpg')
img2 = cv2.imread('43215138.jpg')
find_edge(img2,gray_scale=False,blur=False)

# Folder to do analysis from
#folder_path = './AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646'
folder_path = './AE4317_2019_datasets/cyberzoo_poles/20190121-135009'
#folder_path = './AE4317_2019_datasets/sim_poles/20190121-160844'

# Print all steps of processing
complete_visualization = False

# Resize image to facilitate processing
resize_factor = 1

# Minimum size for obstacle (percentage of full image size)
size_crit = 0.01

images = load_images_from_folder(folder_path)

for image in images:
    contours_im = find_edge_V1(image,size_crit_1=0.05,size_crit_2=0.1,resize_factor=1,gray_scale=True,blur=True,minTH = 150, maxTH = 200,DBR=2, thickness =0,THICKER_LINES = False )

    imS = cv2.resize(contours_im, (960, 540))
    cv2.imshow('image window', imS)
    c = cv2.waitKey(1)
    time.sleep(0.05)
    if c == 27:
        break

cv2.destroyAllWindows()

# TO DO
# implement max size limit
# Bottom obstical removal
