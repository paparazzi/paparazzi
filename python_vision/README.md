# README FOR PYTHON VISION CODE MADE BY GROUP 7
# AE4317 2022

Authors
-------
T. Capra, F. Larocque, Y.D. Li, J. Liang, F.J.T. Rhenman, K.Z. Six, N. Wechtler, M.H. Yoganarasimhan

Required python packages
------------------------
Numpy, Seaborn, TQDM, Matplotlib, opencv-python, os, time

Purpose
------

The purpose of these files is to provide a quick and simple way to evaluate the
accuracy and effectiveness of different optical flow based approaches to obstacle avoidance

1. load_data.py
   
   This file enables the loading and pre-processing of pictures for use in edge detection and optical flow calculations
 
2. Edge_detect.py
   
    This files provides edge detect functions to be used for optical flow based on different methods
    The included edge detection methods are:
   - Canny 
   - Harris
   - FAST
   - Shi Tomassi
3. determine_optic_flow.py

   This file is used for calculating the optical flow between two images. Original file was written by Guido as example
   code for the Autonomous Flight of Micro Air Vehicles course. The file has been modified to be compatible with our
   scripts.
4. quadrant_test.py

   File used for determining the feasibility and accuracy of using split divergence between the left and right side of
   pictures. Evaluates the RMSE and relative computational time by varying the number of points used as well as evaluating
   the probability that no corners appear in either side of the image.
5. performance_test.py

   File used for evaluating the accuracy and relative computational time of optical flow calculations by varying
   the window size to be used.
