/*
 * Copyright (C) IMAV 2016
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_helipad.cpp"
 * @author IMAV 2016
 * Module for detecting helipad marker in IMAV 2016
 */


#include "opencv_helipad.h"
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

// Output
struct results helipad;

struct results opencv_helipad(char *img, int width, int height, int mod)
{
    // Create a new image, using the original bebop image.
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    Mat imcopy;

    // Initialize vectors
    vector<vector<Point> > contours;
    vector<vector<Point> > saved_contours;
    vector<int> saved_area;
    vector<Vec4i> hierarchy;
    vector<cv::Point> found_centroids;
    cv::Point centroid;

    // Convert the image to grayscale
    cvtColor(M, image, CV_YUV2GRAY_Y422);

    // Make a copy of color image- only for testing
    cvtColor(M, imcopy, CV_YUV2GRAY_Y422);

    // Gaussian Blur
    blur(image, image, Size(5, 5));

    // Convert to binary image
    threshold(image, image, 0, 255, THRESH_OTSU);

    // Canny edges
    Canny(image, image, 66, 133);

    // Find contours
    findContours(image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] ) {
        double Area = contourArea(contours[idx]);
        if (Area > 6000) {
            saved_contours.push_back (contours[idx]);
            saved_area.push_back (Area);
        }
    }

    vector<Moments> mu(saved_contours.size() );
    for(int i=0; i < saved_contours.size(); i++) {
        mu[i] = moments( saved_contours[i], false );
    }

    vector<Point2f> mc( saved_contours.size() );
    for(int i = 0; i < saved_contours.size(); i++ ) {
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }

    for(int i = 0; i < saved_contours.size(); i++) {
        double A = saved_area[i];
        for(int j = i + 1; j < saved_contours.size(); j++) {
            double B = saved_area[j];

            double ratio = A / B;

            if( abs(ratio-1.5) > 0.2) {
                if( (abs(mc[i].x - mc[j].x) < 10) & (abs(mc[i].y - mc[j].y) < 10)) {
                    found_centroids.push_back (mc[i]);
                }
            }
        }
    }

    if(found_centroids.size() > 0) {
        centroid = found_centroids[0];
        if (mod) {
            circle(imcopy, centroid, 5, 255, 10);
        }
        helipad.MARKER = 1;
        helipad.maxx = centroid.x;
        helipad.maxy = centroid.y;
    } else {
        helipad.MARKER = 0;
    }

    // Convert back to YUV422, and put it in place of the original image
    grayscale_opencv_to_yuv422(imcopy, img, width, height);

    return helipad;
}