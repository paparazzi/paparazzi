/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/opencv_IMAV_landingpad.cpp"
 * @author J. Lee
 * Module for detecting outdoor landing pad in IMAV 2016
 */

#include "opencv_imav_landingpad.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

// Output
struct results landing;


struct results opencv_imav_landing(char *img, int width, int height, int v_squares, int binary_threshold, int mod)
{
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    Mat binim;
    Mat imcopy;
//    int thresh = 230; //For outdoor landingpad
//    int thresh = 210; //For indoor helipad

//    int z = 4; // z-score for outdoor
    int z = 2; // z-score for indoor


    // Grayscale image
    cvtColor(M, image, CV_YUV2GRAY_Y422);

    // copy image
    cvtColor(M, imcopy, CV_YUV2GRAY_Y422);

    // Gaussian Blur
    blur(image, image, Size(5,5));

    // convert to binary image
    threshold(image, binim, binary_threshold, 255, THRESH_BINARY);
//    adaptiveThreshold(image, binim, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, 2);

    // Canny edges
    Canny(binim, image, 66, 133);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Find contours
    findContours(image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );

    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> >approx(contours.size());
    int detected_squares = 0;
    vector<Moments> mu(contours.size() );
    vector<Point2f> mc( contours.size() );
    vector<float> centroidsx;
    vector<float> centroidsy;


    for( int i = 0; (unsigned)i < contours.size(); i++ )
    {
        double Area = contourArea(contours[i]);
        if (Area > 1200)
        {
            convexHull(Mat(contours[i]), hull[i], false );
            approxPolyDP(Mat(hull[i]), approx[i], arcLength(Mat(hull[i]), true)*0.1, true);
            int sides = approx[i].size();
            if (sides  == 4)
            {
                detected_squares += 1;
                mu[i] = moments(contours[i], false);
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                centroidsx.push_back (mc.at(i).x);
                centroidsy.push_back (mc.at(i).y);
                if (mod) {drawContours( binim, approx, i, 200 , 5, 8, vector<cv::Vec4i>(), 0, cv::Point() ); }
            }
        }
    }


    float sumx = 0;
    float sumy = 0;
    float avgx = 0;
    float avgy = 0;

    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        sumx += centroidsx[i];
        sumy += centroidsy[i];
    }
    avgx = sumx / detected_squares;
    avgy = sumy / detected_squares;


    float innerx = 0;
    float innery = 0;
    float stdx = 0;
    float stdy = 0;

    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        innerx = pow((centroidsx[i] - avgx),2);
        innery = pow((centroidsy[i] - avgy),2);
    }

    stdx = sqrt(innerx/detected_squares);
    stdy = sqrt(innery/detected_squares);


    vector<float> filt_centroidx;
    vector<float> filt_centroidy;
    vector<float> ffilt_centroidx;
    vector<float> ffilt_centroidy;



    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        if (abs((centroidsx[i]-avgx)/stdx) <= z)
        {
            filt_centroidx.push_back (centroidsx[i]);
            filt_centroidy.push_back (centroidsy[i]);
        }
    }


    int n_filtcentroids = 0;
    for(int i = 0; (unsigned)i < filt_centroidy.size(); i++)
    {
        if (abs((filt_centroidy[i]-avgy)/stdy) <= z)
        {
            n_filtcentroids += 1;
            ffilt_centroidx.push_back (filt_centroidx[i]);
            ffilt_centroidy.push_back (filt_centroidy[i]);
        }
    }


    int xsum = 0;
    int ysum = 0;
    int xbar = 0;
    int ybar = 0;

    // mean method

    for(int i = 0; (unsigned)i < ffilt_centroidx.size(); i++)
    {
        xsum += ffilt_centroidx[i];
        ysum += ffilt_centroidy[i];
    }



    if (n_filtcentroids >= v_squares)
    {
        landing.MARKER = 1;
        xbar = (xsum / n_filtcentroids);
        ybar = (ysum / n_filtcentroids);
        landing.maxx = xbar;
        landing.maxy = ybar;
        if (mod) { circle(binim, Point(xbar,ybar), 20, 200, 10); }
    } else
    {
        landing.MARKER = 0;
        landing.maxx   = 0;
        landing.maxy   = 0;
    }

    grayscale_opencv_to_yuv422(binim, img, width, height);

    return landing;
}
