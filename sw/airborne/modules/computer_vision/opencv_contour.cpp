/*
 * Copyright (C) 2016 Peng Lu
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/opencv_contour.cpp
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

//#include <opencv/highgui.h>

using namespace cv;
using namespace std;

//void get_integral_image(uint8_t, uint32_t, uint32_t, uint32_t);


void find_contour(char *img, int width, int height)  
{    
    /// Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);

    //findContours( edge_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    // Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
      { mu[i] = moments( contours[i], false ); }

    //  Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
     
    /// Draw contours
    Mat drawing = Mat::zeros( edge_image.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 );
        //cout<< "mc: " << mc[1] <<std::endl;
    }     
    
    imshow( "Result window", drawing );

/*-------------------------- Find Largest Contour------------------------ */
    int largest_contour_index = 0;
    int largest_area = 0;
    Rect bounding_rect;
    //vector<vector<Point>> contours; // Vector for storing contour
    //vector<Vec4i> hierarchy;
    
    
    // iterate through each contour.
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a = contourArea( contours[i],false); 
        if(a>largest_area)
        {
            largest_area = a; 
            //cout<< i <<" area  "<< a <<endl;
            // Store the index of largest contour
            largest_contour_index = i;               
            // Find the bounding rectangle for biggest contour
            bounding_rect = boundingRect(contours[i]);
        }
    }
    Scalar color( 255,255,255);  // color of the contour in the
    /*Draw the contour and rectangle */
    drawContours( image, contours, largest_contour_index, color, CV_FILLED,8,hierarchy);
    
    rectangle(image, bounding_rect,  Scalar(0,255,0), 2, 8, 0);
    
    // some figure can cause there are no largest circles, in this case, do not draw circle
    circle( image, mc[largest_contour_index], 4, Scalar(0,255,0), -1, 8, 0 );
    
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    imshow( "Display window", image ); 
    //*/
    
    
    //waitKey(1);     
    
    //waitKey(0);                                         
    //return 0;
    
    
}




/*
void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image)
{
  uint16_t x, y;
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < image_height; y++) {
      if (x >= 1 && y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width] +
                                              integral_image[x + (y - 1) * image_width] - integral_image[x - 1 + (y - 1) * image_width];
      } else if (x >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width];
      } else if (y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x + (y - 1) * image_width];
      } else {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width];
      }
    }
  }
}
*/






