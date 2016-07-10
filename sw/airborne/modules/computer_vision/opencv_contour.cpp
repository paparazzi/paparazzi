/*
 * Copyright (C) Peng Lu
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
 * @file "modules/computer_vision/opencv_contour.cpp"
 * @author Peng Lu
 * 
 */


#include "opencv_contour.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

//------------------------ added by Peng Lu ---------------------------//
//#include <opencv2/highgui/highgui.hpp>
float distance_est;
float d_x, d_y, d_z;
//------------------------ added by Peng Lu ---------------------------//



///*
// YUV in opencv convert to YUV on Bebop
void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b &c = image.at<cv::Vec3b>(row, col);

      // Set image buffer values
      int i = row * width + col;
      img[2 * i + 1] = c[0]; // y;
      img[2 * i] = col % 2 ? c[1] : c[2]; // u or v
      //img[2 * i + 1] = 0; // y;
      //img[2 * i] = 1; // u or v
    }
  }
}
///*
void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b &c = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b &c_m1 = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b &c_p1 = image_in.at<cv::Vec3b>(row, col);
      if (col > 0)
      { cv::Vec3b &c_m1 = image_in.at<cv::Vec3b>(row, col-1);}
      if (col < width)
      { cv::Vec3b &c_p1 = image_in.at<cv::Vec3b>(row, col+1);}
      image.at<cv::Vec3b>(row, col)[0] = c[1] ;
      image.at<cv::Vec3b>(row, col)[1] = col % 2 ? c[0] : c_m1[0];
      image.at<cv::Vec3b>(row, col)[2] = col % 2 ? c_p1[0] : c[0];

    }
  }
}
//*/



int find_contour(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img); // original
  //Mat M(width, height, CV_8UC3, img);
  //Mat M(width, height, CV_8UC1, img);
  Mat image, edge_image, thresh_image;

  // 
  //blur(image, image, Size(5, 5));
  //GaussianBlur(image, image, Size(9,9), 5,5);

  // Canny edges, only works with grayscale image
  //int edgeThresh = 35;
  //Canny(image, image, edgeThresh, edgeThresh * 3);
  
  //imshow("edge image", image);
  
  // convert UYVY in paparazzi to YUV in opencv
  //Mat M_out(width, height, CV_8UC3, Scalar(0,0,255));
  cvtColor(M, M, CV_YUV2RGB_Y422);
  cvtColor(M, M, CV_RGB2YUV);
  //Mat M(width, height, CV_8UC3, Scalar(0,0,255));
  //uyvy_opencv_to_yuv_opencv(M_out, M, width, height);
  
  ///*
  int y1, y2, u1, u2, v1, v2;
  

  y1=16;  u1=135; v1=80; // in the mavlab, bright
  y2=100; u2=175; v2=165;
  // for cyberzoo: Y:12-95, U:129-161, V:80-165, turn white.     
  //int y1=16;  int u1=129; int v1=80; % cyberzoo, dark
  //int y2=100; int u2=161; int v2=165;
    
  inRange(M, Scalar(y1,u1,v1), Scalar(y2,u2,v2), thresh_image);
 
  
  
  /*
  cvtColor(M, image, CV_YUV2RGB_Y422);
  Mat hsv_image;
  cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);
  
  Mat lower_red_hue_range;
  Mat upper_red_hue_range;
  inRange(hsv_image, Scalar(0, 100, 100), Scalar(30, 255, 255), lower_red_hue_range);
  inRange(hsv_image, Scalar(150, 100, 100), Scalar(255, 255, 255), upper_red_hue_range);
  Mat thresh_image = lower_red_hue_range+upper_red_hue_range;
  */

  /// Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    edge_image = thresh_image;
    Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
    //findContours( edge_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( edge_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
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
    
    //imshow( "Result window", drawing );

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
    drawContours( M, contours, largest_contour_index, color, CV_FILLED,8,hierarchy);
    
    rectangle(M, bounding_rect,  Scalar(0,255,0), 2, 8, 0);
    
    // some figure can cause there are no largest circles, in this case, do not draw circle
    circle( M, mc[largest_contour_index], 4, Scalar(0,255,0), -1, 8, 0 );
    Point2f rect_center(bounding_rect.x+bounding_rect.width/2 , bounding_rect.y+bounding_rect.height/2);
    circle( image, rect_center, 4, Scalar(0,0,255), -1, 8, 0 );
    
    //namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    //imshow( "Display window", image ); 
    
   
   // Convert back to YUV422, and put it in place of the original image
   grayscale_opencv_to_yuv422(M, img, width, height);
   //yuv_opencv_to_yuv422(image, img, width, height);
   
   //estimate the distance
   float area = bounding_rect.width * bounding_rect.height;
   //double distance_est = 
   /* determine the distance from the area */
    
    //distance_est = 
    ///*
    if (area > 28000.)
    { distance_est = 0.1;}
    if ( (area > 16000.) && ( area < 28000. ) )
    { distance_est = 0.5;}
    if ( (area > 11000.) && ( area < 16000. ) )
    { distance_est = 1;}
    if ( (area > 3000.) && ( area < 11000. ) )
    { distance_est = 1.5;}
    if (area < 3000.)
    { distance_est = 2.0;}
    //*/
    //printf("contour size =%ld\n", contours.size() );
    //printf("center x =%d\n", bounding_rect.x+bounding_rect.width/2);
    //printf("center y =%d\n", bounding_rect.y+bounding_rect.height/2) ;   
    printf("distance estimate =%f\n", distance_est );
   
   //printf("width: %d,", width); // 272
   //printf("height: %d\n", height); // 272
   
   
   d_x = distance_est;
   
   printf("dx =%f\n", d_x );
   
   float Im_center_w = width/2.;
   float Im_center_h = height/2.;
   
   float real_size = 1.; // real size of the object
   d_y = -(rect_center.x - Im_center_w) * real_size/float(bounding_rect.width); // right hand
   d_z = -(rect_center.y - Im_center_h) * real_size/float(bounding_rect.height); // point downwards
   
   
   printf("dy =%f\n", d_y );  
   printf("dz =%f\n", d_z );    
 
 
 
 
   return 0;
    
}














