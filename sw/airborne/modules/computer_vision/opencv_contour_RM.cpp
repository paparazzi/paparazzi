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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, see
* <http://www.gnu.org/licenses/>.
*/
/**
* @file "modules/computer_vision/opencv_example.cpp"
* @author C. De Wagter
* opencv
*/
#include "opencv_contour_RM.h"
#include "cv_opencvdemo.h"
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
//#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;
bool backprojMode = false;
bool selectObject = false;
int hsize = 16;
float hranges[] = {0, 180};
const float *phranges = hranges;
Rect trackWindow;
int trackObject = 0;
Mat hist;
Mat histimg = Mat::zeros(200, 320, CV_8UC3);
Mat image;
Mat hsv;
Mat mask;
Mat backproj;
Mat hue;
void convertBackFromRGB(Mat image, char* img, int width, int height){
//Turn the opencv RGB colored image back in a YUV colored image for the drone
for (int row = 0; row < height; row++) {
for (int col = 0; col < width; col++) {
cv::Vec3b pixelHere = image.at<cv::Vec3b>(row, col);
img[(row * width + col) * 2 + 1] = 0.299 * pixelHere[0] + 0.587 * pixelHere[1] + 0.114 * pixelHere[2];
if (col % 2 == 0) { // U
img[(row * width + col) * 2] = 0.492 * (pixelHere[2] - img[(row * width + col) * 2 + 1] + 127);
} else { // V
img[(row * width + col) * 2] = 0.877 * (pixelHere[0] - img[(row * width + col) * 2 + 1] + 127);
}
}
}
}
void convertBackFromGrayscale(Mat image, char* img, int width, int height){
for (int row = 0; row < height; row++) {
for (int col = 0; col < width; col++) {
img[(row * width + col) * 2 + 1] = image.at<uint8_t>(row, col);
img[(row * width + col) * 2 ]=127;
}
}
}
/* Mat canny_output;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
/// Detect edges using canny
Canny( src_gray, canny_output, thresh, thresh*2, 3 );
/// Find contours
findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
/// Draw contours
Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
for( int i = 0; i< contours.size(); i++ )
{
Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
}
/// Show in a window
namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
imshow( "Contours", drawing );*/
int find_contour(char *img, int width, int height)
{
RNG rng(12345);
// Update how long we are tracking something
//selfie_var.trackingNumber++;
Mat canny_output;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
// Create a new image, using the original bebop image.
Mat M(width, height, CV_8UC2, img);
// Convert the image to a colored image
// cvtColor(M, image, CV_YUV2RGB_Y422);
cvtColor(M, image, CV_YUV2RGB_Y422);
cv::Mat hsv_image;
cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);
// Threshold the HSV image, keep only the red pixels
cv::Mat lower_red_hue_range;
cv::Mat upper_red_hue_range;
cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), lower_red_hue_range);
cv::inRange(hsv_image, cv::Scalar(150, 100, 100), cv::Scalar(255, 255, 255), upper_red_hue_range);
cv::Mat totalImage = lower_red_hue_range+upper_red_hue_range;
int erosion_type;
erosion_type = MORPH_RECT;
float erosion_size=1.5;
Mat element = getStructuringElement( erosion_type,
Size( 2*erosion_size + 1, 2*erosion_size+1 ),
Point( erosion_size, erosion_size ) );
/// Apply the erosion operation
dilate( totalImage, totalImage, element );
// convertBackFromGrayscale( totalImage, img, width, height);
//convertBackFromRGB( image, img, width, height);
// return 0;
cv::Mat gray=totalImage;
//cv::cvtColor(totalImage,gray, CV_GRAY2RGB);
//cv::cvtColor(gray,gray, CV_RGB2GRAY);
/// Detect edges using canny
// Canny( gray, gray, 100, 100*2, 3 );
vector<Vec2f> lines;
HoughLines( gray, lines, 1, CV_PI/180, 50, 0, 0 );
printf("Detected %d lines\n",lines.size());
/* // compute the intersection from the lines detected...
vector<Point2f> intersections;
for( size_t i = 0; i < lines.size(); i++ )
{
for(size_t j = 0; j < lines.size(); j++)
{
Vec2f line1 = lines[i];
Vec2f line2 = lines[j];
if(acceptLinePair(line1, line2, CV_PI / 32))
{
Point2f intersection = computeIntersect(line1, line2);
intersections.push_back(intersection);
}
}
}
if(intersections.size() > 0)
{
vector<Point2f>::iterator i;
for(i = intersections.begin(); i != intersections.end(); ++i)
{
cout << "Intersection is " << i->x << ", " << i->y << endl;
circle(occludedSquare, *i, 1, Scalar(0, 255, 0), 3);
}
}*/
/// Find contours
Mat B = gray.clone();
findContours( B, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
/// Draw contours
Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
// printf("Found %d contours\n",contours.size());
vector<vector<Point> > squareContours;
for( int i = 0; i< contours.size(); i++ )
{
vector<Point> approx;
approxPolyDP(contours[i], approx, 5, true);
// printf("Length approx: %d\n",approx.size());
if(approx.size()==4 ){
// if( isContourConvex(contours[i])){
Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
drawContours( gray, contours, i, color, 2, 8, hierarchy, 0, Point() );
}
}
// display
// cv::imshow("input", input);
// cv::imshow("drawing", drawing);
// cv::waitKey(0);
//
// cv::imwrite("rotatedRect.png",drawing);
// cv::cvtColor(drawing,image, CV_GRAY2RGB);
// Turn the opencv RGB colored image back in a YUV colored image for the drone
// for (int row = 0; row < height; row++) {
// for (int col = 0; col < width; col++) {
// cv::Vec3b pixelHere = image.at<cv::Vec3b>(row, col);
// img[(row * width + col) * 2 + 1] = 0.299 * pixelHere[0] + 0.587 * pixelHere[1] + 0.114 * pixelHere[2];
// if (col % 2 == 0) { // U
// img[(row * width + col) * 2] = 0.492 * (pixelHere[2] - img[(row * width + col) * 2 + 1] + 127);
// } else { // V
// img[(row * width + col) * 2] = 0.877 * (pixelHere[0] - img[(row * width + col) * 2 + 1] + 127);
// }
// }
// }
// void convertBackFromRGB(Mat image, uint8_t* img, int width, int height){
//
// //Turn the opencv RGB colored image back in a YUV colored image for the drone
// for (int row = 0; row < height; row++) {
// for (int col = 0; col < width; col++) {
// cv::Vec3b pixelHere = image.at<cv::Vec3b>(row, col);
// img[(row * width + col) * 2 + 1] = 0.299 * pixelHere[0] + 0.587 * pixelHere[1] + 0.114 * pixelHere[2];
// if (col % 2 == 0) { // U
// img[(row * width + col) * 2] = 0.492 * (pixelHere[2] - img[(row * width + col) * 2 + 1] + 127);
// } else { // V
// img[(row * width + col) * 2] = 0.877 * (pixelHere[0] - img[(row * width + col) * 2 + 1] + 127);
// }
// }
// }
// }
//
convertBackFromGrayscale( gray, img, width, height);
//convertBackFromRGB( image, img, width, height);
return 0;
}
