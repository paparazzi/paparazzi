
#ifndef OPENCV_IMAGE_FUNCTIONS_H
#define OPENCV_IMAGE_FUNCTIONS_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
void color_opencv_to_yuv422(Mat image, char *img, int width, int height);
void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height);
#endif
