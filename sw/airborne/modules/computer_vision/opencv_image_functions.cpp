#include "opencv_image_functions.h"


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

void color_opencv_to_yuv422(Mat image, char *img, int width, int height)
{

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


void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {

      img[(row * width + col) * 2 + 1] =   image.at<uint8_t>(row, col);
      img[(row * width + col) * 2 ] = 127;
    }
  }
}
