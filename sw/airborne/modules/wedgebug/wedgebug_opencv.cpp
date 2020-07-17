/*
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



#include "wedgebug_opencv.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h> // needed for CV_MOP_CLOSE
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdint.h>

using namespace cv;



// Local declarations
int transfer(const Mat *from, const image_t *to);


// Local functions
int transfer(const Mat *from, const image_t *to)
{
  // Determining type of supplied image
  // 1) If image is of type uint16_t
  if (to->type == IMAGE_INT16) {
    typedef uint16_t img_dip_type;
    for (int i = 0; i < (from->rows * from->cols); i++) {
      ((img_dip_type *)to->buf)[i]  = from->at<img_dip_type>
                                      (i); // Using ".at" here, as accessing buffer is problematic with a cropped image as it maintains a connection to original image
    }
  }
  // 2) If image is of type uint8_t
  else if (to->type == IMAGE_GRAYSCALE) { // If image is of type uint8_t
    typedef uint8_t img_dip_type;
    for (int i = 0; i < (from->rows * from->cols); i++) {
      ((img_dip_type *)to->buf)[i]  = from->at<img_dip_type>
                                      (i); // Using ".at" here, as accessing buffer is problematic with a cropped image as it maintains a connection to original image
    }
  }

  return 0;
}






// Global functions
int save_image_gray(struct image_t *img, char *myString)
{
  // Create a new image, using the original bebop image.

  if (img->type == IMAGE_INT16) {
    Mat M(img->h, img->w, CV_16SC1, img->buf);
    Mat Mcopy = M.clone();
    //normalize(M, M, 0, 255, NORM_MINMAX);
    imwrite(myString, Mcopy);

  } else if (img->type == IMAGE_GRAYSCALE) {
    Mat M(img->h, img->w, CV_8UC1, img->buf);
    Mat Mcopy = M.clone();

    imwrite(myString, Mcopy);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }
  return 0;
}



int save_image_color(struct image_t *img, char *myString)
{
  // Create a new image, using the original bebop image.
  Mat M(img->h, img->w, CV_8UC2, img->buf); // CV_8UC2 is the openCV format for UYVY images
  // Definition of Mat: Mat::Mat(int _rows, int _cols, int _type, void* _data, size_t _step)
  // Remember that void takes any data type including char
  Mat image;

  cvtColor(M, image, CV_YUV2GRAY_Y422);
  imwrite(myString, image);

  // Code below is for testing
  //double minVal;
  //double maxVal;
  //Point minLoc;
  //Point maxLoc;
  //minMaxLoc(image ,&minVal, &maxVal, &minLoc, &maxLoc);
  //std::cout << "Left1: Min=" << minVal << "; Max=" << maxVal << std::endl;
  return 0;
}


int save_image_HM(struct image_t *img, char *myString, int const heatmap)
{
  {
    // Create a new image, using the original bebop image.

    /*
     * Heat maps:
     * 0 = COLORMAP_AUTUMN
     * 1 = COLORMAP_BONE
     * 2 = COLORMAP_JET
     * 3 = COLORMAP_WINTER
     * 4 = COLORMAP_RAINBOW
     * 5 = COLORMAP_OCEAN
     * 6 = COLORMAP_SUMMER
     * 7 = COLORMAP_SPRING
     * 8 = COLORMAP_COOL
     * 9 = COLORMAP_HSV
     * 10 = COLORMAP_PINK
     * 11 = COLORMAP_HOT
     */


    if (img->type == IMAGE_INT16) {
      Mat M(img->h, img->w, CV_16SC1, img->buf);
      Mat Mcopy = M.clone();

      //normalize(Mcopy, Mcopy, 0, 255, NORM_MINMAX);
      Mcopy.convertTo(Mcopy, CV_8UC1); //Converting image to 8 bit image
      applyColorMap(Mcopy, Mcopy, heatmap);

      imwrite(myString, Mcopy);

    } else if (img->type == IMAGE_GRAYSCALE) {
      Mat M(img->h, img->w, CV_8UC1, img->buf);
      Mat Mcopy = M.clone();

      applyColorMap(Mcopy, Mcopy, heatmap);

      imwrite(myString, Mcopy);
    } else {
      std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
                std::endl;
      return -1;
    }
    return 0;
  }

}




int SBM_OCV(struct image_t *img_disp, const struct image_t *img_left, const struct image_t *img_right,
            const int ndisparities, const int SADWindowSize, const bool cropped)
{
  // Defining variables
  Mat img_left_OCV(img_left->h, img_left->w, CV_8UC1, img_left->buf);
  Mat img_right_OCV(img_right->h, img_right->w, CV_8UC1, img_right->buf);
  Mat img_disp_OCV;
  Mat img_disp_2_OCV;

  //std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  //std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
  //std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
  //std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;



  // Block matching
  Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
  sbm->compute(img_left_OCV, img_right_OCV, img_disp_OCV); //type of img_disp_OCV is CV_16S i.e. int16_t


  // Determining type of supplied image
  // 1) If image is of type int16_t
  if (img_disp->type == IMAGE_INT16) {
    //std::cout << "int16_t" << std::endl;
    typedef int16_t img_dip_type;

    // Cropping or not cropping:
    if (cropped == 0) { // If image should not be cropped
      for (int i = 0; i < (img_disp_OCV.rows * img_disp_OCV.cols); i++) {
        ((img_dip_type *)img_disp->buf)[i]  = img_disp_OCV.at<img_dip_type>
                                              (i); // Using ".at" here are accessing buffer is problematic with a cropped image as it maintains a connection to oriinal image
      }
    } else if (cropped == 1) { // If image should be cropped
      struct crop_t img_cropped_info;
      struct img_size_t original_img_dims = {img_left->w, img_left->h};

      post_disparity_crop_rect(&img_cropped_info, &original_img_dims,  ndisparities,
                               SADWindowSize); // Function from wedebug.h
      Rect crop_area = Rect(img_cropped_info.x, img_cropped_info.y, img_cropped_info.w, img_cropped_info.h);
      Mat img_cropped = img_disp_OCV(crop_area);// <---

      for (int i = 0; i < (img_cropped.rows * img_cropped.cols); i++) {
        ((img_dip_type *)img_disp->buf)[i]  = img_cropped.at<img_dip_type>
                                              (i); // Using ".at" here are accessing buffer is problematic with a cropped image as it maintains a connection to oriinal image
      }

    } else {return -1;}
  }


  // 2) If image is of type uint8_t
  else if (img_disp->type == IMAGE_GRAYSCALE) { // If image is of type uint8_t

    //std::cout << "uint8_t" << std::endl;
    typedef uint8_t img_dip_type;

    // THe followig code is to display the depth map with depth capped at 256/
    // This is for debugging only such that the depth image has moe contrast when observing it
    //imwrite("/home/dureade/Documents/paparazzi_images/img_disp_int8_cropped_capped.bmp", img_disp_OCV);



    img_disp_OCV = img_disp_OCV / 16; // We divide by 16 to get actual disparity values
    img_disp_OCV.convertTo(img_disp_OCV, CV_8UC1);  // Now we convert 16bit image to 8 bit image



    // Cropping or not cropping:
    if (cropped == 0) { // If image should not be cropped
      for (int i = 0; i < (img_disp_OCV.rows * img_disp_OCV.cols); i++) {
        ((img_dip_type *)img_disp->buf)[i]  = img_disp_OCV.data[i];
      }
    } else if (cropped == 1) { // If image should be cropped
      struct crop_t img_cropped_info;
      struct img_size_t original_img_dims = {img_left->w, img_left->h};


      post_disparity_crop_rect(&img_cropped_info, &original_img_dims,  ndisparities,
                               SADWindowSize); // Function from wedebug.h

      Rect crop_area = Rect(img_cropped_info.x, img_cropped_info.y, img_cropped_info.w, img_cropped_info.h);
      Mat img_cropped = img_disp_OCV(crop_area); // Problem here


      // Test code for saving cropped left image
      //Mat img_left_cropped =img_left_OCV(crop_area);
      //imwrite("/home/dureade/Documents/paparazzi_images/for_report/img_left_int8_cropped.bmp", img_left_cropped);

      for (int i = 0; i < (img_cropped.rows * img_cropped.cols); i++) {
        ((img_dip_type *)img_disp->buf)[i]  = img_cropped.at<img_dip_type>
                                              (i); // Using ".at" here are accessing buffer is problematic with a cropped image as it maintains a connection to oriinal image
      }
    } else {return -1;}

  }


  // 3) If image is of unsupported type
  else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }

  return 0;
}


int opening_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size, const int iteration)
{
  Mat img_input_OCV;
  Mat img_output_OCV;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(SE_size, SE_size));

  if (img_input->type == IMAGE_INT16) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_16S, img_input->buf);
  } else if (img_input->type == IMAGE_GRAYSCALE) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_8UC1, img_input->buf);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }


  morphologyEx(img_input_OCV, img_output_OCV, CV_MOP_OPEN, kernel, Point(-1, -1), iteration);
  //erode(img_input_OCV, img_output_OCV, kernel);
  transfer(&img_output_OCV, img_output);

  return 0;
}




int closing_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size, const int iteration)
{
  Mat img_input_OCV;
  Mat img_output_OCV;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(SE_size, SE_size));

  if (img_input->type == IMAGE_INT16) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_16S, img_input->buf);

  } else if (img_input->type == IMAGE_GRAYSCALE) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_8UC1, img_input->buf);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }


  morphologyEx(img_input_OCV, img_output_OCV, CV_MOP_CLOSE, kernel, Point(-1, -1), iteration);
  //erode(img_input_OCV, img_output_OCV, kernel);
  transfer(&img_output_OCV, img_output);
  return 0;
}



int dilation_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size, const int iteration)
{
  Mat img_input_OCV;
  Mat img_output_OCV;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(SE_size, SE_size));

  if (img_input->type == IMAGE_INT16) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_16S, img_input->buf);

  } else if (img_input->type == IMAGE_GRAYSCALE) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_8UC1, img_input->buf);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }

  dilate(img_input_OCV, img_output_OCV, kernel, Point(-1, -1), iteration);
  //erode(img_input_OCV, img_output_OCV, kernel);
  transfer(&img_output_OCV, img_output);
  return 0;
}



int erosion_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size, const int iteration)
{
  Mat img_input_OCV;
  Mat img_output_OCV;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(SE_size, SE_size));

  if (img_input->type == IMAGE_INT16) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_16S, img_input->buf);

  } else if (img_input->type == IMAGE_GRAYSCALE) {
    img_input_OCV = Mat(img_input->h, img_input->w, CV_8UC1, img_input->buf);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }

  erode(img_input_OCV, img_output_OCV, kernel, Point(-1, -1), iteration);
  //erode(img_input_OCV, img_output_OCV, kernel);
  transfer(&img_output_OCV, img_output);
  return 0;
}



int sobel_OCV(struct image_t *img_input, const struct image_t *img_output, const int kernel_size, const int thr)
{

  Mat img_input_OCV;
  Mat img_grad_x, img_grad_y, img_grad_mag;
  int ddepth = CV_32F;//CV_16S; // Format of gradient image output (CV_32F needed for normalization function)
  int delta  = 0; // Value added to each gradient pixel
  int scale = 1; // Factor by which gradient pixels is increased


  if (img_input->type == IMAGE_INT16) {
    //std::cout << "IMAGE_OPEN_DISP" << std::endl;
    img_input_OCV = Mat(img_input->h, img_input->w, CV_16S, img_input->buf);
    Sobel(img_input_OCV, img_grad_x, ddepth, 1, 0, kernel_size, scale, delta, BORDER_DEFAULT);  // Horizontal gradient
    //imwrite("/home/dureade/Documents/paparazzi_images/img_grad_x.bmp", img_grad_x*-1);
    Sobel(img_input_OCV, img_grad_y, ddepth, 0, 1, kernel_size, scale, delta, BORDER_DEFAULT);  // Vertical gradient
    //imwrite("/home/dureade/Documents/paparazzi_images/img_grad_y.bmp", img_grad_y);
    magnitude(img_grad_x, img_grad_y, img_grad_mag); // Calculating magnitude

    //imwrite("/home/dureade/Documents/paparazzi_images/for_report/img_grad_mag.bmp", img_grad_mag);
    threshold(img_grad_mag, img_grad_mag, thr,  127, THRESH_BINARY);
  } else if (img_input->type == IMAGE_GRAYSCALE) {
    //std::cout << "IMAGE_GRAYSCALE" << std::endl;
    img_input_OCV = Mat(img_input->h, img_input->w, CV_8UC1, img_input->buf);
    Sobel(img_input_OCV, img_grad_x, ddepth, 1, 0, kernel_size, scale, delta, BORDER_DEFAULT);  // Horizontal gradient
    //imwrite("/home/dureade/Documents/paparazzi_images/img_grad_x.bmp", img_grad_x*-1);
    Sobel(img_input_OCV, img_grad_y, ddepth, 0, 1, kernel_size, scale, delta, BORDER_DEFAULT);  // Vertical gradient
    //imwrite("/home/dureade/Documents/paparazzi_images/img_grad_y.bmp", img_grad_y);
    magnitude(img_grad_x, img_grad_y, img_grad_mag); // Calculating magnitude
    //imwrite("/home/dureade/Documents/paparazzi_images/for_report/img_grad_mag.bmp", img_grad_mag);
    threshold(img_grad_mag, img_grad_mag, thr,  127, THRESH_BINARY);
  } else {
    std::cout << "This function only worked with images of type IMAGE_GRAYSCALE and IAMGE_OPENCV_DISP. Leaving function." <<
              std::endl;
    return -1;
  }



  //normalize(img_grad_mag, img_grad_mag,  0, 255, NORM_MINMAX); // Normalizing magnitude between 0 and 255

  img_grad_mag.convertTo(img_grad_mag, CV_8UC1); //Converting image to 8 bit image

  transfer(&img_grad_mag, img_output); // Saving image into output images

  Mat(img_output->h, img_output->w, CV_8UC1, img_output->buf);


  /*
  // Size of variables
  std::cout << "img_input_OCV = " << img_input_OCV.total() * img_input_OCV.elemSize() << std::endl;
  std::cout << "img_grad_x = " << img_grad_x.total() * img_grad_x.elemSize() << std::endl;
  std::cout << "img_grad_y = " << img_grad_y.total() * img_grad_y.elemSize() << std::endl;
  std::cout << "img_grad_mag = " << img_grad_mag.total() * img_grad_mag.elemSize() << std::endl;
  */
  /*
  double minVal;
  double maxVal;
  Point minLoc;
  Point maxLoc;
  minMaxLoc(img_grad_mag ,&minVal, &maxVal, &minLoc, &maxLoc);
  std::cout << "grad_x: Min=" << minVal << "; Max=" << maxVal << std::endl;


  //imwrite("/home/dureade/Documents/paparazzi_images/img_grad_mag.bmp", img_grad_mag);
  //imwrite("/home/dureade/Documents/paparazzi_images/abs_grad_y.bmp", abs_grad_y);
  //imwrite("/home/dureade/Documents/paparazzi_images/grad.bmp", grad);
  //imwrite("/home/dureade/Documents/paparazzi_images/img_input_OCV.bmp", img_input_OCV);
   *
   */



  return 1;
}
