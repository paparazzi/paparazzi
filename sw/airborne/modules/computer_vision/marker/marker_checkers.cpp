
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

#include "marker_checkers.h"

Mat img_marker;

void init_detect_checkers(void) {

    img_marker = imread("/data/ftp/internal_000/imav/marker_checkers.png", IMREAD_GRAYSCALE);
}


struct resultsc opencv_detect_checkers(char *img, int width, int height) {

    struct resultsc marker;

    Mat M(height, width, CV_8UC2, img);
    Mat image;

    cvtColor(M, image, CV_YUV2GRAY_Y422);

    circle(image, Point(120, 120), 20, 200, 10);

    grayscale_opencv_to_yuv422(image, img, width, height);

    return marker;
}