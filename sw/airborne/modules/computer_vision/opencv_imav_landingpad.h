//
// Created by jun on 28-6-16.
//

#ifndef OPENCV_IMAV_LANDINGPAD_H
#define OPENCV_IMAV_LANDINGPAD_H

#ifdef __cplusplus
extern "C" {
#endif

struct results {
    int maxx;
    int maxy;
    int MARKER;
};

struct results opencv_imav_landing(char *img, int width, int height, int v_squares, int binary_threshold, int mod);

#ifdef __cplusplus
}
#endif

#endif
