/*
 * Copyright (C) Wilco Vlenterie (wv-tud)
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
 * @file "modules/computer_vision//cv_active_random_filter.c"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */
#include "std.h"

#define AR_FILTER_MAX_OBJECTS   19  // Maximum nr of objects

typedef struct _trackResults {
    uint16_t     x_p;
    uint16_t     y_p;
    uint32_t  area_p;
    double 	x_c;
    double 	y_c;
    double 	r_c;
    double  x_b;
    double  y_b;
    double  z_b;
    double  x_w;
    double  y_w;
    double  z_w;
} trackResults;

typedef struct _memBlock {
	uint32_t lastSeen;
	uint16_t id;
	uint16_t x_p;
	uint16_t y_p;
	uint32_t area_p;
	double r_c;
	double x_w;
	double y_w;
	double z_w;
} memoryBlock;


extern double 		AR_FILTER_IMAGE_CROP_FOVY;
extern uint8_t 		AR_FILTER_Y_MIN;
extern uint8_t 		AR_FILTER_Y_MAX;
extern uint8_t 		AR_FILTER_U_MIN;
extern uint8_t 		AR_FILTER_U_MAX;
extern uint8_t 		AR_FILTER_V_MIN;
extern uint8_t 		AR_FILTER_V_MAX;
extern uint16_t     AR_FILTER_RND_PIX_SAMPLE;
extern uint16_t     AR_FILTER_MIN_CROP_AREA;
extern uint16_t     AR_FILTER_MAX_LAYERS;
extern uint16_t     AR_FILTER_MIN_LAYERS;
extern uint16_t     AR_FILTER_MIN_POINTS;
extern double       AR_FILTER_MIN_CIRCLE_SIZE;
extern double       AR_FILTER_MAX_CIRCLE_DEF;
extern uint8_t      AR_FILTER_FLOOD_STYLE;
extern uint8_t      AR_FILTER_SAMPLE_STYLE;
extern uint8_t      AR_FILTER_CDIST_YTHRES;
extern uint8_t      AR_FILTER_CDIST_UTHRES;
extern uint8_t      AR_FILTER_CDIST_VTHRES;
extern double       default_k;
extern uint16_t     default_calArea;
extern double       perspective_zCor;
extern double       scale_f;

// Filter sample styles
#define AR_FILTER_STYLE_FULL   0
#define AR_FILTER_STYLE_GRID   1
#define AR_FILTER_STYLE_RANDOM 2
// Filter flood styles
#define AR_FILTER_FLOOD_OMNI   0
#define AR_FILTER_FLOOD_CW     1
// Filter flood directions
#define ARF_NONE      -1
#define ARF_SEARCH     0
#define ARF_UP         1
#define ARF_UP_RIGHT   2
#define ARF_RIGHT      3
#define ARF_RIGHT_DOWN 4
#define ARF_DOWN       5
#define ARF_DOWN_LEFT  6
#define ARF_LEFT       7
#define ARF_LEFT_UP    8
// Filter return status
#define ARF_FINISHED   2
#define ARF_SUCCESS    1
#define ARF_NO_FOUND   0
#define ARF_DUPLICATE -1
#define ARF_ERROR     -2

#ifdef __cplusplus
extern "C" {
#endif

void active_random_filter_init(void);
void active_random_filter(char* buff, uint16_t width, uint16_t height, struct FloatEulers* eulerAngles);

#ifdef __cplusplus
}
#endif
