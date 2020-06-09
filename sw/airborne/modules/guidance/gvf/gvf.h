/*
 * Copyright (C) 2016  Hector Garcia de Marina
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_H
#define GVF_H

#define GVF_GRAVITY 9.806

#include "std.h"

/** @typedef gvf_con
* @brief Control parameters for the GVF
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehile to the vector field
* @param error Error signal. It does not have any specific units. It depends on how the trajectory has been implemented. Check the specific wiki entry for each trajectory.
* @param s Defines the direction to be tracked. Its meaning depends on the trajectory and its implementation. Check the wiki entry of the GVF. It takes the values -1 or 1.
*/
typedef struct {
  float ke;
  float kn;
  float error;
  int8_t s;
} gvf_con;

extern gvf_con gvf_control;

enum trajectories {
  LINE = 0,
  ELLIPSE,
  SIN,
  NONE = 255,
};

typedef struct {
  enum trajectories type;
  float p[16];
} gvf_tra;

/** @typedef gvf_seg
* @brief Struct employed by the LINE trajectory for the special case of trackinga segment, which is described by the coordinates x1, y1, x2, y2
* @param seg Tracking a segment or not
* @param x1 coordinate w.r.t. HOME
* @param y1 coordinate w.r.t. HOME
* @param x2 coordinate w.r.t. HOME
* @param y2 coordinate w.r.t. HOME
*/
typedef struct {
  int seg;
  float x1;
  float y1;
  float x2;
  float y2;
} gvf_seg;

extern gvf_tra gvf_trajectory;

struct gvf_grad {
  float nx;
  float ny;
  float nz;
};

struct gvf_Hess {
  float H11;
  float H12;
  float H13;
  float H21;
  float H22;
  float H23;
  float H31;
  float H32;
  float H33;
};

extern void gvf_init(void);
void gvf_control_2D(float ke, float kn, float e,
                    struct gvf_grad *, struct gvf_Hess *);
extern void gvf_set_direction(int8_t s);

// Straigh line
extern bool gvf_line_XY_heading(float x, float y, float heading);
extern bool gvf_line_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2);
extern bool gvf_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2);
extern bool gvf_segment_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_line_wp_heading(uint8_t wp, float heading);


// Ellipse
extern bool gvf_ellipse_wp(uint8_t wp, float a, float b, float alpha);
extern bool gvf_ellipse_XY(float x, float y, float a, float b, float alpha);

// Sinusoidal
extern bool gvf_sin_XY_alpha(float x, float y, float alpha, float w, float off, float A);
extern bool gvf_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off,
                            float A);
extern bool gvf_sin_wp_alpha(uint8_t wp, float alpha, float w, float off,
                             float A);


#endif // GVF_H
