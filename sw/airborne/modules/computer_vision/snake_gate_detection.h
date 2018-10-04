
/*
 * Copyright (C) 2018, Guido de Croon and Michael Ozo
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/snake_gate_detection.h
 *
 *  Detects gates as used in the IROS drone races, i.e., square colored gates. It does so with snake gate detection, a computationally efficient method that works
 *  onboard of the computationally constrained Parrot Bebop 1/2 drones.
 *
 *  An initial version of this algorithm ran in the drone race 2016. The algorithm was first described in:
 *  First autonomous multi-room exploration with an insect-inspired flapping wing vehicle, May 2018,
 *  IEEE International Conference on Robotics and Automation (ICRA 2018), Brisbane, Australia
 *  by Kirk Scheper, Matej Karasek, Christophe De Wagter, Bart Remes, and Guido de Croon
 *  https://www.researchgate.net/publication/327228053_First_autonomous_multi-room_exploration_with_an_insect-inspired_flapping_wing_vehicle
 *
 *  For the drone race, the algorithm and performance are described and analyzed in more detail in:
 *  Autonomous drone race: A novel vision-based navigation and control strategy,
 *  S.Li, M.M.O.I. Ozo, C. De Wagter, G.C.H.E. de Croon.
 *  Submitted.
 */


#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#define MAX_GATES 50

/* Gate structure */
struct gate_img {
  int x;             ///< The image x coordinate of the gate center
  int y;             ///< The image y coordinate of the gate center
  int x_corners[4];///< Array of corner x coordinates
  int y_corners[4];///< Array of corner y coordinates
  int sz;            ///< Half the image size of the gate
  float quality;      ///< gate quality
  int n_sides;       ///< How many sides are orange (to prevent detecting a small gate in the corner of a big one partially out of view).
  float sz_left;     ///< Half the image size of the left side
  float sz_right;    ///< Half the image size of the right side
};

// snake-gate: main function to be called externally.
int snake_gate_detection(struct image_t *img, int n_samples, int min_px_size, float min_gate_quality,
                         float gate_thickness, int min_n_sides,
                         uint8_t color_Ym, uint8_t color_YM, uint8_t color_Um, uint8_t color_UM, uint8_t color_Vm, uint8_t color_VM,
                         struct gate_img *best_gate, struct gate_img *gates_c, int *n_gates, int exclude_top, int exclude_bottom);

// helper functions:
int check_color_snake_gate_detection(struct image_t *im, int x, int y);
void snake_up_and_down(struct image_t *im, int x, int y, int *x_low, int *y_low, int *x_high, int *y_high);
void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *y_low, int *x_high, int *y_high);
void draw_gate(struct image_t *im, struct gate_img gate);
void draw_gate_color_square(struct image_t *im, struct gate_img gate, uint8_t *color);
void draw_gate_color_polygon(struct image_t *im, struct gate_img gate, uint8_t *color);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points);
void check_gate_initial(struct image_t *im, struct gate_img gate, float *quality, int *sides);
void check_gate_outline(struct image_t *im, struct gate_img gate, float *quality, int *n_sides);
float check_inside(struct image_t *im, int x, int y, int sz, int n_samples_in);
void set_gate_points(struct gate_img *gate);
void gate_refine_corners(struct image_t *color_image, int *x_points, int *y_points, int size);
void refine_single_corner(struct image_t *im, int *corner_x, int *corner_y, int size, float size_factor);
int overlap_intervals(int val_low_1, int val_high_1, int val_low_2, int val_high_2);
int intersection_boxes(int x_box_1[4], int y_box_1[4], int x_box_2[4], int y_box_2[4]);
float intersection_over_union(int x_box_1[4], int y_box_1[4], int x_box_2[4], int y_box_2[4]);

