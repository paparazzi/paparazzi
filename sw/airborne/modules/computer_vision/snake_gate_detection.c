
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
 * @file modules/computer_vision/snake_gate_detection.c
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

// Own header
#include "modules/computer_vision/snake_gate_detection.h"
#include <stdio.h>
#include <stdlib.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "paparazzi.h"

// to debug the algorithm, uncomment the define:
// #define DEBUG_SNAKE_GATE

// the return values of the main detection function:
#define SUCCESS_DETECT 1
#define FAIL_DETECT 0

// whether to filter the image and show the best gate(s):
#define FILTER_IMAGE 0
#define DRAW_GATE 1

// Standard colors in UYVY:
uint8_t green_color[4] = {255, 128, 255, 128};
uint8_t blue_color[4] = {0, 128, 0, 128};
uint8_t white_color[4] = {255, 255, 255, 255};

// Filter Settings
uint8_t color_Y_min;
uint8_t color_Y_max;
uint8_t color_U_min;
uint8_t color_U_max;
uint8_t color_V_min;
uint8_t color_V_max;

// Other settings:
int min_pixel_size;

// variable used to count the number of color checks,
// so that we can better restrain the total number of samples taken:
int n_total_samples;

// Result
struct gate_img temp_check_gate;
struct image_t img_result;
float best_quality = 0;
float best_fitness = 100000;

// Support functions:
int cmpfunc(const void *a, const void *b);
int cmp_i(const void *a, const void *b);
float segment_length(struct point_t Q1, struct point_t Q2);

int cmpfunc(const void *a, const void *b)
{
  return (*(const int *)a - * (const int *)b);
}
int *array;
//return indexes
int cmp_i(const void *a, const void *b)
{
  int ia = *(const int *)a;
  int ib = *(const int *)b;
  return array[ia] < array[ib] ? -1 : array[ia] > array[ib];
}


// TODO: NOT FOR A FIRST PULL REQUEST: Since coordinates matter here, we have to deal with the strange sensor mounting in the Parrot Bebop.
//       This leads to checks such as x < im->h... This is a quite fundamental problem, with not a clear solution. However, if a normally
//       mounted sensor is used, the functions here will fail on this exact point...

/**
 * Run snake gate detection on an image. It assumes that it gets images over time, and remembers previous detections.
 *
 * Snake gate takes samples from an image. If a sample corresponds to the target color,
 * it will then go up and down, and afterwards left and right, in order to find chains
 * of pixels of the right color. This forms the initial guess for a square approximation
 * to the gate. Then, the estimate is refined, by looking around the supposed corner locations.
 * This leads to a polygon gate.
 *
 * @param[out] success Whether a gate was detected
 * @param[in] img The input image. We will draw in it.
 * @param[in] n_samples The number of samples taken to find a gate - proportional to the computational effort and detection performance.
 * @param[in] min_px_size The minimum pixel size an initial, purely square detection should have
 * @param[in] min_gate_quality How much percentage of the initial square outline needs to have the target color.
 * @param[in] gate_thickness After snaking, how much the extreme coordinates have to be adjusted.
 * @param[in] color_ym The Y minimum value
 * @param[in] color_yM The Y maximum value
 * @param[in] color_um The U minimum value
 * @param[in] color_uM The U maximum value
 * @param[in] color_vm The V minimum value
 * @param[in] color_vM The V maximum value
 * @param[out] *best_gate This gate_img struct will be filled with the data of the best detected gate.
 * @param[out] *gates_c Array of gates with size MAX_GATES
 * @param[in] exclude_top The number of pixels excluded for sampling at the top of the image.
 * @param[in] exclude_bottom The number of pixels excluded for sampling at the bottom of the image.
 */

int snake_gate_detection(struct image_t *img, int n_samples, int min_px_size, float min_gate_quality,
                         float gate_thickness, int min_n_sides,
                         uint8_t color_Ym, uint8_t color_YM, uint8_t color_Um, uint8_t color_UM, uint8_t color_Vm, uint8_t color_VM,
                         struct gate_img *best_gate, struct gate_img *gates_c, int *n_gates, int exclude_top, int exclude_bottom)
{

  static int last_frame_detection = 0;
  static int repeat_gate = 0;
  static struct gate_img previous_best_gate = {0};
  static struct gate_img last_gate;

  bool check_initial_square = false;
  float iou_threshold = 0.7; // when bigger than this, gates are assumed to represent the same gate


  (*n_gates) = 0;
  // For a new image, set the total number of samples to 0:
  // This number is augmented when checking the color of a pixel.
  n_total_samples = 0;

  color_Y_min = color_Ym;
  color_Y_max = color_YM;
  color_U_min  = color_Um;
  color_U_max  = color_UM;
  color_V_min  = color_Vm;
  color_V_max  = color_VM;
  min_pixel_size = min_px_size;

  int x, y;
  best_quality = 0;
  best_gate->quality = 0;
  (*n_gates) = 0;

  // variables for snake gate detection:
  int y_low = 0;
  int x_l = 0;
  int y_high = 0;
  int x_h = 0;
  int x_low1 = 0;
  int y_l1 = 0;
  int x_high1 = 0;
  int y_h1 = 0;
  int x_low2 = 0;
  int y_l2 = 0;
  int x_high2 = 0;
  int y_h2 = 0;
  int sz = 0;
  int szx1 = 0;
  int szx2 = 0;

  //for (int i = 0; i < n_samples; i++) {
  while (n_total_samples < n_samples) {

    // TODO: would it work better to scan different lines in the image?
    // get a random coordinate:
    x = rand() % img->h;
    y = exclude_top + rand() % (img->w - exclude_top - exclude_bottom);

    // check if it has the right color
    if (check_color_snake_gate_detection(img, x, y)) {

      // fill histogram (TODO: for a next pull request, in which we add the close-by histogram-detection)
      // histogram[x]++;

      // snake up and down:
      snake_up_and_down(img, x, y, &x_l, &y_low, &x_h, &y_high);

      // This assumes the gate to be square:
      sz = y_high - y_low;
      y_low = y_low + (sz * gate_thickness);
      y_high = y_high - (sz * gate_thickness);
      y = (y_high + y_low) / 2;

      // if the found part of the gate is large enough
      if (sz > min_pixel_size) {

        // snake left and right, both for the top and bottom part of the gate:
        snake_left_and_right(img, x_l, y_low, &x_low1, &y_l1,  &x_high1, &y_h1);
        snake_left_and_right(img, x_h, y_high, &x_low2, &y_l2, &x_high2, &y_h2);
        x_low1 = x_low1 + (sz * gate_thickness);
        x_high1 = x_high1 - (sz * gate_thickness);
        x_low2 = x_low2 + (sz * gate_thickness);
        x_high2 = x_high2 - (sz * gate_thickness);

        // sizes of the left-right stretches: in y pixel coordinates
        szx1 = (x_high1 - x_low1);
        szx2 = (x_high2 - x_low2);

        // set the size according to the biggest detection:
        if (szx1 > szx2) {
          // determine the center x based on the bottom part:
          x = (x_high1 + x_low1) / 2;
          // set the size to the largest line found:
          sz = (sz > szx1) ? sz : szx1;
        } else {
          // determine the center x based on the top part:
          x = (x_high2 + x_low2) / 2;
          // set the size to the largest line found:
          sz = (sz > szx2) ? sz : szx2;
        }

        if (sz > min_pixel_size) {
          // create the gate:
          gates_c[(*n_gates)].x = x;
          gates_c[(*n_gates)].y = y;
          // store the half gate size:
          gates_c[(*n_gates)].sz = sz / 2;

          if (check_initial_square) {

            // check the gate quality:
            check_gate_initial(img, gates_c[(*n_gates)], &gates_c[(*n_gates)].quality, &gates_c[(*n_gates)].n_sides);

          } else {

            // The first two corners have a high y:
            gates_c[(*n_gates)].x_corners[0] = x_low2;
            gates_c[(*n_gates)].y_corners[0] = y_l2;
            gates_c[(*n_gates)].x_corners[1] = x_high2;
            gates_c[(*n_gates)].y_corners[1] = y_h2;

            // The third and fourth corner have a low y:
            gates_c[(*n_gates)].x_corners[2] = x_high1;
            gates_c[(*n_gates)].y_corners[2] = y_h1;
            gates_c[(*n_gates)].x_corners[3] = x_low1;
            gates_c[(*n_gates)].y_corners[3] = y_l1;

            // check the polygon:
            check_gate_outline(img, gates_c[(*n_gates)], &gates_c[(*n_gates)].quality, &gates_c[(*n_gates)].n_sides);
          }

          if (gates_c[(*n_gates)].quality > best_quality) {
            best_quality = gates_c[(*n_gates)].quality;
          }

          // set the corners to make a square gate for now:
          set_gate_points(&gates_c[(*n_gates)]);


          bool add_gate = true;
          float iou;
          for (int g = 0; g < (*n_gates); g++) {
            iou = intersection_over_union(gates_c[g].x_corners, gates_c[g].y_corners, gates_c[(*n_gates)].x_corners,
                                          gates_c[(*n_gates)].y_corners);
            if (iou > iou_threshold) {
              // we are looking at an existing gate:
              add_gate = false;

              if (gates_c[g].quality > gates_c[(*n_gates)].quality) {
                // throw the current gate away:
                break;
              } else {
                // throw the old gate away:
                // TODO: consider making a function for doing this "deep" copy
                add_gate = true;
                gates_c[g].x = gates_c[(*n_gates)].x;
                gates_c[g].y = gates_c[(*n_gates)].y;
                gates_c[g].sz = gates_c[(*n_gates)].sz;
                gates_c[g].quality = gates_c[(*n_gates)].quality;
                memcpy(gates_c[g].x_corners, gates_c[(*n_gates)].x_corners, sizeof(int) * 4);
                memcpy(gates_c[g].y_corners, gates_c[(*n_gates)].y_corners, sizeof(int) * 4);
              }
            }
          }

          if (add_gate) {
            (*n_gates)++;
          }
        }

        if ((*n_gates) >= MAX_GATES) {
          break;
        }
      }
    }
  }

#ifdef DEBUG_SNAKE_GATE
  // draw all candidates:
  printf("(*n_gates):%d\n", (*n_gates));
  for (int i = 0; i < (*n_gates); i++) {
    //draw_gate_color_square(img, gates_c[i], white_color);
    draw_gate_color_polygon(img, gates_c[i], white_color);
  }
#endif

  //init best gate
  best_gate->quality = 0;
  best_gate->n_sides = 0;
  repeat_gate = 0;
  float sz1 =0;
  float sz2 = 0;

  // do an additional fit to improve the gate detection:
  if ((best_quality > min_gate_quality && (*n_gates) > 0) || last_frame_detection) {

    // go over all remaining gates:
    for (int gate_nr = 0; gate_nr < (*n_gates); gate_nr++) {

      // get gate information:
      gate_refine_corners(img, gates_c[gate_nr].x_corners, gates_c[gate_nr].y_corners, gates_c[gate_nr].sz);

      // also get the color fitness
      check_gate_outline(img, gates_c[gate_nr], &gates_c[gate_nr].quality, &gates_c[gate_nr].n_sides);

      // If the gate is good enough:
      float sz1g, sz2g;
      sz1g = (float) (gates_c[gate_nr].x_corners[1] - gates_c[gate_nr].x_corners[0]);
      sz2g = (float) (gates_c[gate_nr].y_corners[1] - gates_c[gate_nr].y_corners[2]);

      // Don't accept gates that look too rectangular (not square enough)
      float ratio;
      static float limit_ratio = 1.5;
      if(sz1g > 0.1 && sz2g > 0.1) {
        ratio = (sz1g >= sz2g) ? sz1g / sz2g : sz2g / sz1g;
      }
      else {
        ratio = limit_ratio + 0.1;
      }

      // Old way: prefer the highest quality gate (most orange one):
      // if (gates_c[gate_nr].n_sides >= min_n_sides && gates_c[gate_nr].quality > best_gate->quality) {

      // Prefer a bigger gate as best gate:
      if (sz1g*sz2g > sz1*sz2 && gates_c[gate_nr].quality > min_gate_quality * 2 && gates_c[gate_nr].n_sides >= min_n_sides && ratio <= limit_ratio) {
        // store the information in the gate:
        best_gate->x = gates_c[gate_nr].x;
        best_gate->y = gates_c[gate_nr].y;
        best_gate->sz = gates_c[gate_nr].sz;
        best_gate->sz_left = gates_c[gate_nr].sz_left;
        best_gate->sz_right = gates_c[gate_nr].sz_right;
        best_gate->quality = gates_c[gate_nr].quality;
        best_gate->n_sides = gates_c[gate_nr].n_sides;
        memcpy(best_gate->x_corners, gates_c[gate_nr].x_corners, sizeof(best_gate->x_corners));
        memcpy(best_gate->y_corners, gates_c[gate_nr].y_corners, sizeof(best_gate->y_corners));
        sz1 = (float) (best_gate->x_corners[1] - best_gate->x_corners[0]);
        sz2 = (float) (best_gate->y_corners[1] - best_gate->y_corners[2]);

      }
    }

    // if the best gate is not good enough, but we did have a detection in the previous image:
    if ((best_gate->quality == 0 && best_gate->n_sides == 0) && last_frame_detection == 1) {

      // TODO: is it really important to do this sorting here to get the maximum size? Is the sz property not accurate enough?
      // Or can we not assume the standard arrangement of the corners?
      int x_values[4];
      int y_values[4];
      memcpy(x_values, last_gate.x_corners, sizeof(x_values));
      memcpy(y_values, last_gate.y_corners, sizeof(y_values));
      //sort small to large
      qsort(x_values, 4, sizeof(int), cmpfunc);
      qsort(y_values, 4, sizeof(int), cmpfunc);
      //check x size, maybe use y also later?
      int radius_p   = x_values[3] - x_values[0];
      // TODO: is 2*radius_p not huge?
      gate_refine_corners(img, last_gate.x_corners, last_gate.y_corners, 2 * radius_p);

      // also get the color fitness
      check_gate_outline(img, last_gate, &last_gate.quality, &last_gate.n_sides);

      // if the refined detection is good enough:
      if (last_gate.n_sides >= min_n_sides && last_gate.quality > best_gate->quality) {
        repeat_gate = 1;
        best_gate->quality = last_gate.quality;
        best_gate->n_sides = last_gate.n_sides;
        memcpy(best_gate->x_corners, last_gate.x_corners, sizeof(best_gate->x_corners));
        memcpy(best_gate->y_corners, last_gate.y_corners, sizeof(best_gate->y_corners));
      }
    }

#ifdef DEBUG_SNAKE_GATE
    // draw the best gate:
    draw_gate(img, (*best_gate));
#endif
  }

  // prepare for the next time:
  previous_best_gate.x = best_gate->x;
  previous_best_gate.y = best_gate->y;
  previous_best_gate.sz = best_gate->sz;
  previous_best_gate.sz_left = best_gate->sz_left;
  previous_best_gate.sz_right = best_gate->sz_right;
  previous_best_gate.quality = best_gate->quality;
  previous_best_gate.n_sides = best_gate->n_sides;
  memcpy(previous_best_gate.x_corners, best_gate->x_corners, sizeof(best_gate->x_corners));
  memcpy(previous_best_gate.y_corners, best_gate->y_corners, sizeof(best_gate->y_corners));

  //color filtered version of image for overlay and debugging
  if (FILTER_IMAGE) { //filter) {
    image_yuv422_colorfilt(img, img, color_Y_min, color_Y_max, color_U_min, color_U_max, color_V_min, color_V_max);
  }

  if (best_gate->quality > (min_gate_quality * 2) && best_gate->n_sides >= min_n_sides) {
    // successful detection
    last_frame_detection = 1;

    //draw_gate_color(img, best_gate, blue_color);
    if (DRAW_GATE) {
      for (int gate_nr = 0; gate_nr < (*n_gates); gate_nr++) {
        if (gates_c[gate_nr].n_sides >= min_n_sides && gates_c[gate_nr].quality > 2 * min_gate_quality) {
          // draw the best gate:
          // draw_gate(img, gates_c[gate_nr]);
          draw_gate_color_polygon(img, gates_c[gate_nr], green_color);
        }
      }

      int size_crosshair = 10;
      if (repeat_gate == 0) {
        draw_gate_color_polygon(img, (*best_gate), blue_color);
      } else if (repeat_gate == 1) {
        draw_gate_color_polygon(img, (*best_gate), green_color);
        for (int i = 0; i < 3; i++) {
          struct point_t loc = { .x = last_gate.x_corners[i], .y = last_gate.y_corners[i] };
          image_draw_crosshair(img, &loc, blue_color, size_crosshair);
        }
      }
    }
    //save for next iteration
    memcpy(last_gate.x_corners, best_gate->x_corners, sizeof(best_gate->x_corners));
    memcpy(last_gate.y_corners, best_gate->y_corners, sizeof(best_gate->y_corners));
    //previous best snake gate
    last_gate.x = best_gate->x;
    last_gate.y = best_gate->y;
    last_gate.sz = best_gate->sz;

    //SIGNAL NEW DETECTION AVAILABLE
    return SUCCESS_DETECT;

  } else {
    //no detection
    last_frame_detection = 0;
    return FAIL_DETECT;
  }
}


/**
 * Draw the gate on an image.
 *
 * @param[in] img The output image.
 * @param[in] gate The gate to be drawn.
 */
void draw_gate(struct image_t *im, struct gate_img gate)
{
  draw_gate_color_polygon(im, gate, white_color);
}


/**
 * Draw the gate on an image, using the corner points, possibly resulting in a polygon.
 *
 * @param[in] img The output image.
 * @param[in] gate The gate to be drawn.
 * @param[in] color The color of the lines, in UYVY format.
 */
void draw_gate_color_polygon(struct image_t *im, struct gate_img gate, uint8_t *color)
{
  // Please note that here we use functions in image.h, so we have to inverse the coordinates:
  // draw four lines and a crosshair on the image:
  struct point_t from, to;

  // a cross at the center
  from.x = gate.y;
  from.y = gate.x;
  image_draw_crosshair(im, &from, color, 10);

  // the four lines:
  from.x = gate.y_corners[0];
  from.y = gate.x_corners[0];
  to.x = gate.y_corners[1];
  to.y = gate.x_corners[1];
  image_draw_line_color(im, &from, &to, color);

  from.x = gate.y_corners[1];
  from.y = gate.x_corners[1];
  to.x = gate.y_corners[2];
  to.y = gate.x_corners[2];
  image_draw_line_color(im, &from, &to, color);

  from.x = gate.y_corners[2];
  from.y = gate.x_corners[2];
  to.x = gate.y_corners[3];
  to.y = gate.x_corners[3];
  image_draw_line_color(im, &from, &to, color);

  from.x = gate.y_corners[3];
  from.y = gate.x_corners[3];
  to.x = gate.y_corners[0];
  to.y = gate.x_corners[0];
  image_draw_line_color(im, &from, &to, color);

}



/**
 * Draw the gate on an image, using only the center coordinate and sizes - resulting in a square gate.
 *
 * @param[in] img The output image.
 * @param[in] gate The gate to be drawn.
 * @param[in] color The color of the lines, in UYVY format.
 */
void draw_gate_color_square(struct image_t *im, struct gate_img gate, uint8_t *color)
{
  // Please note that here we use functions in image.h, so we have to inverse the coordinates:
  // draw four lines and a crosshair on the image:
  struct point_t from, to;

  from.x = gate.y;
  from.y = gate.x;
  image_draw_crosshair(im, &from, color, 10);

  if (gate.sz_left == 0) { gate.sz_left = gate.sz; }
  if (gate.sz_right == 0) { gate.sz_right = gate.sz; }

  from.x = gate.y - gate.sz_left;
  from.y = gate.x - gate.sz;
  to.x = gate.y + gate.sz_left;
  to.y = gate.x - gate.sz;
  image_draw_line_color(im, &from, &to, color);
  from.x = gate.y + gate.sz_left;
  from.y = gate.x - gate.sz;
  to.x = gate.y + gate.sz_right;
  to.y = gate.x + gate.sz;
  image_draw_line_color(im, &from, &to, color);
  from.x = gate.y + gate.sz_right;
  from.y = gate.x + gate.sz;
  to.x = gate.y - gate.sz_right;
  to.y = gate.x + gate.sz;
  image_draw_line_color(im, &from, &to, color);
  from.x = gate.y - gate.sz_right;
  from.y = gate.x + gate.sz;
  to.x = gate.y - gate.sz_left;
  to.y = gate.x - gate.sz;
  image_draw_line_color(im, &from, &to, color);
}

/**
 * Check only the outline of the gate. The gate must have corners already assigned in the struct.
 *
 * @param[in] im The input image.
 * @param[in] gate The gate to be checked.
 * @param[in] quality The ratio of rightly colored pixels, 1.0 is best, 0.0 is worst.
 * @param[in] sides How many of the sides are sufficiently colored?
 */
void check_gate_outline(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;

  // how much of the side should be visible to count as a detected side?
  // TODO: make this a setting. Note: the number is different from the check_gate_initial function.
  float min_ratio_side = 0.4;
  (*n_sides) = 0;

  float min_segment_length = min_pixel_size;

  // check the four lines of which the gate consists:
  struct point_t from, to;

  from.x = gate.x_corners[0];
  from.y = gate.y_corners[0];
  to.x = gate.x_corners[1];
  to.y = gate.y_corners[1];
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side && segment_length(from, to) > min_segment_length) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x_corners[1];
  from.y = gate.y_corners[1];
  to.x = gate.x_corners[2];
  to.y = gate.y_corners[2];
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side && segment_length(from, to) > min_segment_length) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x_corners[2];
  from.y = gate.y_corners[2];
  to.x = gate.x_corners[3];
  to.y = gate.y_corners[3];
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side && segment_length(from, to) > min_segment_length) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x_corners[3];
  from.y = gate.y_corners[3];
  to.x = gate.x_corners[0];
  to.y = gate.y_corners[0];
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side && segment_length(from, to) > min_segment_length) {
    (*n_sides)++;
  }

  n_points += np;
  n_colored_points += nc;

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }


  // Check the inside of the gate - again:
  static int n_samples_in = 100;
  static float center_discard_threshold = 0.25;
  gate.sz = gate.x_corners[1] - gate.x_corners[0];
  float center_factor = check_inside(im, gate.x, gate.y, gate.sz, n_samples_in);
  if (center_factor > center_discard_threshold) {
    (*quality) = 0;
  }
}


/**
 * Check the outline and the center of the gate. The outline should be of the right color.
 * The inner part should be a different color, or else we may be looking at a distractor object.
 * The gate does not yet have to have corners assigned in the struct.
 *
 * @param[in] im The input image.
 * @param[in] gate The gate to be checked.
 * @param[in] quality The ratio of rightly colored pixels, 1.0 is best, 0.0 is worst.
 *            If too many pixels inside the gate are of the right color, the quality is also 0.
 *  @param[in] sides How many of the sides are sufficiently colored?
 */
extern void check_gate_initial(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;

  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;

  from.x = gate.x - gate.sz;
  from.y = gate.y - gate.sz_left;
  to.x = gate.x - gate.sz;
  to.y = gate.y + gate.sz_left;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x - gate.sz;
  from.y = gate.y + gate.sz_left;
  to.x = gate.x + gate.sz;
  to.y = gate.y + gate.sz_right;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y + gate.sz_right;
  to.x = gate.x + gate.sz;
  to.y = gate.y - gate.sz_right;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y - gate.sz_right;
  to.x = gate.x - gate.sz;
  to.y = gate.y - gate.sz_left;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }

  n_points += np;
  n_colored_points += nc;


  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }

  // check that the inside of the gate is not of the target color as well:
  int n_samples_in = 100;
  float center_discard_threshold = 0.25;
  float center_factor = check_inside(im, gate.x, gate.y, gate.sz, n_samples_in);
  if (center_factor > center_discard_threshold) {
    (*quality) = 0;
  }


}

/* Check inside of a gate, in order to exclude solid areas.
 *
 * @param[out] center_factor The ratio of pixels inside the box that are of the right color.
 * @param[in] im The YUV422 image.
 * @param[in] x The center x-coordinate of the gate
 * @param[in] y The center y-coordinate of the gate
 * @param[in] sz The size of the gate - when approximated as square.
 * @param[in] n_samples_in The number of samples used to determine the ratio.
 */

float check_inside(struct image_t *im, int x, int y, int sz, int n_samples_in)
{
  int num_color_center = 0;
  int n_samples = 0;

  if (sz == 0) {
    return 1.0f;
  }

  for (int i = 0; i < n_samples_in; i++) {
    // get a random coordinate:
    int x_in = x + (rand() % sz) - (0.5 * sz);
    int y_in = y + (rand() % sz) - (0.5 * sz);

    if (y_in >= 0 && y_in < im->w && x_in >= 0 && x_in < im->h) {
      n_samples++;
      // check if it has the right color
      if (check_color_snake_gate_detection(im, x_in, y_in)) {
        num_color_center ++;
      }
    }
  }

  //how much center pixels colored?
  if (n_samples == 0) {
    return 1.0f;
  }

  float center_factor = 0;
  if (n_samples != 0) {
    center_factor = num_color_center / (float)n_samples;
  }
  return center_factor;
}

/**
 * Determine the segment length between two 2D-points.
 *
 * @param[in] Q1 Point 1.
 * @param[in] Q2 Point 2.
 */
float segment_length(struct point_t Q1, struct point_t Q2)
{

  float r = sqrtf((Q1.x - Q2.x) * (Q1.x - Q2.x) + (Q1.y - Q2.y) * (Q1.y - Q2.y));
  return r;
}

/**
 * Checks whether points on a line between two 2D-points are of a given color.
 *
 * @param[in] im The input image.
 * @param[in] Q1 Point 1.
 * @param[in] Q2 Point 2.
 * @param[in] n_points The number of sampled points.
 * @param[in] n_colored_points The number of sampled points that were of the right color.
 */
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points)
{

  (*n_points) = 0;
  (*n_colored_points) = 0;

  // t_step determines how many samples are taken (1.0 / t_step)
  float t_step = 0.05;
  int x, y;
  float t;
  // go from Q1 to Q2 in 1/t_step steps:
  for (t = 0.0f; t < 1.0f; t += t_step) {
    // determine integer coordinate on the line:
    x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
    y = (int)(t * Q1.y + (1.0f - t) * Q2.y);

    // Is the point in the image?
    if (x >= 0 && x < im->h && y >= 0 && y < im->w) {
      // augment number of checked points:
      (*n_points)++;

      if (check_color_snake_gate_detection(im, x, y)) {
        // the point is of the right color:
        (*n_colored_points)++;
      }
    }
  }
}

/**
 * The actual snaking. An "agent" starts at (x,y) and goes as far as possible up and down (y-direction),
 * keeping a chain of "connected" target color pixels. The agent can go slightly to the right and left.
 *
 * @param[in] im The input image.
 * @param[in] x The initial x-coordinate
 * @param[in] y The initial y-coordinate
 * @param[in] x_low The x-coordinate that corresponds to the current y_low.
 * @param[in] y_low The current lowest y-estimate
 * @param[in] x_high The x-coordinate that corresponds to the current y_high
 * @param[in] y_high The current highest y-estimate
 */
void snake_up_and_down(struct image_t *im, int x, int y, int *x_low, int *y_low, int *x_high, int *y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;

  // TODO: perhaps it is better to put the big steps first, as to reduce computation.
  // snake towards negative y
  while ((*y_low) > 0 && !done) {
    if (check_color_snake_gate_detection(im, x, (*y_low) - 1)) {
      (*y_low)--;
    } else if ((*y_low) - 2 >= 0 && check_color_snake_gate_detection(im, x, (*y_low) - 2)) {
      (*y_low) -= 2;
    } else if (x + 1 < im->h && check_color_snake_gate_detection(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
    } else if (x - 1 >= 0 && check_color_snake_gate_detection(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
    } else {
      done = 1;
      (*x_low) = x;
    }
  }

  // snake towards positive y
  x = x_initial;
  (*y_high) = y;
  done = 0;
  while ((*y_high) < im->w - 1 && !done) {
    if (check_color_snake_gate_detection(im, x, (*y_high) + 1)) {
      (*y_high)++;
    } else if ((*y_high) < im->w - 2 && check_color_snake_gate_detection(im, x, (*y_high) + 2)) {
      (*y_high) += 2;
    } else if (x < im->h - 1 && check_color_snake_gate_detection(im, x + 1, (*y_high) + 1)) {
      x++;
      (*y_high)++;
    } else if (x > 0 && check_color_snake_gate_detection(im, x - 1, (*y_high) + 1)) {
      x--;
      (*y_high)++;
    } else {
      done = 1;
      (*x_high) = x;
    }
  }
}


/**
 * The actual snaking. An "agent" starts at (x,y) and goes as far as possible left and right (x-direction),
 * keeping a chain of "connected" target color pixels. The agent can go slightly to the top and bottom.
 *
 * @param[in] im The input image.
 * @param[in] x The initial x-coordinate
 * @param[in] y The initial y-coordinate
 * @param[in] x_low The current lowest x-estimate
 * @param[in] y_low The y-coordinate that corresponds to the current x_low
 * @param[in] x_high The current highest x-estimate
 * @param[in] y_high The y-coordinate that corresponds to the current x_high
 */
void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *y_low, int *x_high, int *y_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color_snake_gate_detection(im, (*x_low) - 1, y)) {
      (*x_low)--;
    } else if ((*x_low) > 1 && check_color_snake_gate_detection(im, (*x_low) - 2, y)) {
      (*x_low) -= 2;
    } else if (y < im->w - 1 && check_color_snake_gate_detection(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
    } else if (y > 0 && check_color_snake_gate_detection(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
    } else {
      done = 1;
      (*y_low) = y;
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  while ((*x_high) < im->h - 1 && !done) {
    if (check_color_snake_gate_detection(im, (*x_high) + 1, y)) {
      (*x_high)++;
    } else if ((*x_high) < im->h - 2 && check_color_snake_gate_detection(im, (*x_high) + 2, y)) {
      (*x_high) += 2;
    } else if (y < im->w - 1 && check_color_snake_gate_detection(im, (*x_high) + 1, y++)) {
      y++;
      (*x_high)++;
    } else if (y > 0 && check_color_snake_gate_detection(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
    } else {
      done = 1;
      (*y_high) = y;
    }
  }
}

/**
 * Determine and set the corner locations in gate.x_corners, g.y_corners, based
 * on the center of the gate and its size, assuming a square window.
 *
 * @param[in] gate The gate struct with the relevant information.
 */
void set_gate_points(struct gate_img *gate)
{
  // In Parrot Bebop coordinates, this goes from bottom-right CCW:
  gate->x_corners[0] = gate->x - gate->sz;
  gate->y_corners[0] = gate->y + gate->sz;
  gate->x_corners[1] = gate->x + gate->sz;
  gate->y_corners[1] = gate->y + gate->sz;
  gate->x_corners[2] = gate->x + gate->sz;
  gate->y_corners[2] = gate->y - gate->sz;
  gate->x_corners[3] = gate->x - gate->sz;
  gate->y_corners[3] = gate->y - gate->sz;

}

/**
 * Refine the four corners of the gate, based on the color around the supposed corner locations.
 *
 * @param[in] color_image The color image.
 * @param[in] x_points The x-coordinates of the gate. These will be updated.
 * @param[in] y_points The y-coordinates of the gate. These will be updated.
 * @param[in] size Full size of the gate.
 */
void gate_refine_corners(struct image_t *color_image, int *x_points, int *y_points, int size)
{

  // TODO: make parameter?
  float corner_area = 0.3f;
  refine_single_corner(color_image, x_points, y_points, size, corner_area);
  refine_single_corner(color_image, &(x_points[1]), &(y_points[1]), size, corner_area);
  refine_single_corner(color_image, &(x_points[2]), &(y_points[2]), size, corner_area);
  refine_single_corner(color_image, &(x_points[3]), &(y_points[3]), size, corner_area);
}

/**
 * Refine a single corner, based on the color around the coordinate.
 *
 * @param[in] im The color image.
 * @param[in] corner_x The corner's initial x-coordinate
 * @param[in] corner_y The corner's initial y-coordinate
 * @param[in] size Full size of the gate.
 * @param[in] size_factor The ratio of the size in which we will look for a better corner location.
 */
void refine_single_corner(struct image_t *im, int *corner_x, int *corner_y, int size, float size_factor)
{

  float x_corner_f = (float)(*corner_x);
  float y_corner_f = (float)(*corner_y);
  float size_f     = (float)size;
  size_factor = 0.4f;

  int x_l = (int)(x_corner_f - size_f * size_factor);
  Bound(x_l, 0, im->h);
  int x_r = (int)(x_corner_f + size_f * size_factor);
  Bound(x_r, 0, im->h);
  int y_h = (int)(y_corner_f + size_f * size_factor);
  Bound(y_h, 0, im->w);
  int y_l = (int)(y_corner_f - size_f * size_factor);
  Bound(y_l, 0, im->w);


#ifdef DEBUG_SNAKE_GATE
  // draw the box of refinement:
  struct gate_img box;
  box.x_corners[0] = x_l;
  box.y_corners[0] = y_l;
  box.x_corners[1] = x_l;
  box.y_corners[1] = y_h;
  box.x_corners[2] = x_r;
  box.y_corners[2] = y_h;
  box.x_corners[3] = x_r;
  box.y_corners[3] = y_l;
  draw_gate_color_polygon(im, box, green_color); // becomes grey, since it is called before the filtering...
#endif

  int x_size = x_r - x_l + 1;
  int y_size = y_h - y_l + 1;

  int x_hist[x_size];
  int y_hist[y_size];
  memset(x_hist, 0, sizeof(int)*x_size);
  memset(y_hist, 0, sizeof(int)*y_size);


  int best_x = 0;
  int best_x_loc = x_l;
  int x_best_start = x_l;
  int best_y = 0;
  int best_y_loc = y_l;
  int y_best_start = y_l;


  for (int y_pix = y_l; y_pix < y_h; y_pix++) {
    for (int x_pix = x_l; x_pix < x_r; x_pix++) {
      if (check_color_snake_gate_detection(im, x_pix, y_pix) > 0) {

        int cur_x = x_hist[x_pix - x_l];
        int cur_y = y_hist[y_pix - y_l];
        x_hist[x_pix - x_l] = cur_x + 1;
        y_hist[y_pix - y_l] = cur_y + 1;

        if (x_hist[x_pix - x_l] > best_x) {
          best_x = x_hist[x_pix - x_l];
          best_x_loc = x_pix;
          x_best_start = x_pix;
        } else if (cur_x == best_x) {
          best_x_loc = (x_pix + x_best_start) / 2;
        }
        if (y_hist[y_pix - y_l] > best_y) {
          best_y = y_hist[y_pix - y_l];
          best_y_loc = y_pix;
          y_best_start = y_pix;
        } else if (cur_y == best_y) {
          best_y_loc = (y_pix + y_best_start) / 2;
        }

      }
    }
  }

  // Update the corner location:
  *corner_x = best_x_loc;
  *corner_y = best_y_loc;
}


/* Check the color of a pixel, within the snake gate detection scheme
 * @param[in] *im The YUV422 color image
 * @param[in] x The image x-coordinate of the pixel
 * @param[in] y The image y-coordinate of the pixel
 * @param[out] success Whether the pixel is the right color (1) or not (0)
 */
int check_color_snake_gate_detection(struct image_t *im, int x, int y)
{

  // Call the function in image.c with the color thresholds:
  // Please note that we have to switch x and y around here, due to the strange sensor mounting in the Bebop:
  int success = check_color_yuv422(im, y, x, color_Y_min, color_Y_max, color_U_min, color_U_max, color_V_min,
                                   color_V_max);
  n_total_samples++;
  /*
  #ifdef DEBUG_SNAKE_GATE
    if(success) {
      set_color_yuv422(im, y, x, 0, 0, 0);
    }
  #endif
  */

  return success;
}


/* Calculate the intersection over union of two boxes. The boxes are organized as the locations of the gate's corners.
 * So, from top left clock-wise.
 *
 * @param[in] x_box_1 The x-coordinates of the first box's corners
 * @param[in] y_box_1 The y-coordinates of the first box's corners
 * @param[in] x_box_2 The x-coordinates of the second box's corners
 * @param[in] y_box_2 The y-coordinates of the second box's corners
 * @param[out] The ratio of the intersection of the two boxes divided by their union.
 */
float intersection_over_union(int x_box_1[4], int y_box_1[4], int x_box_2[4], int y_box_2[4])
{

  // TODO: please note that the order of the indices here depends on the set_gate_points function.
  // A future pull request might adapt the indexing automatically to the chosen order in that function.
  float iou;

  // intersection:
  int intersection = intersection_boxes(x_box_1, y_box_1, x_box_2, y_box_2);

  // union:
  int w1, h1, w2, h2, un;
  w1 = x_box_1[1] - x_box_1[0];
  h1 = y_box_1[0] - y_box_1[2];
  w2 = x_box_2[1] - x_box_2[0];
  h2 = y_box_2[0] - y_box_2[2];
  un = w1 * h1 + w2 * h2 - intersection;

  // ratio of intersection over union:
  if (un == 0) {
    iou = 1.0f;
  } else {
    iou = (float) intersection / (float) un;
  }

  return iou;
}

/* Calculate the intersection of two boxes.
 *
 * @param[in] x_box_1 The x-coordinates of the first box's corners
 * @param[in] y_box_1 The y-coordinates of the first box's corners
 * @param[in] x_box_2 The x-coordinates of the second box's corners
 * @param[in] y_box_2 The y-coordinates of the second box's corners
 * @param[out] The number of pixels in the intersection area.
 */
int intersection_boxes(int x_box_1[4], int y_box_1[4], int x_box_2[4], int y_box_2[4])
{

  int width = overlap_intervals(x_box_1[0], x_box_1[1], x_box_2[0], x_box_2[1]);
  int height = overlap_intervals(y_box_1[2], y_box_1[0], y_box_2[2], y_box_2[0]);

  return width * height;
}

/* Calculate the overlap of two 1-dimensional intervals.
 * @param[in] val_low_1 The low value of the first interval.
 * @param[in] val_high_1 The high value of the first interval.
 * @param[in] val_low_2 The low value of the second interval.
 * @param[in] val_high_2 The low value of the second interval.
 * @param[out] int Number of overlapping units (pixels).
 */
int overlap_intervals(int val_low_1, int val_high_1, int val_low_2, int val_high_2)
{

  int overlap;
  int min_val;
  if (val_low_2 < val_low_1) {
    if (val_high_2 < val_low_1) {
      overlap = 0;
    } else {
      min_val = (val_high_1 > val_high_2) ? val_high_2 : val_high_1;
      overlap = min_val - val_low_1;
    }
  } else {
    if (val_high_1 < val_low_2) {
      overlap = 0;
    } else {
      min_val = (val_high_1 > val_high_2) ? val_high_2 : val_high_1;
      overlap = min_val - val_low_2;
    }
  }

  return overlap;
}
