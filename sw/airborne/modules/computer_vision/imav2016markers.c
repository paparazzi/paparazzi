/*
 * Copyright (C) IMAV 2016
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
 * @file "modules/computer_vision/imav2016markers.c"
 * @author IMAV 2016
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "imav2016markers.h"

/**
 * Filter colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @param[in] mod Image-modification trigger
 * @return The amount of filtered pixels and the average sum of their (X,Y) locations
 */
struct colorfilter_t colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M,
                                     uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint8_t mod)
{

  struct colorfilter_t blob;

  blob.x_pos = 0;
  blob.y_pos = 0;
  blob.cnt = 0;

  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if (
        (dest[1] >= y_m)
        && (dest[1] <= y_M)
        && (dest[0] >= u_m)
        && (dest[0] <= u_M)
        && (dest[2] >= v_m)
        && (dest[2] <= v_M)
        ) {
        blob.cnt ++;
        blob.x_pos += x;
        blob.y_pos += y;

        // Check if the image needs to be modified
        if (mod >0) {
          // UYVY
          dest[0] = 64;         // U
          dest[1] = source[1];  // Y
          dest[2] = 255;        // V
          dest[3] = source[3];  // Y
        }

      } else {

        // Check if the image needs to be modified
        if (mod >0) {
          // UYVY
          char u = source[0] - 127;
          u /= 4;
          dest[0] = 127;        // U
          dest[1] = source[1];  // Y
          u = source[2] - 127;
          u /= 4;
          dest[2] = 127;        // V
          dest[3] = source[3];  // Y
        }
      }

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
  return blob;
}


/**
 * Filter white in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] mod Image-modification trigger
 * @return The amount of filtered pixels and the average sum of their (X,Y) locations
 */
struct colorfilter_t grayfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t mod)
{

  struct colorfilter_t blob;

  blob.x_pos = 0;
  blob.y_pos = 0;
  blob.cnt = 0;

  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if ((dest[1] >= y_m) && (dest[1] <= y_M)) {
        blob.cnt ++;
        blob.x_pos += x;
        blob.y_pos += y;

        // Check if the image needs to be modified
        if (mod) {
          // UYVY
          dest[0] = 127;  // U
          dest[1] = 255;  // Y
          dest[2] = 127;  // V
          dest[3] = 255;  // Y
        }

      } else {

        // Check if the image needs to be modified
        if (mod) {
          // UYVY
          dest[0] = 127;  // U
          dest[1] = 0;    // Y
          dest[2] = 127;  // V
          dest[3] = 0;    // Y
        }
      }

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }

  return blob;
}


#include "modules/computer_vision/blob/blob_finder.h"

/**
 * Blob locator using white filter in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @param[in] threshold Minimum number of pixels that the largest blob should have
 * @return (X,Y) location of the largest blob's centroid
 */

struct results_color locate_blob(struct image_t *input,
                                 int y_m, int y_M,
                                 int u_m, int u_M,
                                 int v_m, int v_M,
                                 int threshold)
{
  // Output
  struct results_color marker;

  // Color Filter
  struct image_filter_t filter[2];
  filter[0].y_min = y_m;
  filter[0].y_max = y_M;
  filter[0].u_min = u_m;
  filter[0].u_max = u_M;
  filter[0].v_min = v_m;
  filter[0].v_max = v_M;

  // Output image
  struct image_t dst;
  image_create(&dst,
               input->w,
               input->h,
               IMAGE_GRADIENT);

  // Labels
  uint16_t labels_count = 512;
  struct image_label_t labels[512];

  // Blob finder
  image_labeling(input, &dst, filter, 1, labels, &labels_count);

  int largest_id = -1;
  int largest_size = 0;

  // Find largest
  for (int i=0; i<labels_count; i++) {
    // Only consider large blobs
    if (labels[i].pixel_cnt > threshold) {
      if (labels[i].pixel_cnt > largest_size) {
        largest_size = labels[i].pixel_cnt;
        largest_id = i;
      }
    }
  }

  if (largest_id >= 0)
  {
    marker.MARKER = TRUE;
    marker.maxx   = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
    marker.maxy   = labels[largest_id].y_sum / labels[largest_id].pixel_cnt;
  } else {
    marker.MARKER = FALSE;
  }

  image_free(&dst);

  return marker;
}


#include "modules/computer_vision/imav2016markers.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/downlink.h"

/**
 * Geo-reference computer vision detection
 * @param[in] *tar Camera frame
 * @param[in] Waypoint to be modified
 * @return marker (X,Y) location of the marker
 */

struct georeference_t {
  struct Int32Vect3 target_i;   ///< Target in pixels, with z being the focal length in pixels, x=up,y=right,out
  struct Int32Vect3 target_l;   ///< Target in meters, relative to the drone in LTP frame
  struct Int32Vect3 x_t;        ///< Target coordinates NED
};

struct georeference_t geo;


struct centroid_t georeference_project(struct camera_frame_t *tar)
{
  // Target direction in camera frame: Zero is looking down in body frames
  // Pixel with x (width) value 0 projects to the left (body-Y-axis)
  // and y = 0 (height) to the top (body-X-axis)
  VECT3_ASSIGN(geo.target_i,
               ((tar->h / 2) - tar->py),
               (tar->px - (tar->w / 2)),
               (tar->f)
              );
  INT32_VECT3_LSHIFT(geo.target_i, geo.target_i, 4)

  // Camera <-> Body
  // Looking down in body frame
  // Bebop has 180deg Z rotation in camera (butt up yields normal webcam)
  struct Int32RMat body_to_cam_rmat;
  INT32_MAT33_ZERO(body_to_cam_rmat);
  MAT33_ELMT(body_to_cam_rmat, 0, 0) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 1, 1) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 2, 2) =  1 << INT32_TRIG_FRAC;

  struct Int32Vect3 target_b;
  int32_rmat_transp_vmult(&target_b, &body_to_cam_rmat, &geo.target_i);

  // Body <-> LTP
  struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&geo.target_l, ltp_to_body_rmat, &target_b);

  // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
  // Divide by z-component to normalize the projection vector
  int32_t zi = geo.target_l.z;
  if (zi <= 0)
  {
    // Output
    struct centroid_t marker;
    marker.x = 0;
    marker.y = 0;

    // Pointing up or horizontal -> no ground projection
    return marker;
  }

  // Multiply with height above ground
  struct NedCoor_i *pos = stateGetPositionNed_i();
  int32_t zb = pos->z;
  geo.target_l.x *= zb;
  geo.target_l.y *= zb;

  // Divide by z-component
  geo.target_l.x /= zi;
  geo.target_l.y /= zi;
  geo.target_l.z = zb;

  // Marker location w.r.t. the body
  struct centroid_t marker;
  marker.x = - geo.target_l.x * 0.0039063;
  marker.y = - geo.target_l.y * 0.0039063;

  return marker;
}


void georeference_init(void)
{
  INT32_VECT3_ZERO(geo.target_i);
  INT32_VECT3_ZERO(geo.target_l);

  VECT3_ASSIGN(geo.x_t, 0, 0, 0);
}