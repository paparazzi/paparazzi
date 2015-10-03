/*
 * Copyright (C) 2015
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/blob/blob_finder.c
 *
 * Parse UYVY images and make a list of blobs of connected pixels
 */



#include "blob_finder.h"
#include <stdio.h>

void image_labeling(struct image_t *input, struct image_t *output, struct image_filter_t *filters, uint8_t filters_cnt,
                    struct image_label_t *labels, uint16_t *labels_count)
{
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint16_t *output_buf = (uint16_t *)output->buf;

  // Initialize labels
  uint16_t labels_size = *labels_count;
  uint16_t labels_cnt = 0;
  uint16_t i, x, y;

  // Initialize first line with empty groups
  uint16_t *p = output_buf;
  for (i = 0; i < output->w; i++) {
    *p++ = 0xffff;
  }

  // Do steps of 2 for YUV image
  // Skip first line as we need previous groups for connectivity
  for (y = 1; y < input->h; y++) {
    for (x = 0; x < input->w / 2; x++) {
      uint16_t lid = 0;
      uint8_t p_y = (input_buf[y * input->w * 2 + x * 4 + 1] + input_buf[y * input->w * 2 + x * 4 + 3]) / 2;
      uint8_t p_u = input_buf[y * input->w * 2 + x * 4];
      uint8_t p_v = input_buf[y * input->w * 2 + x * 4 + 2];

      // Go trough the filters
      uint8_t f = 0;
      for (; f < filters_cnt; f++) {
        if (p_y > filters[f].y_min && p_y < filters[f].y_max &&
            p_u > filters[f].u_min && p_u < filters[f].u_max &&
            p_v > filters[f].v_min && p_v < filters[f].v_max) {
          break;
        }
      }

      // Check if this pixel belongs to a filter else goto next
      if (f >= filters_cnt) {
        output_buf[y * output->w + x] = 0xFFFF;
        continue;
      }

      // Check pixel above (if the same filter then take same group)
      lid = output_buf[(y - 1) * output->w + x];
      if (y > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Check pixel top right (check for merging)
      lid = output_buf[(y - 1) * output->w + x + 1];
      if (y > 0 && x < output->w - 1 && lid < labels_size && labels[lid].filter == f) {

        // Merging labels if needed
        uint16_t lid_tl = output_buf[(y - 1) * output->w + x - 1]; // Top left
        uint16_t lid_l = output_buf[y * output->w + x - 1]; // Left
        uint16_t m = labels[lid].id, n = labels[lid].id;
        if (x > 0 && lid_tl < labels_size && labels[lid_tl].filter == f) {
          // Merge with top left
          m = labels[lid].id;
          n = labels[lid_tl].id;
        } else if (x > 0 && lid_l < labels_size && labels[lid_l].filter == f) {
          // Merge with left
          m = labels[lid].id;
          n = labels[lid_l].id;
        }

        // Change the id of the highest id label
        if (m != n) {
          if (m > n) {
            m = n;
            n = labels[lid].id;
          }

          for (i = 0; i < labels_cnt; i++) {
            if (labels[i].id == n) {
              labels[i].id = m;
            }
          }
        }

        // Update the label
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Take top left
      lid = output_buf[(y - 1) * output->w + x - 1];
      if (y > 0 && x > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Take left
      lid = output_buf[y * output->w + x - 1];
      if (x > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Check if there is enough space
      if (labels_cnt >= labels_size - 1) {
        break;
      }

      // Create new group
      lid = labels_cnt;
      output_buf[y * output->w + x] = lid;
      labels[lid].id = lid;
      labels[lid].filter = f;
      labels[lid].pixel_cnt = 1;
      labels[lid].x_min = x;
      labels[lid].y_min = y;
      labels[lid].x_sum = x;
      labels[lid].y_sum = y;
      labels_cnt++;
    }
  }

  if (labels_cnt >= labels_size - 1) {
    printf("Break did not work: we have %d labels\n", labels_cnt);
  }

  // Merge connected labels
  for (i = 0; i < labels_cnt; i++) {
    if (labels[i].id != i) {
      uint16_t new_id = labels[i].id;
      labels[new_id].pixel_cnt += labels[i].pixel_cnt;
      labels[new_id].x_sum += labels[i].x_sum;
      labels[new_id].y_sum += labels[i].y_sum;

      //printf("%d == %d,  ",new_id, i);

      if (labels[i].x_min < labels[new_id].x_min) { labels[new_id].x_min = labels[i].x_min; }
      if (labels[i].y_min < labels[new_id].y_min) { labels[new_id].y_min = labels[i].y_min; }
    }
  }

  *labels_count = labels_cnt;

  // Replace ID's
  for (y = 0; y < input->h; y++) {
    for (x = 0; x < input->w / 2; x++) {
      uint16_t lid = output_buf[y * output->w + x];
      if (lid < labels_cnt) {
        output_buf[y * output->w + x] = labels[lid].id;
      }
    }
  }
}
