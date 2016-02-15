/*
 * Copyright (c) 2012 Ted Carancho. (AeroQuad)
 * (c) 2012 Gautier Hattenberger
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

#ifndef MEDIAN_H
#define MEDIAN_H

#define MEDIAN_DATASIZE 5

#include "std.h"
#include "math/pprz_algebra_int.h"

struct MedianFilterInt {
  int32_t data[MEDIAN_DATASIZE], sortData[MEDIAN_DATASIZE];
  int8_t dataIndex;
};

inline void init_median_filter(struct MedianFilterInt *filter);
inline int32_t update_median_filter(struct MedianFilterInt *filter, int32_t new_data);
inline int32_t get_median_filter(struct MedianFilterInt *filter);

inline void init_median_filter(struct MedianFilterInt *filter)
{
  int i;
  for (i = 0; i < MEDIAN_DATASIZE; i++) {
    filter->data[i] = 0;
    filter->sortData[i] = 0;
  }
  filter->dataIndex = 0;
}

inline int32_t update_median_filter(struct MedianFilterInt *filter, int32_t new_data)
{
  int temp, i, j; // used to sort array

  // Insert new data into raw data array round robin style
  filter->data[filter->dataIndex] = new_data;
  if (filter->dataIndex < (MEDIAN_DATASIZE - 1)) {
    filter->dataIndex++;
  } else {
    filter->dataIndex = 0;
  }

  // Copy raw data to sort data array
  memcpy(filter->sortData, filter->data, sizeof(filter->data));

  // Insertion Sort
  for (i = 1; i <= (MEDIAN_DATASIZE - 1); i++) {
    temp = filter->sortData[i];
    j = i - 1;
    while (j >= 0 && temp < filter->sortData[j]) {
      filter->sortData[j + 1] = filter->sortData[j];
      j = j - 1;
    }
    filter->sortData[j + 1] = temp;
  }
  return filter->sortData[(MEDIAN_DATASIZE) >> 1]; // return data value in middle of sorted array
}

inline int32_t get_median_filter(struct MedianFilterInt *filter)
{
  return filter->sortData[(MEDIAN_DATASIZE) >> 1];
}

struct MedianFilter3Int {
  struct MedianFilterInt mf[3];
};

#define InitMedianFilterVect3Int(_f) {  \
    for (int i = 0; i < 3; i++) {         \
      init_median_filter(&(_f.mf[i]));    \
    }                                     \
  }

#define InitMedianFilterEulerInt(_f) InitMedianFilterVect3Int(_f)
#define InitMedianFilterRatesInt(_f) InitMedianFilterVect3Int(_f)

#define UpdateMedianFilterVect3Int(_f, _v) {          \
    (_v).x = update_median_filter(&(_f.mf[0]), (_v).x); \
    (_v).y = update_median_filter(&(_f.mf[1]), (_v).y); \
    (_v).z = update_median_filter(&(_f.mf[2]), (_v).z); \
  }

#define UpdateMedianFilterEulerInt(_f, _v) {                  \
    (_v).phi = update_median_filter(&(_f.mf[0]), (_v).phi);     \
    (_v).theta = update_median_filter(&(_f.mf[1]), (_v).theta); \
    (_v).psi = update_median_filter(&(_f.mf[2]), (_v).psi);     \
  }

#define UpdateMedianFilterRatesInt(_f, _v) {          \
    (_v).p = update_median_filter(&(_f.mf[0]), (_v).p); \
    (_v).q = update_median_filter(&(_f.mf[1]), (_v).q); \
    (_v).r = update_median_filter(&(_f.mf[2]), (_v).r); \
  }

#define GetMedianFilterVect3Int(_f, _v) {   \
    (_v).x = get_median_filter(&(_f.mf[0]));  \
    (_v).y = get_median_filter(&(_f.mf[1]));  \
    (_v).z = get_median_filter(&(_f.mf[2]));  \
  }

#define GetMedianFilterEulerInt(_f, _v) {       \
    (_v).phi = get_median_filter(&(_f.mf[0]));    \
    (_v).theta = get_median_filter(&(_f.mf[1]));  \
    (_v).psi = get_median_filter(&(_f.mf[2]));    \
  }

#define GetMedianFilterRatesInt(_f, _v) {   \
    (_v).p = get_median_filter(&(_f.mf[0]));  \
    (_v).q = get_median_filter(&(_f.mf[1]));  \
    (_v).r = get_median_filter(&(_f.mf[2]));  \
  }

#endif
