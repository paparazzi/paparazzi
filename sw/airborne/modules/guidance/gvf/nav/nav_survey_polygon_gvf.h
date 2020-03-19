/*
 * Copyright (C) 2017  The Paparazzi Team
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
 */

/**
 * @file modules/guidance/gvf/nav_survey_polygon_gvf.h
 *
 * Advanced polygon survey for fixedwings from Uni Stuttgart
 * adapted to be employed with the Guidance Vector Field
 *
 */

#ifndef NAV_SURVEY_POLYGON_GVF_H
#define NAV_SURVEY_POLYGON_GVF_H

#include "std.h"
#include "math/pprz_algebra_float.h"

/*
  SurveyStage starts at ENTRY and than circles trought the other
  states until to polygon is completely covered
  ENTRY : getting in the right position and height for the first flyover
  SEG   : fly from seg_start to seg_end and take pictures,
  then calculate navigation points of next flyover
  TURN1 : do a 180° turn around seg_center1
  RET   : fly from ret_start to ret_end
  TURN2 : do a 180° turn around seg_center2
*/
enum gvf_SurveyStage {gERR, gENTRY, gSEG, gTURN1, gRET, gTURN2};

struct gvf_SurveyPolyAdv {
  /*
  The following variables are set by nav_survey_polygon_start and not changed later on
  */

  // precomputed vectors to ease calculations
  struct FloatVect2 dir_vec;
  struct FloatVect2 sweep_vec;
  struct FloatVect2 rad_vec;

  //the polygon from the flightplan
  uint8_t poly_first;
  uint8_t poly_count;

  //desired properties of the flyover
  float psa_min_rad;
  float psa_sweep_width;
  float psa_shot_dist;
  float psa_altitude;

  //direction for the flyover (0° == N)
  int segment_angle;
  int return_angle;

  /*
     The Following variables are dynamic, changed while navigating.
  */
  enum gvf_SurveyStage stage;
  // points for navigation
  struct FloatVect2 seg_start;
  struct FloatVect2 seg_end;
  struct FloatVect2 seg_center1;
  struct FloatVect2 seg_center2;
  struct FloatVect2 entry_center;
  struct FloatVect2 ret_start;
  struct FloatVect2 ret_end;
};

// external setting
extern float gvf_nav_survey_sweep;

extern void gvf_nav_survey_polygon_setup(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist,
                                     float min_rad, float altitude);

void gvf_nav_direction_circle(float rad);
extern bool gvf_nav_survey_polygon_run(void);

#endif
