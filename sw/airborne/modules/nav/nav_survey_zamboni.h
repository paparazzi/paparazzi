/*
 * Copyright (C) 2013 Jorn Anke, Felix Ruess
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
 * @file modules/nav/nav_survey_zamboni.h
 *
 * Zamboni pattern survey for fixedwings.
 */

#ifndef NAV_SURVEY_ZAMBONI_H
#define NAV_SURVEY_ZAMBONI_H

#include "std.h"
#include "math/pprz_algebra_float.h"

typedef enum {Z_ERR, Z_ENTRY, Z_SEG, Z_TURN1, Z_RET, Z_TURN2} z_survey_stage;

struct ZamboniSurvey {
  /* variables used to store values from the flight plan */
  struct FloatVect2 wp_center;
  struct FloatVect2 wp_dir;
  struct FloatVect2 sweep_width;
  float altitude;

  /** in degrees. Leave turncircles a small angle before the 180deg turns are completed
   * to get a smoother transition to flight-lines
   */
  int pre_leave_angle;
  float flight_angle; ///< in degrees
  float return_angle; ///< in degrees
  int current_laps;
  int total_laps;
  float turnradius1;
  float turnradius2;
  struct FloatVect2 turn_center1;
  struct FloatVect2 turn_center2;
  struct FloatVect2 seg_start;
  struct FloatVect2 seg_end;
  struct FloatVect2 ret_start;
  struct FloatVect2 ret_end;
  /**
   * z_stage starts at ENTRY and than circles trought the other
   * states until to rectangle is completely covered
   * ENTRY : getting in the right position and height for the first flyover
   * SEG   : fly from seg_start to seg_end and take pictures,
   * then calculate navigation points of next flyover
   * TURN1 : do a 180° turn around seg_center1
   * RET   : fly from ret_start to ret_end
   * TURN2 : do a 180° turn around seg_center2
   */
  z_survey_stage stage;
};


extern bool nav_survey_zamboni_setup(uint8_t center_wp, uint8_t dir_wp, float sweep_length, float sweep_spacing,
                                       int sweep_lines, float altitude);
extern bool nav_survey_zamboni_run(void);

#endif //ZAMBONI_SURVEY_H
