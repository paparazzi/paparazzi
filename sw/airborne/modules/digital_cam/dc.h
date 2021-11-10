/*
 * Copyright (C) 2010 The Paparazzi Team
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


/** @file modules/digital_cam/dc.h
 * Standard Digital Camera Control Interface.
 *
 * -Standard IO
 * -I2C Control
 *
 * Usage: (from the flight plan, the settings or any airborne code):
 * - dc_send_command(  )
 * - set the appropriate autoshoot mode (off/time/distance/trigger)
 * - use the module periodic function to set the autorepeat interval
 * - define SENSOR_SYNC_SEND to get the DC_SHOT_MESSAGE on every SHOOT command
 */

#ifndef DC_H
#define DC_H

#include "float.h"
#include "std.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/gps/gps.h"

/** export the number of the last photo */
extern uint16_t dc_photo_nr;

/** number of images taken since the last change of dc_mode */
extern uint16_t dc_gps_count;


/*
 * Variables for PERIODIC mode.
 */
/** AutoShoot photos every X seconds */
extern float dc_autoshoot_period;


/*
 * Variables for DISTANCE mode.
 */
/** AutoShoot photos on distance to last shot in meters */
extern float dc_distance_interval;


/*
 * Variables for SURVEY mode.
 */
/** distance between dc shots in meters */
extern float dc_survey_interval;

/** point of reference for the survey mode */
extern float dc_gps_x, dc_gps_y;

extern float dc_gps_next_dist;


/*
 * Variables for CIRCLE mode.
 */
/** angle a where first image will be taken at a + delta */
extern float dc_circle_start_angle;

/** angle between dc shots in degree */
extern float dc_circle_interval;

extern float dc_circle_max_blocks;
extern float dc_circle_last_block;


/** camera angle */
extern float dc_cam_angle;
extern uint8_t dc_cam_tracing;

/** Generic Set of Digital Camera Commands */
typedef enum {
  DC_GET_STATUS = 0,

  DC_HOLD = 13,
  DC_SHOOT = 32,

  DC_WIDER = 'w',
  DC_TALLER = 't',

  DC_UP = 'u',
  DC_DOWN = 'd',
  DC_CENTER = 'c',
  DC_LEFT = 'l',
  DC_RIGHT = 'r',

  DC_MENU = 'm',
  DC_HOME = 'h',
  DC_PLAY = 'p',

  DC_ON = 'O',
  DC_OFF = 'o',

} dc_command_type;

/** Send Command To Camera */
extern void dc_send_command(uint8_t cmd);

/** Command sending function */
extern void dc_send_command_common(uint8_t cmd);

/** Auotmatic Digital Camera Photo Triggering modes */
typedef enum {
  DC_AUTOSHOOT_STOP = 0,
  DC_AUTOSHOOT_PERIODIC = 1,
  DC_AUTOSHOOT_DISTANCE = 2,
  DC_AUTOSHOOT_EXT_TRIG = 3,
  DC_AUTOSHOOT_SURVEY = 4,
  DC_AUTOSHOOT_CIRCLE = 5
} dc_autoshoot_type;
extern dc_autoshoot_type dc_autoshoot;

/** Send Down the coordinates of where the photo was taken */
void dc_send_shot_position(void);

/* Macro value used to indicate a discardable argument */
#ifndef DC_IGNORE
#define DC_IGNORE FLT_MAX
#endif

/* Default values for buffer control */
#ifndef DC_IMAGE_BUFFER
#define DC_IMAGE_BUFFER 65535
#endif

/******************************************************************
 * FUNCTIONS
 *****************************************************************/

/** initialize settings */
extern void dc_init(void);

/** periodic function */
extern void dc_periodic(void);

/**
 * Sets the dc control in distance mode.
 *
 * Shoot the next pic if distance to last saved shot position
 * is greater than @a interval.
 *
 * The first picture is taken at after @a interval.
 *
 * @param interval minimum distance between shots in m
 * @return zero
 */
extern uint8_t dc_distance(float interval);

/**
 * Sets the dc control in circle mode.
 *
 * In this mode the dc control assumes a perfect circular
 * course.
 *
 * The 'start' value is the reference course and 'interval'
 * the minimum angle between shots.
 * If 'start' is DC_IGNORE the current course is used instead.
 *
 * The first picture is taken at angle start+interval.
 *
 * @param interval minimum angle between shots in deg
 * @param start reference course in deg (or special DC_IGNORE)
 * @return zero
 */
extern uint8_t dc_circle(float interval, float start);

#define dc_Circle(interval) dc_circle(interval, DC_IGNORE)

/**
 * Sets the dc control in distance mode.
 *
 * In this mode, the dc control assumes a perfect
 * line formed course since the distance is calculated
 * relative to the first given point of reference.
 * So not usable for circles or other comparable
 * shapes.
 *
 * The values of 'x' and 'y' are the coordinates
 * of the reference point used for the distance
 * calculations.
 * If 'y' is DC_IGNORE the value of 'x' is interpreted
 * as index of a waypoint declared in the flight plan.
 * If both 'x' and 'y' are DC_IGNORE the current position
 * will be used as reference point.
 *
 * @param interval distance between shots in meters
 * @param x x coordinate of reference point (or special DC_IGNORE)
 * @param y y coordinate of reference point (or special DC_IGNORE)
 * @return zero
 */
extern uint8_t dc_survey(float interval, float x, float y);

#define dc_Survey(interval) dc_survey(interval, DC_IGNORE, DC_IGNORE)


/**
 * Stop dc control.
 * Sets the dc control in inactive mode,
 * stopping all current actions.
 */
extern uint8_t dc_stop(void);

#define dc_Stop(_) dc_stop()

/**
 * Send an info message.
 * Containing information about position,
 * course, buffer and all other internal
 * variables used by the dc control.
 */
extern uint8_t dc_info(void);


#endif // DC_H
