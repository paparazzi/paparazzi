/*
 * Copyright (C) 2009  Christophe De Wagter
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

/** @file modules/cartography/photogrammetry_calculator.h

Add to airframe file:

@verbatim
  <section name="Photogrammetry" prefix="PHOTOGRAMMETRY_">
    <!-- Camera Parameters -->
    <define name="FOCAL_LENGTH" value="35" unit="mm"/>
    <define name="SENSOR_WIDTH" value="24" unit="mm"/>    <!-- In direction of the plane's wings -->
    <define name="SENSOR_HEIGHT" value="13.5" unit="mm"/> <!-- In direction of the plane's nose -->
    <define name="PIXELS_WIDTH" value="1024" unit=""/>

    <!-- Flight Safety Parameters -->
    <define name="HEIGHT_MIN" value="35" unit="m"/>
    <define name="HEIGHT_MAX" value="500" unit="m"/>
    <define name="RADIUS_MIN" value="70" unit="m"/>
  </section>

  <modules>
    <load name="photogrammetry_calculator.xml" />
  </modules>
@endverbatim

Add to flightplan or airframe file:
@verbatim
    <!-- Photogrammetry Parameters: define these in the flightplan-->
    <define name="OVERLAP" value="0.5" unit="PROCENT"/>
    <define name="SIDELAP" value="0.5" unit="PROCENT"/>
    <define name="RESOLUTION" value="50" unit="mm pixel projection"/>
@endverbatim

Add to flightplan
@verbatim
  <header>
#define PHOTOGRAMMETRY_SWEEP_ANGLE RadOfDeg(53)  // angle in radians from the North
#define PHOTOGRAMMETRY_OVERLAP 50              // 1-99 Procent
#define PHOTOGRAMMETRY_SIDELAP 50              // 1-99 Procent
#define PHOTOGRAMMETRY_RESOLUTION 80             // mm pixel projection size
</header>

    <block group="survey" name="Initialize Poly Survey 56789" strip_button="Survey5678" strip_icon="survey.png">
      <call fun="PhotogrammetryCalculatorPolygonSurvey(WP_5, 5)"/>
      <call fun="PolygonSurvey()"/>
    </block>
    <block group="survey" name="Initialize ADV Poly 1234 Survey" strip_button="SurveyADV" strip_icon="survey.png">
      <call fun="PhotogrammetryCalculatorPolygonSurveyADV(WP_1, 4)"/>
      <call fun="poly_survey_adv()"/>
    </block>
@endverbatim

 */

#ifndef PHOTOGRAMMETRY_CALCULATOR_H
#define PHOTOGRAMMETRY_CALCULATOR_H

#include "std.h"
#include "paparazzi.h"

#include "modules/nav/nav_survey_poly_osam.h"
#include "modules/nav/nav_survey_polygon.h"


// Flightplan Variables
extern float photogrammetry_sweep_angle;
extern int photogrammetry_sidestep;
extern int photogrammetry_triggerstep;
extern int photogrammetry_height;

extern int photogrammetry_height_min;
extern int photogrammetry_height_max;
extern int photogrammetry_radius_min;


// Photogrammetry Goals
extern int photogrammetry_sidelap;
extern int photogrammetry_overlap;
extern int photogrammetry_resolution;

void init_photogrammetry_calculator(void);
void photogrammetry_calculator_update_camera2flightplan(void);
void photogrammetry_calculator_update_flightplan2camera(void);

// Update Flightplan on Camera Change
#define photogrammetry_calculator_UpdateSideLap(X)  {   \
    photogrammetry_sidelap = X;         \
    photogrammetry_calculator_update_camera2flightplan();   \
  }

#define photogrammetry_calculator_UpdateOverLap(X)  {   \
    photogrammetry_overlap = X;         \
    photogrammetry_calculator_update_camera2flightplan();   \
  }

#define photogrammetry_calculator_UpdateResolution(X) {   \
    photogrammetry_resolution = X;        \
    photogrammetry_calculator_update_camera2flightplan();   \
  }

// Update Camera on Flightplan Change
#define photogrammetry_calculator_UpdateHeight(X) {   \
    photogrammetry_height = X;          \
    photogrammetry_calculator_update_flightplan2camera();   \
  }

#define photogrammetry_calculator_UpdateSideStep(X) {   \
    photogrammetry_sidestep = X;          \
    photogrammetry_calculator_update_flightplan2camera();   \
  }

#define photogrammetry_calculator_UpdateTriggerStep(X)  {   \
    photogrammetry_triggerstep = X;       \
    photogrammetry_calculator_update_flightplan2camera();   \
  }


// Flightplan Routine Wrappers
#define PhotogrammetryCalculatorPolygonSurveyOsam(_WP, _COUNT) {        \
    WaypointAlt(_WP) = photogrammetry_height + GROUND_ALT;      \
    int _ang = 90 - DegOfRad(photogrammetry_sweep_angle);       \
    while (_ang > 90) _ang -= 180; while (_ang < -90) _ang += 180;      \
    nav_survey_poly_osam_setup((_WP), (_COUNT), 2*photogrammetry_sidestep, _ang);   \
  }

#define PhotogrammetryCalculatorPolygonSurvey(_WP, _COUNT) {      \
    nav_survey_polygon_setup((_WP), (_COUNT), DegOfRad(photogrammetry_sweep_angle), \
                             photogrammetry_sidestep, photogrammetry_triggerstep,      \
                             photogrammetry_radius_min,  photogrammetry_height + GROUND_ALT);    \
  }

#endif
