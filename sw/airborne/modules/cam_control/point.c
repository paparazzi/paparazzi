/*
 * Copyright (C) 2005-2008  Arnold Schroeter
 * Modified and expanded to show the coordinates of where the camera is looking at
 * by Chris Efstathiou 23-Jan-2011 AD.
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

/** \file point.c
 *  \brief Determines camera pan and tilt angles.
 *
 * project:        Paparazzi
 * description:    Determines camera pan and tilt angles from
 *                 plane's and object's positions and plane's
 *                 pitch and roll angles. Software might be optimized
 *                 by removing multiplications with 0, it is left this
 *                 way for better understandabilty and changeability.
 *
 * author:         Arnold Schroeter, Martin Mueller
 *
 * hardware:
 *
 * The camera control is made of normal servos. Usually servos have a
 * turn angle of about 90°. This is changed electrically so that they
 * can do a 180°. It is achieved by adding two serial resistors at both
 * sides of the potentiometer (P1), one for increasing the usable angle
 * (R1) and the other for moving the middle position to a useful angle
 * (R2). Therefore a servo with a 270° potentiometer is needed. Very
 * small and light servos have 180° potentiometers, these do not allow
 * a 180° degrees sweep. Cut the outer two connections between the
 * potentiometer and the board to insert the resistors. The values for
 * R1 and R2 should be found out by testing as there might be serial
 * resistors on the servo board that affect the values. Start with
 * about 1/2 the value of P1 for R1 and change R1 until you get a
 * little more than 180° sweep. Then insert and modify R2 to set
 * neutral back to the middle position of the potentiometer.
 *
 *
 *                     ^
 *                    /
 *              ----------
 *      *-------I   /    I-------*
 *      I       ----------       I
 *      I         /     P1       I
 *      I         I              I
 *      I         I              I
 *     ---        I             ---
 *     I I        I             I I
 *     I I        I             I I
 *     I I        I             I I
 *     --- R1     I             --- R2
 *      I         I              I
 *      I         I              I
 *
 *
 */

#include <math.h>
#include "cam.h"
#include "point.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/navigation/common_nav.h"
#include "subsystems/gps.h"
#include "math/pprz_geodetic_float.h"

typedef struct {
  float fx;
  float fy;
  float fz;
} VECTOR;

typedef struct {
  float fx1; float fx2; float fx3;
  float fy1; float fy2; float fy3;
  float fz1; float fz2; float fz3;
} MATRIX;

float cam_theta;
float cam_phi;
bool_t heading_positive = 0;
float  memory_x, memory_y, memory_z;
#if defined(SHOW_CAM_COORDINATES)
float   cam_point_x;
float   cam_point_y;
uint16_t cam_point_distance_from_home;
float cam_point_lon,  cam_point_lat;
float distance_correction = 1;
#endif

void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC);
void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC);

/*******************************************************************
; function name:   vSubtractVectors
; description:     subtracts two vectors a = b - c
; parameters:
;*******************************************************************/
void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC)
{
  svA->fx = svB.fx - svC.fx;
  svA->fy = svB.fy - svC.fy;
  svA->fz = svB.fz - svC.fz;
}

/*******************************************************************
; function name:   vMultiplyMatrixByVector
; description:     multiplies matrix by vector svA = smB * svC
; parameters:
;*******************************************************************/
void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}

/*******************************************************************
; function name:   vPoint
; description:     Transforms ground coordinate system into
;                  plane's coordinate system via three rotations
;                  and determines positions of camera servos.
; parameters:      fPlaneNorth, fPlaneEast, fPlaneAltitude  plane's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same as for the object's
;                           position)
;                  fRollAngle  level=0; right wing down = positive values
;                  fPitchAngle level=0; nose up = positive values
;                           plane's pitch and roll angles
;                           with respect to ground in radians
;                  fYawAngle   north=0; right= positive values in radians
;                           plane's yaw angle with respect to north
;                  fObjectNorth, fObjectEast, fAltitude object's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same for the plane's
;                           position)
;                  fPan, fTilt angles for camera servos in radians,
;                           pan is turn/left-right and tilt is down-up
;                           in reference to the camera picture
; camera mount:    The way the camera is mounted is given through a
;                  define POINT_CAM_a_[_b] where a gives the mount
;                  angle within the aircraft and b the angle when
;                  viewing the direction of the first servo.
;*******************************************************************/
void vPoint(float fPlaneEast, float fPlaneNorth, float fPlaneAltitude,
            float fRollAngle, float fPitchAngle, float fYawAngle,
            float fObjectEast, float fObjectNorth, float fAltitude,
            float *fPan, float *fTilt)
{
  static VECTOR svPlanePosition,
         svObjectPosition,
         svObjectPositionForPlane,
         svObjectPositionForPlane2;

  static VECTOR sv_cam_projection,
         sv_cam_projection_buf;

  static MATRIX smRotation;

  /***        BELOW IS THE CODE THAT READS THE RC PAN AND TILT CHANNELS AND CONVERTS THEM TO ANGLES (RADIANS)   ***/
  /***        IT IS USED FOR CALCULATING THE COORDINATES OF THE POINT WHERE THE CAMERA IS LOOKING AT            ***/
  /***        THIS IS DONE ONLY FOR THE CAM_MODE_STABILIZED OR CAM_MODE_RC.                                     ***/
  /***        IN OTHER MODES ONLY THE CAM_POINT WAYPOINT AND THE DISTANCE FROM TARGET IS UPDATED                ***/
  if (cam_mode == CAM_MODE_STABILIZED || cam_mode == CAM_MODE_RC || cam_mode == CAM_MODE_WP_TARGET
      || cam_mode == CAM_MODE_XY_TARGET) {

    if (cam_mode == CAM_MODE_STABILIZED || cam_mode == CAM_MODE_RC) {

      /*########################################  TILT CONTROL INPUT  #############################################*/
#ifdef CAM_TILT_NEUTRAL

#if defined(RADIO_TILT)
      if ((*fbw_state).channels[RADIO_TILT] >= 0) {
        cam_theta = (float)RadOfDeg(CAM_TILT_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_TILT] / (float)MAX_PPRZ) *
                    (float)(RadOfDeg(CAM_TILT_MAX - CAM_TILT_NEUTRAL)));

      } else {
        cam_theta = (float)RadOfDeg(CAM_TILT_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_TILT] / (float)MIN_PPRZ) *
                    (float)(RadOfDeg(CAM_TILT_MIN - CAM_TILT_NEUTRAL)));
      }
#elif defined(RADIO_PITCH)
      if ((*fbw_state).channels[RADIO_PITCH] >= 0) {
        cam_theta = (float)RadOfDeg(CAM_TILT_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_PITCH] / (float)MAX_PPRZ) *
                    (float)(RadOfDeg(CAM_TILT_MAX - CAM_TILT_NEUTRAL)));

      } else {
        cam_theta = (float)RadOfDeg(CAM_TILT_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_PITCH] / (float)MIN_PPRZ) *
                    (float)(RadOfDeg(CAM_TILT_MIN - CAM_TILT_NEUTRAL)));
      }
#else
#error RADIO_TILT or RADIO_PITCH not defined.
#endif

#else   //#ifdef CAM_TILT_NEUTRAL

#if defined(RADIO_TILT)
      cam_theta = RadOfDeg(CAM_TILT_MIN) + (RadOfDeg(CAM_TILT_MAX - CAM_TILT_MIN) * ((float)(
                                              *fbw_state).channels[RADIO_TILT] / (float)MAX_PPRZ));
#elif defined(RADIO_PITCH)
      cam_theta = RadOfDeg(CAM_TILT_MIN) + (RadOfDeg(CAM_TILT_MAX - CAM_TILT_MIN) * ((float)(
                                              *fbw_state).channels[RADIO_PITCH] / (float)MAX_PPRZ));
#else
#error RADIO_TILT or RADIO_PITCH not defined.
#endif

#endif   //#ifdef CAM_TILT_NEUTRAL
      /*########################################  END OF TILT CONTROL INPUT  ########################################*/

      /*###########################################  PAN CONTROL INPUT  #############################################*/
#ifdef CAM_PAN_NEUTRAL

#if defined(RADIO_PAN)
      if ((*fbw_state).channels[RADIO_PAN] >= 0) {
        cam_phi = RadOfDeg(CAM_PAN_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_PAN] / (float)MAX_PPRZ) *
                                               RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL));

      } else {
        cam_phi = RadOfDeg(CAM_PAN_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_PAN] / (float)MIN_PPRZ) *
                                               RadOfDeg(CAM_PAN_MIN - CAM_PAN_NEUTRAL));
      }
#elif defined(RADIO_ROLL)
      if ((*fbw_state).channels[RADIO_ROLL] >= 0) {
        cam_phi = RadOfDeg(CAM_PAN_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_ROLL] / (float)MAX_PPRZ) *
                                               RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL));

      } else {
        cam_phi = RadOfDeg(CAM_PAN_NEUTRAL) + (((float)(*fbw_state).channels[RADIO_ROLL] / (float)MIN_PPRZ) *
                                               RadOfDeg(CAM_PAN_MIN - CAM_PAN_NEUTRAL));
      }
#else
#error RADIO_PAN or RADIO_ROLL not defined.
#endif

#else   //#ifdef CAM_PAN_NEUTRAL

#if defined(RADIO_PAN)
      cam_phi = RadOfDeg(CAM_PAN_MIN) + (RadOfDeg(CAM_PAN_MAX - CAM_PAN_MIN) * ((float)(*fbw_state).channels[RADIO_PAN] /
                                         (float)MAX_PPRZ));
#elif defined(RADIO_ROLL)
      cam_phi = RadOfDeg(CAM_PAN_MIN) + (RadOfDeg(CAM_PAN_MAX - CAM_PAN_MIN) * ((float)(*fbw_state).channels[RADIO_ROLL] /
                                         (float)MAX_PPRZ));
#else
#error RADIO_PAN or RADIO_ROLL not defined.
#endif

#endif  //#ifdef CAM_PAN_NEUTRAL
      /*########################################  END OF PAN CONTROL INPUT  #############################################*/

      // Bound Pan and Tilt angles.
      if (cam_theta > RadOfDeg(CAM_TILT_MAX)) {
        cam_theta = RadOfDeg(CAM_TILT_MAX);

      } else if (cam_theta < RadOfDeg(CAM_TILT_MIN)) { cam_theta = RadOfDeg(CAM_TILT_MIN); }

      if (cam_phi > RadOfDeg(CAM_PAN_MAX)) {
        cam_phi = RadOfDeg(CAM_PAN_MAX);

      } else if (cam_phi < RadOfDeg(CAM_PAN_MIN)) { cam_phi = RadOfDeg(CAM_PAN_MIN); }


      svPlanePosition.fx = fPlaneEast;
      svPlanePosition.fy = fPlaneNorth;
      svPlanePosition.fz = fPlaneAltitude;

// FOR TESTING ANGLES IN THE SIMULATOR ONLY CODE UNCOMMENT THE TWO BELOW LINES
//cam_phi = RadOfDeg(90); // LOOK 45 DEGREES TO THE LEFT, -X IS TO THE LEFT AND +X IS TO THE RIGHT
//cam_theta = RadOfDeg(70);     //  LOOK 45 DEGREES DOWN, 0 IS STRAIGHT DOWN 90 IS STRAIGHT IN FRONT

      if (cam_theta > RadOfDeg(80) && cam_mode == CAM_MODE_RC) { // Not much to see after 80 degrees of tilt so stop tracking.
        *fPan = cam_phi;
        *fTilt = cam_theta;
#ifdef SHOW_CAM_COORDINATES
        cam_point_distance_from_home = 0;
        cam_point_lon = 0;
        cam_point_lat = 0;
#endif
        return;

      } else {
        sv_cam_projection_buf.fx = svPlanePosition.fx + (tanf(cam_theta) * (fPlaneAltitude - ground_alt));
        sv_cam_projection_buf.fy = svPlanePosition.fy;
      }

#if defined(WP_CAM_POINT)
      sv_cam_projection_buf.fz = waypoints[WP_CAM_POINT].a;
#else
      sv_cam_projection_buf.fz = ground_alt;
#endif

      /* distance between plane and camera projection */
      vSubtractVectors(&sv_cam_projection, sv_cam_projection_buf, svPlanePosition);

      float heading_radians = RadOfDeg(90) - fYawAngle; //Convert the gps heading (radians) to standard mathematical notation.
      if (heading_radians > RadOfDeg(180)) { heading_radians -= RadOfDeg(360); }
      if (heading_radians < RadOfDeg(-180)) { heading_radians += RadOfDeg(360); }
      //heading_radians += cam_theta;

      /* camera pan angle correction, using a clockwise rotation */
      smRotation.fx1 = (float)(cos(cam_phi));
      smRotation.fx2 = (float)(sin(cam_phi));
      smRotation.fx3 = 0.;
      smRotation.fy1 = -smRotation.fx2;
      smRotation.fy2 = smRotation.fx1;
      smRotation.fy3 = 0.;
      smRotation.fz1 = 0.;
      smRotation.fz2 = 0.;
      smRotation.fz3 = 1.;

      vMultiplyMatrixByVector(&sv_cam_projection_buf, smRotation, sv_cam_projection);

      /* yaw correction using a counter clockwise rotation*/
      smRotation.fx1 = (float)(cos(heading_radians));
      smRotation.fx2 = -(float)(sin(heading_radians));
      smRotation.fx3 = 0.;
      smRotation.fy1 = -smRotation.fx2;
      smRotation.fy2 = smRotation.fx1;
      smRotation.fy3 = 0.;
      smRotation.fz1 = 0.;
      smRotation.fz2 = 0.;
      smRotation.fz3 = 1.;

      vMultiplyMatrixByVector(&sv_cam_projection, smRotation, sv_cam_projection_buf);

#if defined(RADIO_CAM_LOCK)
      if ((float)(*fbw_state).channels[RADIO_CAM_LOCK] > MAX_PPRZ / 2)) && pprz_mode == PPRZ_MODE_AUTO2) { cam_lock = TRUE; }
      if ((float)(*fbw_state).channels[RADIO_CAM_LOCK] < MIN_PPRZ / 2 && pprz_mode == PPRZ_MODE_AUTO2) { cam_lock = FALSE; }
#endif
    // When the variable "cam_lock" is set then the last calculated position is set as the target waypoint.
    if (cam_lock == FALSE) {
      fObjectEast = (fPlaneEast + sv_cam_projection.fx) ;
        fObjectNorth = (fPlaneNorth + sv_cam_projection.fy) ;
        fAltitude = ground_alt;
        memory_x = fObjectEast;
        memory_y = fObjectNorth;
        memory_z = fAltitude;
#if defined(WP_CAM_POINT)
        waypoints[WP_CAM_POINT].x = fObjectEast;
        waypoints[WP_CAM_POINT].y = fObjectNorth;
        waypoints[WP_CAM_POINT].a = ground_alt;
#endif
#if defined(SHOW_CAM_COORDINATES)
        cam_point_x = fObjectEast;
        cam_point_y = fObjectNorth;

        cam_point_distance_from_home = distance_correction * (uint16_t)(sqrt((cam_point_x * cam_point_x) +
                                       (cam_point_y * cam_point_y)));

        struct UtmCoor_f utm;
        utm.east = gps.utm_pos.east / 100. + sv_cam_projection.fx;
        utm.north = gps.utm_pos.north / 100. + sv_cam_projection.fy;
        utm.zone = gps.utm_pos.zone;
        struct LlaCoor_f lla;
        lla_of_utm_f(&lla, &utm);
        cam_point_lon = lla.lon * (180 / M_PI);
        cam_point_lat = lla.lat * (180 / M_PI);
#endif

      } else {
        fObjectEast = memory_x;
        fObjectNorth = memory_y;
        fAltitude = memory_z;
#if defined(WP_CAM_POINT)
        waypoints[WP_CAM_POINT].x = fObjectEast;
        waypoints[WP_CAM_POINT].y = fObjectNorth;
        waypoints[WP_CAM_POINT].a = fAltitude;
#endif
      }

      if (cam_mode == CAM_MODE_RC && cam_lock == 0) {
      *fPan = cam_phi;
      *fTilt = cam_theta;
      return;
    }

#if defined(WP_CAM_POINT)
    else {
      waypoints[WP_CAM_POINT].x = fObjectEast;
        waypoints[WP_CAM_POINT].y = fObjectNorth;
        waypoints[WP_CAM_POINT].a = fAltitude;
      }
#endif
      /***    END OF THE CODE THAT CALCULATES THE COORDINATES OF WHERE THE CAMERA IS LOOKING AT     ***/
    } else {
      /***    THE BELOW CODE IS ONLY EXECUTED IN CAM_MODE_WP_TARGET OR CAM_MODE_XY_TARGET           ***/
#ifdef SHOW_CAM_COORDINATES
      cam_point_distance_from_home = distance_correction * (uint16_t) fabs(((uint16_t)(sqrt((fObjectNorth * fObjectNorth) +
                                     (fObjectEast * fObjectEast)))) -
                                     ((uint16_t)(sqrt((fPlaneNorth * fPlaneNorth) + (fPlaneEast * fPlaneEast)))));

      struct UtmCoor_f utm;
      utm.east = nav_utm_east0 + fObjectEast;
      utm.north = nav_utm_north0 + fObjectNorth;
      utm.zone = gps.utm_pos.zone;
      struct LlaCoor_f lla;
      lla_of_utm_f(&lla, &utm);
      cam_point_lon = lla.lon * (180 / M_PI);
      cam_point_lat = lla.lat * (180 / M_PI);
#endif


#if defined(WP_CAM_POINT)
      waypoints[WP_CAM_POINT].x = fObjectEast;
      waypoints[WP_CAM_POINT].y = fObjectNorth;
      waypoints[WP_CAM_POINT].a = fAltitude;
#endif

    }

  }

//************************************************************************************************
//************************************************************************************************
//************************************************************************************************
//************************************************************************************************

  /*
  By swapping coordinates (fx=fPlaneNorth, fy=fPlaneEast) we make the the circle angle go from 0 (0 is to the top of the circle)
  to 360 degrees or from 0 radians to 2 PI radians in a clockwise rotation. This way the GPS reported angle can be directly
  applied to the rotation matrices (in radians).
  In standard mathematical notation 0 is to the right (East) of the circle, -90 is to the bottom, +-180 is to the left
  and +90 is to the top (counterclockwise rotation).
  When reading back the actual rotated coordinates sv_cam_projection.fx has the y coordinate and sv_cam_projection.fy has the x
  represented on a circle in standard mathematical notation.
  */
  svPlanePosition.fx = fPlaneNorth;
  svPlanePosition.fy = fPlaneEast;
  svPlanePosition.fz = fPlaneAltitude;

  svObjectPosition.fx = fObjectNorth;
  svObjectPosition.fy = fObjectEast;
  svObjectPosition.fz = fAltitude;

  /* distance between plane and object */
  vSubtractVectors(&svObjectPositionForPlane, svObjectPosition, svPlanePosition);

  /* yaw */
  smRotation.fx1 = (float)(cos(fYawAngle));
  smRotation.fx2 = (float)(sin(fYawAngle));
  smRotation.fx3 = 0.;
  smRotation.fy1 = -smRotation.fx2;
  smRotation.fy2 = smRotation.fx1;
  smRotation.fy3 = 0.;
  smRotation.fz1 = 0.;
  smRotation.fz2 = 0.;
  smRotation.fz3 = 1.;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);

  /* pitch */
  smRotation.fx1 = (float)(cos(fPitchAngle));
  smRotation.fx2 = 0.;
  smRotation.fx3 = (float)(sin(fPitchAngle));
  smRotation.fy1 = 0.;
  smRotation.fy2 = 1.;
  smRotation.fy3 = 0.;
  smRotation.fz1 = -smRotation.fx3;
  smRotation.fz2 = 0.;
  smRotation.fz3 = smRotation.fx1;

  vMultiplyMatrixByVector(&svObjectPositionForPlane, smRotation, svObjectPositionForPlane2);

  /* roll */
  smRotation.fx1 = 1.;
  smRotation.fx2 = 0.;
  smRotation.fx3 = 0.;
  smRotation.fy1 = 0.;
  smRotation.fy2 = (float)(cos(fRollAngle));
  smRotation.fy3 = (float)(-sin(fRollAngle));
  smRotation.fz1 = 0.;
  smRotation.fz2 = -smRotation.fy3;
  smRotation.fz3 = smRotation.fy2;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);

#ifdef POINT_CAM_PITCH

  /*
   * This is for one axis pitch camera mechanisms. The pitch servo neutral
   * makes the camera look down, 90° is to the front and -90° is to the
   * back. The pitch value is given through the tilt parameter.
   * The camera picture is upright when looking in flight direction.
   *
   * tilt servo, looking from left:
   *
   *     plane front <-------------- plane back
   *                      / I \
   *                     /  I  \
   *                   45°  I  -45°
   *                        0°
   *
   * (should be hyperbolic, we use lines to make it better, the plane rolls
   *  away from the object while flying towards it!)
   *
   */

  /* fTilt =   0 -> camera looks down
              90 -> camera looks forward
             -90 -> camera looks backward
  */
#if 0 //we roll away anyways
  *fTilt = (float)(atan2(svObjectPositionForPlane2.fx,
                         sqrt(svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy
                              + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz)
                        ));
#else
  *fTilt = (float)(atan2(svObjectPositionForPlane2.fx, -svObjectPositionForPlane2.fz));
#endif

  /* fPan is deactivated
  */
  *fPan = 0;
#else
#ifdef POINT_CAM_ROLL

  /*
   * This is for single axis roll camera mechanisms. The tilt servo neutral
   * makes the camera look down, -90° is to the right and 90° is to the
   * left.
   * The camera picture is upright when looking to the right.
   *
   *
   * tilt servo, looking from behind:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                  45°  I  -45°
   *                       0°
   *
   */
#if 1  // have to check if it helps
  *fTilt = (float)(atan2(svObjectPositionForPlane2.fy,
                         sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                              + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz)
                        ));
#else
  *fTilt = (float)(atan2(svObjectPositionForPlane2.fy, -svObjectPositionForPlane2.fz));
#endif

  /* fPan is deactivated
  */
  *fPan = 0;
#else
#ifdef POINT_CAM_YAW_PITCH_NOSE

  /*
                   -45   0   45
                      \  |  /
                       \ | /
                        \|/
                      ##
                      ##
           _____________##______________
  left tip|_____________________________|right wing tip
                      ##
                      ##
                      ##
                      ##
                  ______##______
                 |_____|_|_____|
                        |
  */

#if defined(CAM_PAN_MODE) && CAM_PAN_MODE == 360
  /* fixed to the plane*/
  *fPan = (float)(atan2(svObjectPositionForPlane2.fy, (svObjectPositionForPlane2.fx)));

  *fTilt = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                              + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                         -svObjectPositionForPlane2.fz
                        ));

  // I need to avoid oscillations around the 180 degree mark.
  /*
     if (*fPan > 0 && *fPan <= RadOfDeg(175)){ heading_positive = 1; }
     if (*fPan < 0 && *fPan >= RadOfDeg(-175)){ heading_positive = 0; }

     if (*fPan > RadOfDeg(175) && heading_positive == 0){
        *fPan = RadOfDeg(-180);

     }else if (*fPan < RadOfDeg(-175) && heading_positive){
              *fPan = RadOfDeg(180);
              heading_positive = 0;
           }
  */
#else
  *fPan = (float)(atan2(svObjectPositionForPlane2.fy, fabs(svObjectPositionForPlane2.fx)));

  *fTilt = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                              + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                         -svObjectPositionForPlane2.fz
                        ));

  if (svObjectPositionForPlane2.fx < 0) {
    *fPan = -*fPan;
    *fTilt = -*fTilt;
  }

  // I need to avoid oscillations around the 180 degree mark.
  /*
     if (*fPan > 0 && *fPan <= RadOfDeg(85)){ heading_positive = 1; }
     if (*fPan < 0 && *fPan >= RadOfDeg(-85)){ heading_positive = 0; }

     if (*fPan > RadOfDeg(85) && heading_positive == 0){
        *fPan = RadOfDeg(-90);

     }else if (*fPan < RadOfDeg(-85) && heading_positive){
              *fPan = RadOfDeg(90);
              heading_positive = 0;
           }
  */
#endif

#else
#ifdef POINT_CAM_YAW_PITCH

  /*
   * This is for two axes pan/tilt camera mechanisms. The default is to
   * circle clockwise so view is right. The pan servo neutral makes
   * the camera look to the right with 0° given, 90° is to the back and
   * -90° is to the front. The tilt servo neutral makes the camera look
   * down with 0° given, 90° is to the right and -90° is to the left (all
   * values are used in radian in the software). If the camera looks to
   * the right side of the plane, the picture is upright. It is upside
   * down when looking to the left. That is corrected with the MPEG
   * decoding software on the laptop by mirroring. The pan servo is fixed
   * in the plane and the tilt servo is moved by the pan servo and moves
   * the camera.
   *
   *
   * pan servo, tilt set to 90°, looking from top:
   *
   *   plane front
   *
   *       ^
   *       I
   *       I  45°
   *       I /
   *       I/
   *       I------- 0°
   *       I\
   *       I \
   *       I  -45°
   *       I
   *
   *   plane back
   *
   *
   * tilt servo, pan set to 0°, looking from back:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                 -45°  I   45°
   *                       0°
   *
   */

  /* fPan =   0  -> camera looks along the wing
             90  -> camera looks in flight direction
            -90  -> camera looks backwards
  */
  /* fixed to the plane*/
  *fPan = (float)(atan2(svObjectPositionForPlane2.fx,
                        fabs(svObjectPositionForPlane2.fy)));// Or is it the opposite??? (CEF)
  // (CEF) It turned out that Object_North is loaded with x and Object_East with y (reverse). See line #155
  // this means that:
  // *fPan = (float)(atan2(svObjectPositionForPlane2.fy, svObjectPositionForPlane2.fy)); // makes the camera 0 to look to the nose?

  /* fTilt =   0  -> camera looks down
              90  -> camera looks into right hemisphere
             -90  -> camera looks into left hemispere
     actually the camera always looks more or less downwards, but never upwards
  */
  *fTilt = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                              + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                         -svObjectPositionForPlane2.fz
                        ));
  if (svObjectPositionForPlane2.fy < 0) {
    *fPan = -*fPan;
    *fTilt = -*fTilt;
  }

#else
#ifdef POINT_CAM_PITCH_ROLL

  /*
   * This is for another two axes camera mechanisms. The tilt servo is fixed to
   * the fuselage and moves the pan servo.
   *
   * tilt servo, looking from left:
   *
   *    plane front <--------------- plane back
   *                      / I \
   *                     /  I  \
   *                   45°  I  -45°
   *                        0°
   *
   *
   * pan servo, looking from back:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                  45°  I  -45°
   *                       0°
   *
   */

  *fTilt = (float)(atan2(svObjectPositionForPlane2.fx, -svObjectPositionForPlane2.fz));

  *fPan  = (float)(atan2(-svObjectPositionForPlane2.fy,
                         sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                              + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz)
                        ));

#else
#error at least one POINT_CAM_* camera mount has to be defined!
#endif
#endif
#endif
#endif
#endif
}
