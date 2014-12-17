/*
 * Determines antenna pan angle.
 *
 * project:        Paparazzi
 * description:    Determines antenna pan angle from
 *                 plane's and home's positions and plane's heading angle.
 *                 Software might be optimized
 *                 by removing multiplications with 0, it is left this
 *                 way for better understandabilty and changeability.
 *
 * authors:        Arnold Schroeter, Martin Mueller, Chris Efstathiou
 *
 *
 *
 *
 */

#if defined(USE_AIRBORNE_ANT_TRACKING) && USE_AIRBORNE_ANT_TRACKING == 1

#include <math.h>
#include <inttypes.h>
#include "inter_mcu.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "subsystems/navigation/traffic_info.h"
#include "airborne_ant_track.h"


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

float   airborne_ant_pan;
static bool_t ant_pan_positive = 0;

void ant_point(void);
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC);
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC);

/*******************************************************************
; function name:   vSubtractVectors
; description:     subtracts two vectors a = b - c
; parameters:
;*******************************************************************/
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC)
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
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}

void airborne_ant_point_init(void)
{

  return;
}

void airborne_ant_point_periodic(void)
{
  float airborne_ant_pan_servo = 0;

  static VECTOR svPlanePosition,
         Home_Position,
         Home_PositionForPlane,
         Home_PositionForPlane2;

  static MATRIX smRotation;

  svPlanePosition.fx = stateGetPositionEnu_f()->y;
  svPlanePosition.fy = stateGetPositionEnu_f()->x;
  svPlanePosition.fz = stateGetPositionUtm_f()->alt;

  Home_Position.fx = WaypointY(WP_HOME);
  Home_Position.fy = WaypointX(WP_HOME);
  Home_Position.fz = waypoints[WP_HOME].a;

  /* distance between plane and object */
  vSubtractVectors(&Home_PositionForPlane, Home_Position, svPlanePosition);

  /* yaw */
  smRotation.fx1 = (float)(cos((*stateGetHorizontalSpeedDir_f())));
  smRotation.fx2 = (float)(sin((*stateGetHorizontalSpeedDir_f())));
  smRotation.fx3 = 0.;
  smRotation.fy1 = -smRotation.fx2;
  smRotation.fy2 = smRotation.fx1;
  smRotation.fy3 = 0.;
  smRotation.fz1 = 0.;
  smRotation.fz2 = 0.;
  smRotation.fz3 = 1.;

  vMultiplyMatrixByVector(&Home_PositionForPlane2, smRotation, Home_PositionForPlane);


  /*
   * This is for one axis pan antenna mechanisms. The default is to
   * circle clockwise so view is right. The pan servo neutral makes
   * the antenna look to the right with 0° given, 90° is to the back and
   * -90° is to the front.
   *
   *
   *
   *   plane front
   *
   *                  90
                      ^
   *                  I
   *             135  I  45°
   *                \ I /
   *                 \I/
   *        180-------I------- 0°
   *                 /I\
   *                / I \
   *            -135  I  -45°
   *                  I
   *                -90
   *             plane back
   *
   *
   */

  /* fPan =   0  -> antenna looks along the wing
             90  -> antenna looks in flight direction
            -90  -> antenna looks backwards
  */
  /* fixed to the plane*/
  airborne_ant_pan = (float)(atan2(Home_PositionForPlane2.fx, (Home_PositionForPlane2.fy)));

  // I need to avoid oscillations around the 180 degree mark.
  if (airborne_ant_pan > 0 && airborne_ant_pan <= RadOfDeg(175)) { ant_pan_positive = 1; }
  if (airborne_ant_pan < 0 && airborne_ant_pan >= RadOfDeg(-175)) { ant_pan_positive = 0; }

  if (airborne_ant_pan > RadOfDeg(175) && ant_pan_positive == 0) {
    airborne_ant_pan = RadOfDeg(-180);

  } else if (airborne_ant_pan < RadOfDeg(-175) && ant_pan_positive) {
    airborne_ant_pan = RadOfDeg(180);
    ant_pan_positive = 0;
  }

#ifdef ANT_PAN_NEUTRAL
  airborne_ant_pan = airborne_ant_pan - RadOfDeg(ANT_PAN_NEUTRAL);
  if (airborne_ant_pan > 0) {
    airborne_ant_pan_servo = MAX_PPRZ * (airborne_ant_pan / (RadOfDeg(ANT_PAN_MAX - ANT_PAN_NEUTRAL)));
  } else {
    airborne_ant_pan_servo = MIN_PPRZ * (airborne_ant_pan / (RadOfDeg(ANT_PAN_MIN - ANT_PAN_NEUTRAL)));
  }
#endif

  airborne_ant_pan_servo = TRIM_PPRZ(airborne_ant_pan_servo);

#ifdef COMMAND_ANT_PAN
  ap_state->commands[COMMAND_ANT_PAN] = airborne_ant_pan_servo;
#endif


  return;
}

#endif
