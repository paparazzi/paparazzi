/** \file formation.c
 *  \brief library for formation flight
 */

#define FORMATION_C

#include <math.h>

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
//#include "uart.h"
#include "downlink.h"

#include "formation.h"
#include "estimator.h"
#include "fw_h_ctl.h"
#include "fw_v_ctl.h"
#include "autopilot.h"
#include "gps.h"
#include "flight_plan.h"
#include "airframe.h"
#include "dl_protocol.h"

#include <stdio.h>

float form_n, form_e, form_a;
float form_speed, form_speed_n, form_speed_e;

float form_carrot;
float form_prox;
float coef_form_pos;
float coef_form_speed;
float coef_form_course;
float coef_form_alt;
int form_mode;
uint8_t leader_id;
float old_cruise, old_alt;

#ifndef FORM_CARROT
#define FORM_CARROT 2.
#endif

#ifndef FORM_POS_PGAIN
#define FORM_POS_PGAIN 0.
#endif

#ifndef FORM_SPEED_PGAIN
#define FORM_SPEED_PGAIN 0.
#endif

#ifndef FORM_COURSE_PGAIN
#define FORM_COURSE_PGAIN 0.
#endif

#ifndef FORM_ALTITUDE_PGAIN
#define FORM_ALTITUDE_PGAIN 0.
#endif

#ifndef FORM_PROX
#define FORM_PROX 20.
#endif

#ifndef FORM_MODE
#define FORM_MODE 0
#endif

int formation_init(void) {
  int i;
  for (i = 0; i < NB_ACS; ++i) formation[i].status = UNSET;

  leader_id = 0;
  form_carrot = FORM_CARROT;
  coef_form_pos = FORM_POS_PGAIN;
  coef_form_speed = FORM_SPEED_PGAIN;
  coef_form_course = FORM_COURSE_PGAIN;
  coef_form_alt = FORM_ALTITUDE_PGAIN;
  form_prox = FORM_PROX;
  form_mode = FORM_MODE;
  old_cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  old_alt = GROUND_ALT + SECURITY_HEIGHT;
  return FALSE;
}

int add_slot(uint8_t _id, float slot_e, float slot_n, float slot_a) {
  DOWNLINK_SEND_FORMATION_SLOT_TM(&_id, &form_mode, &slot_e, &slot_n, &slot_a);
  formation[_id].status = IDLE;
  formation[_id].east = slot_e;
  formation[_id].north = slot_n;
  formation[_id].alt = slot_a;
  return FALSE;
}

int start_formation(void) {
  int i;
  uint8_t ac_id = AC_ID;
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == IDLE) formation[i].status = ACTIVE;
  }
  DOWNLINK_SEND_FORMATION_STATUS_TM(&ac_id,&leader_id,&formation[AC_ID].status);
  // store current cruise and alt
  old_cruise = v_ctl_auto_throttle_cruise_throttle;
  old_alt = nav_altitude;
  return FALSE;
}

int stop_formation(void) {
  int i;
  uint8_t ac_id = AC_ID;
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == ACTIVE) formation[i].status = IDLE;
  }
  DOWNLINK_SEND_FORMATION_STATUS_TM(&ac_id,&leader_id,&formation[AC_ID].status);
  // restore cruise and alt
  v_ctl_auto_throttle_cruise_throttle = old_cruise;
  old_cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  nav_altitude = old_alt;
  old_alt = GROUND_ALT + SECURITY_HEIGHT;
  return FALSE;
}


int formation_flight(void) {

  static uint8_t _1Hz   = 0;
  int nb = 0, i;
  float ch = cos(estimator_hspeed_dir);
  float sh = sin(estimator_hspeed_dir);
  form_n = 0.;
  form_e = 0.;
  form_a = 0.;
  form_speed = estimator_hspeed_mod;
  form_speed_n = estimator_hspeed_mod * ch;
  form_speed_e = estimator_hspeed_mod * sh;

  // broadcast info
  uint8_t ac_id = AC_ID;
  DOWNLINK_SEND_FORMATION_STATUS_TM(&ac_id,&leader_id,&formation[AC_ID].status);
  if (++_1Hz>=4) {
    _1Hz=0;
    DOWNLINK_SEND_FORMATION_SLOT_TM(&ac_id, &form_mode,
        &formation[AC_ID].east,
        &formation[AC_ID].north,
        &formation[AC_ID].alt);
  }

  // set info for this AC
  SetAcInfo(AC_ID, estimator_x, estimator_y, estimator_hspeed_dir, estimator_z, estimator_hspeed_mod, gps_itow);
  if (formation[AC_ID].status != ACTIVE) return FALSE; // AC not ready

  // get leader info
  struct ac_info_ * leader = get_ac_info(leader_id);
  if (formation[leader_id].status != ACTIVE) return FALSE; // leader not ready
  //if (formation[leader_id].status == UNSET) return FALSE; // leader not ready
  //else if (formation[leader_id].status == IDLE) {
  //    if(Max(((int)gps_itow - (int)leader->itow) / 1000., 0.) > FORM_CARROT) return TRUE; // still not ready
  //    else formation[leader_id].status = ACTIVE;
  //}

  // compute slots in the right reference frame
  struct slot_ form[NB_ACS];
  float cr = 0., sr = 1.;
  if (form_mode == 1.) {
    cr = cos(leader->course);
    sr = sin(leader->course);
  }
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == UNSET) continue;
    form[i].east  = formation[i].east*sr - formation[i].north*cr;
    form[i].north = formation[i].east*cr + formation[i].north*sr;
    form[i].alt = formation[i].alt;
  }

  // compute control forces
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == UNSET || i == AC_ID) continue;
    struct ac_info_ * ac = get_ac_info(i);
    if (fabs(estimator_z - ac->alt) < form_prox && ac->alt > 0) {
      float delta_t = Max(((int)gps_itow - (int)ac->itow) / 1000., 0.);
      //printf("dt %d %d %u %u %f \n",AC_ID,i,gps_itow,ac->itow,delta_t);
      if (delta_t > FORM_CARROT) {
        // if AC not responding for too long
        formation[i].status = IDLE;
        continue;
      }
      else formation[i].status = ACTIVE;
      form_e += (ac->east  + ac->gspeed*sin(ac->course)*delta_t - estimator_x)
        - (form[i].east - form[AC_ID].east);
      form_n += (ac->north + ac->gspeed*cos(ac->course)*delta_t - estimator_y)
        - (form[i].north - form[AC_ID].north);
      form_a += (ac->alt - estimator_z) - (formation[i].alt - formation[AC_ID].alt);
      form_speed += ac->gspeed;
      //form_speed_e += ac->gspeed * sin(ac->course);
      //form_speed_n += ac->gspeed * cos(ac->course);
      ++nb;
    }
  }
  if (nb > 0) {
    form_n /= nb;
    form_e /= nb;
    form_a /= nb;
  }
  form_speed = form_speed / (nb+1) - estimator_hspeed_mod;
  //form_speed_e = form_speed_e / (nb+1) - estimator_hspeed_mod * sh;
  //form_speed_n = form_speed_n / (nb+1) - estimator_hspeed_mod * ch;

  // set commands
  NavVerticalAutoThrottleMode(0.);

  // altitude loop
  float alt = 0.;
  if (AC_ID == leader_id) alt = nav_altitude;
  else alt = leader->alt - form[leader_id].alt;
  alt += formation[AC_ID].alt + coef_form_alt * form_a;
  //NavVerticalAltitudeMode(Max(alt, ground_alt+SECURITY_HEIGHT), 0.);
  flight_altitude = Max(alt, ground_alt+SECURITY_HEIGHT);

  // carrot
  if (AC_ID != leader_id) {
    float dx = form[AC_ID].east - form[leader_id].east;
    float dy = form[AC_ID].north - form[leader_id].north;
    desired_x = leader->east  + NOMINAL_AIRSPEED * form_carrot * sin(leader->course) + dx;
    desired_y = leader->north + NOMINAL_AIRSPEED * form_carrot * cos(leader->course) + dy;
    // scale course_pgain only for followers
    h_ctl_course_pgain = -coef_form_course;
    // fly to desired
    fly_to_xy(desired_x, desired_y);
    desired_x = leader->east  + dx;
    desired_y = leader->north + dy;
    //fly_to_xy(desired_x, desired_y);
    // lateral correction
    //float diff_heading = asin((dx*ch - dy*sh) / sqrt(dx*dx + dy*dy));
    //float diff_course = leader->course - estimator_hspeed_dir;
    //NormRadAngle(diff_course);
    //h_ctl_roll_setpoint += coef_form_course * diff_course;
    //h_ctl_roll_setpoint += coef_form_course * diff_heading;
  }
  else {
    desired_x += form[leader_id].east;
    desired_y += form[leader_id].north;
    // fly to desired
    fly_to_xy(desired_x, desired_y);
  }
  //BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

  // speed loop
  float cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  cruise += coef_form_pos * (form_n * ch + form_e * sh) + coef_form_speed * form_speed;
  Bound(cruise, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
  v_ctl_auto_throttle_cruise_throttle = cruise;

  return TRUE;
}


