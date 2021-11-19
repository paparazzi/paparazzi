/** \file formation.c
 *  \brief library for formation flight
 */

#define FORMATION_C

#include "multi/formation.h"

#include "std.h"
#include "state.h"

#include "modules/datalink/downlink.h"

#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"

#include "generated/flight_plan.h"    // SECURITY_HEIGHT

float form_n, form_e, form_a;
float form_speed, form_speed_n, form_speed_e;

float form_carrot;
float form_prox;
float coef_form_pos;
float coef_form_speed;
float coef_form_course;
float coef_form_alt;
uint8_t form_mode;
uint8_t leader_id;
float old_cruise, old_alt;

struct slot_ formation[NB_ACS];

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

int formation_init(void)
{
  int i;
  for (i = 0; i < NB_ACS; ++i) { formation[i].status = UNSET; }

  leader_id = 0;
  form_carrot = FORM_CARROT;
  coef_form_pos = FORM_POS_PGAIN;
  coef_form_speed = FORM_SPEED_PGAIN;
  coef_form_course = FORM_COURSE_PGAIN;
  coef_form_alt = FORM_ALTITUDE_PGAIN;
  form_prox = FORM_PROX;
  form_mode = FORM_MODE;
  old_cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  old_alt = ground_alt + SECURITY_HEIGHT;
  return false;
}

int add_slot(uint8_t _id, float slot_e, float slot_n, float slot_a)
{
  if (_id != AC_ID && ti_acs_id[_id] == 0) { return false; } // no info for this AC
  DOWNLINK_SEND_FORMATION_SLOT_TM(DefaultChannel, DefaultDevice, &_id, &form_mode, &slot_e, &slot_n, &slot_a);
  formation[ti_acs_id[_id]].status = IDLE;
  formation[ti_acs_id[_id]].east = slot_e;
  formation[ti_acs_id[_id]].north = slot_n;
  formation[ti_acs_id[_id]].alt = slot_a;
  return false;
}

int start_formation(void)
{
  uint8_t i;
  uint8_t ac_id = AC_ID;
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == IDLE) { formation[i].status = ACTIVE; }
  }
  uint8_t active = ACTIVE;
  DOWNLINK_SEND_FORMATION_STATUS_TM(DefaultChannel, DefaultDevice, &ac_id, &leader_id, &active);
  // store current cruise and alt
  old_cruise = v_ctl_auto_throttle_cruise_throttle;
  old_alt = nav_altitude;
  return false;
}

int stop_formation(void)
{
  uint8_t i;
  uint8_t ac_id = AC_ID;
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == ACTIVE) { formation[i].status = IDLE; }
  }
  uint8_t idle = IDLE;
  DOWNLINK_SEND_FORMATION_STATUS_TM(DefaultChannel, DefaultDevice, &ac_id, &leader_id, &idle);
  // restore cruise and alt
  v_ctl_auto_throttle_cruise_throttle = old_cruise;
  old_cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  nav_altitude = old_alt;
  old_alt = ground_alt + SECURITY_HEIGHT;
  return false;
}


int formation_flight(void)
{
  static uint8_t _1Hz = 0;
  uint8_t nb = 0, i;
  float hspeed_dir = stateGetHorizontalSpeedDir_f();
  float ch = cosf(hspeed_dir);
  float sh = sinf(hspeed_dir);
  form_n = 0.;
  form_e = 0.;
  form_a = 0.;
  form_speed = stateGetHorizontalSpeedNorm_f();
  form_speed_n = form_speed * ch;
  form_speed_e = form_speed * sh;

  if (AC_ID == leader_id) {
    stateGetPositionEnu_f()->x += formation[ti_acs_id[AC_ID]].east;
    stateGetPositionEnu_f()->y += formation[ti_acs_id[AC_ID]].north;
  }

  // broadcast info
  uint8_t ac_id = AC_ID;
  uint8_t status = formation[ti_acs_id[AC_ID]].status;
  DOWNLINK_SEND_FORMATION_STATUS_TM(DefaultChannel, DefaultDevice, &ac_id, &leader_id, &status);
  if (++_1Hz >= 4) {
    _1Hz = 0;
    DOWNLINK_SEND_FORMATION_SLOT_TM(DefaultChannel, DefaultDevice, &ac_id, &form_mode,
                                    &formation[ti_acs_id[AC_ID]].east,
                                    &formation[ti_acs_id[AC_ID]].north,
                                    &formation[ti_acs_id[AC_ID]].alt);
  }
  if (formation[ti_acs_id[AC_ID]].status != ACTIVE) { return false; } // AC not ready

  // get leader info
  struct EnuCoor_f *leader_pos = acInfoGetPositionEnu_f(leader_id);
  if (formation[ti_acs_id[leader_id]].status == UNSET ||
      formation[ti_acs_id[leader_id]].status == IDLE) {
    // leader not ready or not in formation
    return false;
  }

  // compute slots in the right reference frame
  struct slot_ form[NB_ACS];
  float cr = 0., sr = 1.;
  if (form_mode == FORM_MODE_COURSE) {
    cr = cosf(acInfoGetCourse(leader_id));
    sr = sinf(acInfoGetCourse(leader_id));
  }
  for (i = 0; i < NB_ACS; ++i) {
    if (formation[i].status == UNSET) { continue; }
    form[i].east  = formation[i].east * sr - formation[i].north * cr;
    form[i].north = formation[i].east * cr + formation[i].north * sr;
    form[i].alt = formation[i].alt;
  }

  struct EnuCoor_f *my_pos = stateGetPositionEnu_f();
  // compute control forces
  for (i = 0; i < NB_ACS; ++i) {
    if (ti_acs[i].ac_id == AC_ID) { continue; }
    struct EnuCoor_f *ac = acInfoGetPositionEnu_f(ti_acs[i].ac_id);
    struct EnuCoor_f *ac_speed = acInfoGetVelocityEnu_f(ti_acs[i].ac_id);

    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[i].ac_id)) / 1000., 0.);
    if (delta_t > FORM_CARROT) {
      // if AC not responding for too long
      formation[i].status = LOST;
      continue;
    } else {
      // compute control if AC is ACTIVE and around the same altitude (maybe not so useful)
      formation[i].status = ACTIVE;
      if (ac->z > 0 && fabs(my_pos->z - ac->z) < form_prox) {
        form_e += (ac->x + ac_speed->x * delta_t - my_pos->x) - (form[i].east  - form[ti_acs_id[AC_ID]].east);
        form_n += (ac->y + ac_speed->y * delta_t - my_pos->y) - (form[i].north - form[ti_acs_id[AC_ID]].north);
        form_a += (ac->z + ac_speed->z * delta_t - my_pos->z) - (form[i].alt   - form[ti_acs_id[AC_ID]].alt);
        form_speed += acInfoGetGspeed(ti_acs[i].ac_id);
        //form_speed_e += ac->gspeed * sinf(ac->course);
        //form_speed_n += ac->gspeed * cosf(ac->course);
        ++nb;
      }
    }
  }
  uint8_t _nb = Max(1, nb);
  form_n /= _nb;
  form_e /= _nb;
  form_a /= _nb;
  form_speed = form_speed / (nb + 1) - stateGetHorizontalSpeedNorm_f();
  //form_speed_e = form_speed_e / (nb+1) - stateGetHorizontalSpeedNorm_f() * sh;
  //form_speed_n = form_speed_n / (nb+1) - stateGetHorizontalSpeedNorm_f() * ch;

  // set commands
  NavVerticalAutoThrottleMode(0.);

  // altitude loop
  float alt = 0.;
  if (AC_ID == leader_id) {
    alt = nav_altitude;
  } else {
    alt = leader_pos->z - form[ti_acs_id[leader_id]].alt;
  }
  alt += formation[ti_acs_id[AC_ID]].alt + coef_form_alt * form_a;
  flight_altitude = Max(alt, ground_alt + SECURITY_HEIGHT);

  // carrot
  if (AC_ID != leader_id) {
    float dx = form[ti_acs_id[AC_ID]].east - form[ti_acs_id[leader_id]].east;
    float dy = form[ti_acs_id[AC_ID]].north - form[ti_acs_id[leader_id]].north;
    desired_x = leader_pos->x  + NOMINAL_AIRSPEED * form_carrot * sinf(acInfoGetCourse(leader_id)) + dx;
    desired_y = leader_pos->y + NOMINAL_AIRSPEED * form_carrot * cosf(acInfoGetCourse(leader_id)) + dy;
    // fly to desired
    fly_to_xy(desired_x, desired_y);
    desired_x = leader_pos->x  + dx;
    desired_y = leader_pos->y + dy;
    // lateral correction
    //float diff_heading = asin((dx*ch - dy*sh) / sqrt(dx*dx + dy*dy));
    //float diff_course = leader->course - hspeed_dir;
    //NormRadAngle(diff_course);
    //h_ctl_roll_setpoint += coef_form_course * diff_course;
    //h_ctl_roll_setpoint += coef_form_course * diff_heading;
  }
  //BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

  // speed loop
  if (nb > 0) {
    float cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
    cruise += coef_form_pos * (form_n * ch + form_e * sh) + coef_form_speed * form_speed;
    Bound(cruise, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
    v_ctl_auto_throttle_cruise_throttle = cruise;
  }
  return true;
}

void formation_pre_call(void)
{
  if (leader_id == AC_ID) {
    stateGetPositionEnu_f()->x -= formation[ti_acs_id[AC_ID]].east;
    stateGetPositionEnu_f()->y -= formation[ti_acs_id[AC_ID]].north;
  }
}

