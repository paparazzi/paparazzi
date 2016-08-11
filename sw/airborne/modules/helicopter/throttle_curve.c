/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/helicopter/throttle_curve.c"
 * @author C. De Wagter and Freek van Tienen
 * Throttle Curve Mixers
 */

#include "throttle_curve.h"
#include "subsystems/commands.h"
#include "autopilot.h"
#include "subsystems/radio_control.h"
#include "subsystems/abi.h"

/* The switching values for the Throttle Curve Mode switch */
#define THROTTLE_CURVE_SWITCH_VAL (MAX_PPRZ*2/THROTTLE_CURVES_NB)

/* Default RPM feedback gains */
#ifndef THROTTLE_CURVE_RPM_FB_P
#define THROTTLE_CURVE_RPM_FB_P 0.0
#endif

#ifndef THROTTLE_CURVE_RPM_FB_I
#define THROTTLE_CURVE_RPM_FB_I 0.0
#endif

/* Register the RPM callback */
#ifndef THROTTLE_CURVE_RPM_ID
#define THROTTLE_CURVE_RPM_ID ABI_BROADCAST
#endif
static abi_event rpm_ev;
static void rpm_cb(uint8_t sender_id, uint16_t rpm);

/* Initialize the throttle curves from the airframe file */
struct throttle_curve_t throttle_curve = {
  .nb_curves = THROTTLE_CURVES_NB,
  .curves = THROTTLE_CURVES
};

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void throttle_curve_send_telem(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_THROTTLE_CURVE(trans, dev, AC_ID, &throttle_curve.mode, &throttle_curve.throttle, &throttle_curve.collective,
    &throttle_curve.rpm, &throttle_curve.rpm_meas, &throttle_curve.rpm_err_sum);
}
#endif

/**
 * Initialize the default throttle curve values
 */
void throttle_curve_init(void)
{
  throttle_curve.mode       = THROTTLE_CURVE_MODE_INIT;
  throttle_curve.throttle   = throttle_curve.curves[THROTTLE_CURVE_MODE_INIT].throttle[0];
  throttle_curve.collective = throttle_curve.curves[THROTTLE_CURVE_MODE_INIT].collective[0];
  throttle_curve.rpm_fb_p = THROTTLE_CURVE_RPM_FB_P;
  throttle_curve.rpm_fb_i = THROTTLE_CURVE_RPM_FB_I;
  throttle_curve.rpm_err_sum = 0;
  throttle_curve.rpm_measured = false;

  AbiBindMsgRPM(THROTTLE_CURVE_RPM_ID, &rpm_ev, rpm_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_THROTTLE_CURVE, throttle_curve_send_telem);
#endif
}

/**
 * RPM callback for RPM based control throttle curves
 */
static void rpm_cb(uint8_t __attribute__((unused)) sender_id, uint16_t rpm)
{
  throttle_curve.rpm_meas = rpm;
  throttle_curve.rpm_measured = true;
}

/**
 * Run the throttle curve and generate the output throttle and pitch
 * This depends on the FMODE(flight mode) and TRHUST command
 */
void throttle_curve_run(pprz_t cmds[], uint8_t ap_mode)
{
  // Calculate the mode value from the switch
  if(ap_mode != AP_MODE_NAV) {
    int8_t mode = ((float)(radio_control.values[RADIO_FMODE] + MAX_PPRZ) / THROTTLE_CURVE_SWITCH_VAL);
    Bound(mode, 0, THROTTLE_CURVES_NB - 1);
    throttle_curve.mode = mode;
  }

  // Check if we have multiple points or a single point
  struct curve_t curve = throttle_curve.curves[throttle_curve.mode];
  if (curve.nb_points == 1) {
    throttle_curve.throttle = curve.throttle[0];
    throttle_curve.collective = curve.collective[0];
    throttle_curve.rpm = curve.rpm[0];
  } else {
    // Calculate the left point on the curve we need to use
    uint16_t curve_range = (MAX_PPRZ / (curve.nb_points - 1));
    int8_t curve_p = ((float)cmds[COMMAND_THRUST] / curve_range);
    Bound(curve_p, 0, curve.nb_points - 1);

    // Calculate the throttle, pitch and rpm value
    uint16_t x = cmds[COMMAND_THRUST] - curve_p * curve_range;
    throttle_curve.throttle = curve.throttle[curve_p]
                              + ((curve.throttle[curve_p + 1] - curve.throttle[curve_p]) * x / curve_range);
    throttle_curve.collective = curve.collective[curve_p]
                                + ((curve.collective[curve_p + 1] - curve.collective[curve_p]) * x / curve_range);
    if(curve.rpm[0] != 0xFFFF)
      throttle_curve.rpm = curve.rpm[curve_p]
                            + ((curve.rpm[curve_p + 1] - curve.rpm[curve_p]) * x / curve_range);
    else
      throttle_curve.rpm = 0xFFFF;
  }

  // Update RPM feedback
  if(curve.rpm[0] != 0xFFFF && throttle_curve.rpm_measured) {
    // Calculate RPM error
    int32_t rpm_err = (throttle_curve.rpm - throttle_curve.rpm_meas);

    // Calculate integrated error
    throttle_curve.rpm_err_sum += rpm_err * throttle_curve.rpm_fb_i / 512.0f;
    Bound(throttle_curve.rpm_err_sum, -throttle_curve.throttle, (MAX_PPRZ - throttle_curve.throttle));

    // Calculate feedback command
    int32_t rpm_feedback = rpm_err * throttle_curve.rpm_fb_p + throttle_curve.rpm_err_sum;
    Bound(rpm_feedback, -MAX_PPRZ, MAX_PPRZ);

    // Apply feedback command
    int32_t new_throttle = throttle_curve.throttle + rpm_feedback;
    Bound(new_throttle, 0, MAX_PPRZ);
    throttle_curve.throttle = new_throttle;
    throttle_curve.rpm_measured = false;
  }
  else if(curve.rpm[0] == 0xFFFF) {
    throttle_curve.rpm_err_sum = 0;
  }

  // Set the commands
  cmds[COMMAND_THRUST] = throttle_curve.throttle; //Reuse for now
  cmds[COMMAND_COLLECTIVE] = throttle_curve.collective;

  // Only set throttle if motors are on
  if (!autopilot_motors_on) {
    cmds[COMMAND_THRUST] = 0;
    throttle_curve.rpm_err_sum = 0;
  }
}

/**
 * Set a specific throttle curve based on the mode given with this function
 */
void nav_throttle_curve_set(uint8_t mode)
{
  int16_t new_mode = mode;
  Bound(new_mode, 0, THROTTLE_CURVES_NB - 1);
  throttle_curve.mode = new_mode;
}
