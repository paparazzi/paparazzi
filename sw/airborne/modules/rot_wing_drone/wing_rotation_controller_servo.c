/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rot_wing_drone/wing_rotation_controller_v3b.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module to control wing rotation servo command based on prefered angle setpoint
 */

#include "modules/rot_wing_drone/wing_rotation_controller_servo.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"

#include "modules/core/abi.h"
#include "state.h"

#include <stdlib.h>
#include "mcu_periph/adc.h"


#if !USE_NPS

#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION ADC_5
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES 16
#endif

#endif // !USE_NPS

#ifndef WING_ROTATION_CONTROLLER_POSITION_ADC_0
#error "No WING_ROTATION_CONTROLLER_POSITION_ADC_0 defined"
#endif

#ifndef WING_ROTATION_CONTROLLER_POSITION_ADC_90
#error "No WING_ROTATION_CONTROLLER_POSITION_ADC_90 defined"
#endif

#ifndef WING_ROTATION_CONTROLLER_FIRST_DYN
#define WING_ROTATION_CONTROLLER_FIRST_DYN 0.001
#endif

#ifndef WING_ROTATION_CONTROLLER_SECOND_DYN
#define WING_ROTATION_CONTROLLER_SECOND_DYN 0.003
#endif

// Parameters
struct wing_rotation_controller_t wing_rotation_controller = {0};

bool in_transition = false;

#if !USE_NPS
static struct adc_buf buf_wing_rot_pos;
#endif

#if USE_NPS
#include "modules/actuators/actuators.h"
#endif

// Inline functions
inline void wing_rotation_adc_to_deg(void);
inline void wing_rotation_compute_pprz_cmd(void);

void wing_rotation_init(void)
{
  // your init code here
#if !USE_NPS
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION, &buf_wing_rot_pos, ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES);
#endif

  // Init wing_rotation_controller struct
  wing_rotation_controller.wing_angle_virtual_deg_sp = 45;
  wing_rotation_controller.wing_rotation_first_order_dynamics = WING_ROTATION_CONTROLLER_FIRST_DYN;
  wing_rotation_controller.wing_rotation_second_order_dynamics = WING_ROTATION_CONTROLLER_SECOND_DYN;
}

void wing_rotation_periodic(void)
{
  // After 5 loops, set current setpoint and enable wing_rotation
  // freq = 1.0 Hz
  if (!wing_rotation_controller.initialized) {
    wing_rotation_controller.init_loop_count += 1;
    if (wing_rotation_controller.init_loop_count > 4) {
      wing_rotation_controller.initialized = true;
      wing_rotation_controller.wing_angle_deg_sp = 45.;
    }
  }
}

void wing_rotation_event(void)
{
  // First check if safety switch is triggered
#ifdef WING_ROTATION_CONTROLLER_RESET_RADIO_CHANNEL
  // Update wing_rotation deg setpoint when RESET switch triggered
  if (radio_control.values[WING_ROTATION_CONTROLLER_RESET_RADIO_CHANNEL] > 1750) {
    wing_rotation_controller.airspeed_scheduling = false;
    wing_rotation_controller.wing_angle_deg_sp = 0;
  }
#endif

  if (!wing_rotation_controller.airspeed_scheduling) {
    wing_rotation_controller.transition_forward = false;
  }

  // Update Wing position sensor
  wing_rotation_adc_to_deg();

  // Run control if initialized
  if (wing_rotation_controller.initialized) {

    if (wing_rotation_controller.airspeed_scheduling) {
      float wing_angle_scheduled_sp_deg = 0;
      float airspeed = stateGetAirspeed_f();
      if (airspeed < 8) {
        in_transition = false;
        wing_angle_scheduled_sp_deg = 0;
      } else if (airspeed < 10 && in_transition) {
        wing_angle_scheduled_sp_deg = 55;
      } else if (airspeed > 10) {
        wing_angle_scheduled_sp_deg = ((airspeed - 10.)) / 4. * 35. + 55.;
        in_transition = true;
      } else {
        wing_angle_scheduled_sp_deg = 0;
      }

      Bound(wing_angle_scheduled_sp_deg, 0., 90.)
      wing_rotation_controller.wing_angle_deg_sp = wing_angle_scheduled_sp_deg;

    }

    // Setpoint checks
    Bound(wing_rotation_controller.wing_angle_deg_sp, 0., 90.);

    // Control the wing rotation position.
    wing_rotation_compute_pprz_cmd();
  }
}

void wing_rotation_adc_to_deg(void)
{
#if !USE_NPS
#if ROTWING_V3B
  wing_rotation_controller.adc_wing_rotation = buf_wing_rot_pos.sum / buf_wing_rot_pos.av_nb_sample;

  wing_rotation_controller.wing_angle_deg = 0.00247111 * (float)wing_rotation_controller.adc_wing_rotation - 25.635294;
#else
wing_rotation_controller.wing_angle_deg = 0;
#endif // ROTWING_V3B

#else // !USE_NPS
  // Copy setpoint as actual angle in simulation
  wing_rotation_controller.wing_angle_deg = wing_rotation_controller.wing_angle_virtual_deg_sp;
#endif

  // SEND ABI Message to ctr_eff_sched and other modules that want Actuator position feedback
  struct act_feedback_t feedback;
  feedback.idx =  SERVO_ROTATION_MECH;
  feedback.position = 0.5 * M_PI - RadOfDeg(wing_rotation_controller.wing_angle_deg);
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);


}

void wing_rotation_compute_pprz_cmd(void)
{
  float angle_error = wing_rotation_controller.wing_angle_deg_sp - wing_rotation_controller.wing_angle_virtual_deg_sp;
  float speed_sp = wing_rotation_controller.wing_rotation_first_order_dynamics * angle_error;
  float speed_error = speed_sp - wing_rotation_controller.wing_rotation_speed;
  wing_rotation_controller.wing_rotation_speed += wing_rotation_controller.wing_rotation_second_order_dynamics * speed_error;
  wing_rotation_controller.wing_angle_virtual_deg_sp += wing_rotation_controller.wing_rotation_speed;

#if !USE_NPS
  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(wing_rotation_controller.wing_angle_virtual_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  wing_rotation_controller.servo_pprz_cmd = servo_pprz_cmd;
#else
  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(wing_rotation_controller.wing_angle_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  wing_rotation_controller.servo_pprz_cmd = servo_pprz_cmd;
  actuators_pprz[INDI_NUM_ACT] = servo_pprz_cmd;
#endif
}

// Function to call in flightplan to switch scheduler on or off
bool set_wing_rotation_scheduler(bool rotation_scheduler_on)
{
  if (rotation_scheduler_on) {
    wing_rotation_controller.airspeed_scheduling = true;
  } else {
    wing_rotation_controller.airspeed_scheduling = false;
    if (!wing_rotation_controller.force_rotation_angle) {
      wing_rotation_controller.wing_angle_deg_sp = 0;
    }
  }
  return false;
}
