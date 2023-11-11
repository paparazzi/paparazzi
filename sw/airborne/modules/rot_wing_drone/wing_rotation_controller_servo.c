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
#include "modules/rot_wing_drone/rotwing_state.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"

#include <stdlib.h>
#include "mcu_periph/adc.h"

#if USE_NPS
#include "modules/actuators/actuators.h"
#endif


#if !USE_NPS

#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION ADC_5
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES 16
#endif

#endif // !USE_NPS

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


// Inline functions
inline void wing_rotation_adc_to_deg(void);
inline void wing_rotation_compute_pprz_cmd(void);

// Initialization
void wing_rotation_init(void)
{
  // ADC init
#if !USE_NPS
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION, &buf_wing_rot_pos, ADC_CHANNEL_WING_ROTATION_CONTROLLER_POSITION_NB_SAMPLES);
#endif

  // Init Data
  wing_rotation_controller.wing_angle_virtual_deg_sp = 0;
  wing_rotation_controller.wing_rotation_first_order_dynamics = WING_ROTATION_CONTROLLER_FIRST_DYN;
  wing_rotation_controller.wing_rotation_second_order_dynamics = WING_ROTATION_CONTROLLER_SECOND_DYN;
}

void wing_rotation_event(void)
{
  // Update Wing position sensor
  wing_rotation_adc_to_deg();

  // Control the wing rotation position.
  wing_rotation_compute_pprz_cmd();
}

void wing_rotation_adc_to_deg(void)
{
#if !USE_NPS
  // Read ADC
  wing_rotation_controller.adc_wing_rotation = buf_wing_rot_pos.sum / buf_wing_rot_pos.av_nb_sample;

  wing_rotation_controller.wing_angle_deg = 0.00247111 * (float)wing_rotation_controller.adc_wing_rotation - 25.635294;
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
  wing_rotation_controller.wing_angle_deg_sp = rotwing_state_skewing.wing_angle_deg_sp;
  // Smooth accerelation and rate limited setpoint
  float angle_error = wing_rotation_controller.wing_angle_deg_sp - wing_rotation_controller.wing_angle_virtual_deg_sp;
  float speed_sp = wing_rotation_controller.wing_rotation_first_order_dynamics * angle_error;
  float speed_error = speed_sp - wing_rotation_controller.wing_rotation_speed;
  wing_rotation_controller.wing_rotation_speed += wing_rotation_controller.wing_rotation_second_order_dynamics * speed_error;
  wing_rotation_controller.wing_angle_virtual_deg_sp += wing_rotation_controller.wing_rotation_speed;

  // Send to actuators
  int32_t servo_pprz_cmd;
  servo_pprz_cmd = (int32_t)(wing_rotation_controller.wing_angle_virtual_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);
  wing_rotation_controller.servo_pprz_cmd = servo_pprz_cmd;

#if USE_NPS
  actuators_pprz[INDI_NUM_ACT] = servo_pprz_cmd;
#endif
}