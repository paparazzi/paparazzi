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

float pprz_angle_step = 9600. / 45.; // CMD per degree

// Parameters
struct wing_rotation_controller_t wing_rotation_controller = {0};

#if USE_NPS
#include "modules/actuators/actuators.h"
#endif

// Inline functions
inline void wing_rotation_adc_to_deg(void);
inline void wing_rotation_compute_pprz_cmd(void);

void wing_rotation_event(void)
{
  // Control the wing rotation position.
  wing_rotation_compute_pprz_cmd();
  wing_rotation_adc_to_deg();
}

void wing_rotation_adc_to_deg(void)
{
#if USE_NPS
  // Copy setpoint as actual angle in simulation

  // SEND ABI Message
  struct act_feedback_t feedback;
  feedback.idx =  SERVO_ROTATION_MECH;
  feedback.position = 0.5 * M_PI - RadOfDeg(wing_rotation_controller.wing_angle_deg_sp);
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);


  wing_rotation_controller.wing_angle_deg = wing_rotation_controller.wing_angle_deg_sp;
#endif
}

void wing_rotation_compute_pprz_cmd(void)
{
  wing_rotation_controller.wing_angle_deg_sp = rotwing_state_skewing.wing_angle_deg_sp;
#if !USE_NPS
  int32_t servo_pprz_cmd;  // Define pprz cmd

  servo_pprz_cmd = MAX_PPRZ - (int16_t)(wing_rotation_controller.wing_angle_deg_sp * pprz_angle_step);
  // Calulcate rotation_cmd
  Bound(servo_pprz_cmd, -MAX_PPRZ, MAX_PPRZ);
  wing_rotation_controller.servo_pprz_cmd = servo_pprz_cmd;
#else
  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(wing_rotation_controller.wing_angle_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  wing_rotation_controller.servo_pprz_cmd = servo_pprz_cmd;
  actuators_pprz[INDI_NUM_ACT] = servo_pprz_cmd;
#endif
}
