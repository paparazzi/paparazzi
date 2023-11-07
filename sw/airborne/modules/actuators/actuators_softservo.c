/*
 * Copyright (C) 2023 MAVLab
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

/** @file actuators_softservo.c
 *  Drive a motor until an ADC port has the correct value
 */

#include "modules/actuators/actuators.h"
#include "modules/actuators/actuators_softservo.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "mcu_periph/adc.h"
#include "state.h"


/* Currently we only support 1 channels */
#if SERVOS_SOFT_SERVO_NB > 1
#error Soft Servo actuators only support max 1 servo
#endif

/* Calculate the frequency divider to aim at 7 ms */
#if PERIODIC_FREQUENCY < 450
#error Soft Servo actuators need at least a frequency of 500 Hz
#endif

/* Main actuator structure */
struct ActuatorsSoftServo actuators_soft_servo = {0};


/* ADC settings */
#ifndef ACTUATORS_SOFTSERVO_ADC
#define ACTUATORS_SOFTSERVO_ADC ADC_5
#endif

#ifndef ACTUATORS_SOFTSERVO_ADC_NB_SAMPLES
#define ACTUATORS_SOFTSERVO_ADC_NB_SAMPLES 16
#endif

static struct adc_buf buf_softservo_pos;


/* Servo Gains */
#ifndef ACTUATORS_SOFTSERVO_FIRST_DYN
#define ACTUATORS_SOFTSERVO_FIRST_DYN 0.001
#endif

#ifndef ACTUATORS_SOFTSERVO_SECOND_DYN
#define ACTUATORS_SOFTSERVO_SECOND_DYN 0.003
#endif



// Inline functions
inline void actuator_softservo_adc_to_deg(void);
inline void actuator_softservo_control_loop(void);

/*
 * Initialize the soft servo and ADC
 */
void actuators_soft_servo_init(void)
{
  // ADC
  adc_buf_channel(ACTUATORS_SOFTSERVO_ADC, &buf_softservo_pos, ACTUATORS_SOFTSERVO_ADC_NB_SAMPLES);

  // Init actuators_softservo struct
  actuators_softservo.wing_rotation_first_order_dynamics = ACTUATORS_SOFTSERVO_FIRST_DYN;
  actuators_softservo.wing_rotation_second_order_dynamics = ACTUATORS_SOFTSERVO_SECOND_DYN;
}



/*
 * Run the soft servo control loop
 */
void actuators_softservo_update(void)
{
  // Initialize the soft servo if not initialized
  // After 5 sec, set current setpoint and enable wing_rotation
  if (!actuators_softservo.initialized) {
    actuators_softservo.init_loop_count += 1;
    if (actuators_softservo.init_loop_count > 4*PERIODIC_FREQUENCY) {
      actuators_softservo.initialized = true;
    }
  }

  // Update Wing position sensor
  actuator_softservo_adc_to_deg();

  // Run control if initialized
  if (actuators_softservo.initialized) {

    // Setpoint checks
    Bound(actuators_softservo.wing_angle_deg_sp, 0., 90.);

    // Control the wing rotation position.
    actuator_softservo_control_loop();
  }
}

void actuator_softservo_adc_to_deg(void)
{
  actuators_softservo.adc_wing_rotation = buf_softservo_pos.sum / buf_softservo_pos.av_nb_sample;
  float wing_angle_deg = 0.00247111 * (float)actuators_softservo.adc_wing_rotation - 25.635294;

  // SEND ABI Message with Actuator position feedback
  struct act_feedback_t feedback;
  feedback.idx =  SERVO_ROTATION_MECH;
  feedback.position = 0.5 * M_PI - RadOfDeg(wing_angle_deg);
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
}

void actuator_softservo_control_loop(void)
{
  float angle_error = actuators_softservo.wing_angle_deg_sp - actuators_softservo.wing_angle_virtual_deg_sp;
  float speed_sp = actuators_softservo.wing_rotation_first_order_dynamics * angle_error;
  float speed_error = speed_sp - actuators_softservo.wing_rotation_speed;
  actuators_softservo.wing_rotation_speed += actuators_softservo.wing_rotation_second_order_dynamics * speed_error;
  actuators_softservo.wing_angle_virtual_deg_sp += actuators_softservo.wing_rotation_speed;

  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(actuators_softservo.wing_angle_virtual_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  actuators_softservo.servo_pprz_cmd = servo_pprz_cmd;
}
