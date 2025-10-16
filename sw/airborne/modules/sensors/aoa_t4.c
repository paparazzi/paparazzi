/*
 * Copyright (C) 2024 The Paparazzi Team
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

/**
 * @file modules/sensors/aoa_t4.c
 * @brief Angle of Attack sensor using T4 Actuators Board and a modified serial bus servo added wind vane. The SBS must be of hall effect type to minimize friction.
 * @author: Sunyou Hwang, OpenUAS
 */

#include "modules/sensors/aoa_t4.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"
#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "filters/low_pass_filter.h"
// #include "modules/sensors/actuators_t4_uart.h"

/// Default offset value (assuming 0 AOA is in the middle of the range)
#ifndef AOA_ANGLE_OFFSET
#define AOA_ANGLE_OFFSET M_PI
#endif

/// Small offset value in radians to get the sensor perfectly lined out
#ifndef AOA_T4_OFFSET
#define AOA_T4_OFFSET 0.0f
#endif

/// Use lowpass filter for AOA
#ifndef AOA_T4_USE_FILTER
#define AOA_T4_USE_FILTER FALSE
#endif

/// Default filter value for reasonable minimum filtering
/** Time constant for second order Butterworth low pass filter
 * Default of 0.15 should give cut-off freq of 1/(2*pi*tau) ~= 1Hz
 * (0.01->15.9 / 0.05->3.2 / 0.1->1.59 / 0.2->0.79 / 0.3->0.53)
 */
#ifndef AOA_T4_FILTER
#define AOA_T4_FILTER 0.15f
#endif

/// To set reverse direction on readings use true
#if AOA_T4_REVERSE
#define AOA_T4_SIGN -1
#else
#define AOA_T4_SIGN 1
#endif

// Enable telemetry report
#ifndef AOA_T4_SYNC_SEND
#define AOA_T4_SYNC_SEND FALSE
#endif

/// Servo ID to use for the modified servo as AOA sensor
#ifndef AOA_T4_SERVO_ID
#error "No AOA_T4_SERVO_ID defined, however it must be defined to be able to use the aoa_t4 module"
#endif

#ifdef AOA_T4_USE_FILTER
Butterworth2LowPass aoa_t4_lowpass_filter;
#ifndef AOA_T4_FILTER_SAMPLING_TIME
#define AOA_T4_FILTER_SAMPLING_TIME 0.0005 // 200Hz
#endif
#endif

struct Aoa_T4 aoa_t4;

/// Enable A1 A2 to Theta compensation for AOA sensor angle
#ifndef AOA_T4_USE_COMPENSATION
#define AOA_T4_USE_COMPENSATION FALSE
#endif

// linear fn; aoa += A*theta + B;
// aoa += A1*theta+B1 when theta >= 0
// aoa += A2*theta+B2 when theta < 0
#ifndef AOA_T4_COMP_A1
#define AOA_T4_COMP_A1 0.0f
#endif

#ifndef AOA_T4_COMP_B1
#define AOA_T4_COMP_B1 0.0f
#endif

#ifndef AOA_T4_COMP_A2
#define AOA_T4_COMP_A2 -0.5457f // TODO: Explain the magic number...
#endif

#ifndef AOA_T4_COMP_B2
#define AOA_T4_COMP_B2 0.0f
#endif

/* SSA */

/// Default offset value (assuming 0 SSA is in the middle of the range)
#ifndef SSA_ANGLE_OFFSET
#define SSA_ANGLE_OFFSET M_PI
#endif

/// Small offset value in radians to get the sensor perfectly lined out
#ifndef SSA_T4_OFFSET
#define SSA_T4_OFFSET 0.0f
#endif

/// Use lowpass filter for SSA
#ifndef SSA_T4_USE_FILTER
#define SSA_T4_USE_FILTER FALSE
#endif

#ifndef SSA_T4_FILTER
#define SSA_T4_FILTER 0.20f
#endif

#if SSA_T4_REVERSE
#define SSA_T4_SIGN -1
#else
#define SSA_T4_SIGN 1
#endif

#ifdef SSA_T4_USE_FILTER
Butterworth2LowPass ssa_t4_lowpass_filter;
//Use same AOA_T4_FILTER_SAMPLING_TIME
#endif

struct Aoa_T4 ssa_t4;

struct ActuatorsT4In actuators_t4_in_local;

static abi_event ACTUATORS_T4_IN;

struct FloatEulers eulers_t4;
float aoa_t4_a1 = AOA_T4_COMP_A1;
float aoa_t4_b1 = AOA_T4_COMP_B1;
float aoa_t4_a2 = AOA_T4_COMP_A2;
float aoa_t4_b2 = AOA_T4_COMP_B2;

enum Aoa_Type aoa_send_type;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_aoa(struct transport_tx *trans, struct link_device *dev)
{
  // FIXME: Add a "sensor_id" uint8_t to AOA message to nicer send sideslip more important, be able to combine all AOA sensors like AOA_adc, AOA_pwm etc sensors
  switch (aoa_send_type) {
    case SEND_TYPE_SIDESLIP:
      pprz_msg_send_AOA(trans, dev, AC_ID, &ssa_t4.raw, &ssa_t4.angle);
      break;
    case SEND_TYPE_AOA:
    default:
      pprz_msg_send_AOA(trans, dev, AC_ID, &aoa_t4.raw, &aoa_t4.angle);
      break;
  }
}

#endif

/* The incoming angle from T4 is Degrees x 100 from -18000 to 18000 need to be re-mapped from 0 upto 36000
   else it will not fit the raw filed with uint32  type as in default AOA raw message */
uint32_t convert_angle_x100_to_raw(int16_t angle)
{
    int16_t input_min = -18000; // The default value in T4 Actuators Board code
    int16_t input_max = 18000;  // Also max default in T4 Actuators Board code
    // Conversion range
    uint32_t output_min = 0;
    uint32_t output_max = abs(input_min) + input_max;
    // Perform the mapping
    uint32_t result = (uint32_t)((((int32_t)angle - input_min) * (uint64_t)(output_max - output_min)) / (input_max - input_min) + output_min);
    return result;
}

static void actuators_t4_abi_in(uint8_t sender_id __attribute__((unused)), struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr __attribute__((unused)))
{
    memcpy(&actuators_t4_in_local, actuators_t4_in_ptr, sizeof(struct ActuatorsT4In));

    int16_t angle = 0;

    switch (AOA_T4_SERVO_ID)
    {
    /*
    Note that the T4 Actuator Board has a max of 10 serial bus servos that can give real feedback,
    the PWM devices 11 and 12 do NOT GIVE real angle FEEDBACK
    */

    case 1:
        angle = actuators_t4_in_ptr->servo_1_angle;
        break;
    case 2:
        angle = actuators_t4_in_ptr->servo_2_angle;
        break;
    case 3:
        angle = actuators_t4_in_ptr->servo_3_angle;
        break;
    case 4:
        angle = actuators_t4_in_ptr->servo_4_angle;
        break;
    case 5:
        angle = actuators_t4_in_ptr->servo_5_angle;
        break;
    case 6:
        angle = actuators_t4_in_ptr->servo_6_angle;
        break;
    case 7:
        angle = actuators_t4_in_ptr->servo_7_angle;
        break;
    case 8:
        angle = actuators_t4_in_ptr->servo_8_angle;
        break;
    case 9:
        angle = actuators_t4_in_ptr->servo_9_angle;
        break;
    case 10:
        angle = actuators_t4_in_ptr->servo_10_angle;
        break;
    default:
        break;
    }

    aoa_t4.raw = convert_angle_x100_to_raw(angle); // To adhere to default message type of uint32_t so from 0 to 36000

#ifdef SSA_T4_SERVO_ID

    angle = 0;

    switch (SSA_T4_SERVO_ID)
    {
    case 1:
        angle = actuators_t4_in_ptr->servo_1_angle;
        break;
    case 2:
        angle = actuators_t4_in_ptr->servo_2_angle;
        break;
    case 3:
        angle = actuators_t4_in_ptr->servo_3_angle;
        break;
    case 4:
        angle = actuators_t4_in_ptr->servo_4_angle;
        break;
    case 5:
        angle = actuators_t4_in_ptr->servo_5_angle;
        break;
    case 6:
        angle = actuators_t4_in_ptr->servo_6_angle;
        break;
    case 7:
        angle = actuators_t4_in_ptr->servo_7_angle;
        break;
    case 8:
        angle = actuators_t4_in_ptr->servo_8_angle;
        break;
    case 9:
        angle = actuators_t4_in_ptr->servo_9_angle;
        break;
    case 10:
        angle = actuators_t4_in_ptr->servo_10_angle;
        break;
    default:
        break;
    }

    ssa_t4.raw = convert_angle_x100_to_raw(angle);

#endif //SSA_T4_SERVO_ID

}

void aoa_t4_init_filters(void)
{
    init_butterworth_2_low_pass(&aoa_t4_lowpass_filter, AOA_T4_FILTER,
                                AOA_T4_FILTER_SAMPLING_TIME, 0);
}

void ssa_t4_init_filters(void)
{
    init_butterworth_2_low_pass(&ssa_t4_lowpass_filter, SSA_T4_FILTER,
                                AOA_T4_FILTER_SAMPLING_TIME, 0);
}

void aoa_t4_init(void)
{
    aoa_t4.raw = 0;
    aoa_t4.angle = 0.0f;
    aoa_t4.offset = AOA_T4_OFFSET;
    aoa_t4.filter = AOA_T4_FILTER;

    ssa_t4.raw = 0;
    ssa_t4.angle = 0.0f;
    ssa_t4.offset = SSA_T4_OFFSET;
    ssa_t4.filter = SSA_T4_FILTER;

    aoa_send_type = SEND_TYPE_AOA; //Per default send AOA

#if AOA_T4_USE_FILTER
    aoa_t4_init_filters();
#endif

#if SSA_T4_USE_FILTER
    ssa_t4_init_filters();
#endif

    // IN indicates INcoming values from the T4 Actuator Board 
    AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &ACTUATORS_T4_IN, actuators_t4_abi_in);

#if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AOA, send_aoa);
#endif
}

void aoa_t4_update(void)
{
#if USE_AOA || USE_SIDESLIP
  uint8_t flag = 0;
#endif

    /* Convert raw value of degrees x 100 (0 to 36000) to real degrees and to radians and add offsets */
    float a_rad;
    a_rad = AOA_T4_SIGN * ((float)(aoa_t4.raw / 100.0f) * (M_PI / 180.0f)) + aoa_t4.offset + AOA_ANGLE_OFFSET;
    if (a_rad > M_PI) a_rad -= 2 * M_PI; // Adjust range to be from -PI to PI
    aoa_t4.angle = a_rad;

#if AOA_T4_USE_COMPENSATION
    float_eulers_of_quat_zxy(&eulers_t4, stateGetNedToBodyQuat_f());
    // First order fn
    if (eulers_t4.theta >= 0)
    {
        aoa_t4.angle += aoa_t4_a1 * eulers_t4.theta + aoa_t4_b1;
    }
    else
    {
        aoa_t4.angle += aoa_t4_a2 * eulers_t4.theta + aoa_t4_b2;
    }
#endif

#if AOA_T4_USE_FILTER
    aoa_t4.angle = update_butterworth_2_low_pass(&aoa_t4_lowpass_filter, aoa_t4.angle);
#endif

    a_rad = SSA_T4_SIGN * ((float)(ssa_t4.raw / 100.0f) * (M_PI / 180.0f)) + ssa_t4.offset + SSA_ANGLE_OFFSET;
    if (a_rad > M_PI) a_rad -= 2 * M_PI;
    ssa_t4.angle = a_rad;

#if SSA_T4_USE_FILTER
    ssa_t4.angle = update_butterworth_2_low_pass(&aoa_t4_lowpass_filter, ssa_t4.angle);
#endif

 // Do set values in the state for AOA
#if USE_AOA
    SetBit(flag, 0); // Bit 1 is for AOA
#endif 

// Do set values in the state for sideslip
#ifdef USE_SIDESLIP
    SetBit(flag, 1); // Bit 2 is for SSA
#endif

#if USE_AOA || USE_SIDESLIP
    AbiSendMsgINCIDENCE(AOA_PWM_ID, flag, aoa_t4.angle, ssa_t4.angle);
#endif

#if AOA_T4_SYNC_SEND
    RunOnceEvery(10, send_aoa(&(DefaultChannel).trans_tx, &(DefaultDevice).device));
#endif

}
