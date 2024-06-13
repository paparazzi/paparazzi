/*
 * Copyright (C) 2010 The Paparazzi Team
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
 * @brief Angle of Attack sensor using Teensy 4.0 + servo
 * Autor: Sunyou Hwang
 */

#include "modules/sensors/aoa_t4.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"
#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "filters/low_pass_filter.h"
#include "modules/sensors/serial_act_t4.h"

/// Offset value in radian
#ifndef AOA_T4_OFFSET
#define AOA_T4_OFFSET 0
#endif
/// Use lowpass filter
#ifndef AOA_T4_USE_LOWPASS_FILTER
#define AOA_T4_USE_LOWPASS_FILTER FALSE
#endif
/** Time constant for second order Butterworth low pass filter
 * Default of 0.15 should give cut-off freq of 1/(2*pi*tau) ~= 1Hz
 * (0.01->15.9 / 0.05->3.2 / 0.1->1.59 / 0.2->0.79 / 0.3->0.53)
 */
#ifndef AOA_T4_LOWPASS_TAU
#define AOA_T4_LOWPASS_TAU 0.15
#endif
/// To use inverted direction; use -1 for inverted direction
#ifndef AOA_T4_SIGN
#define AOA_T4_SIGN 1
#endif
/// Servo ID
#ifndef AOA_T4_SERVO_ID
#error "AOA_T4_SERVO_ID must be defined to use AOA_t4 module"
#endif

#ifdef AOA_T4_USE_LOWPASS_FILTER
Butterworth2LowPass aoa_t4_lowpass_filter;
#ifndef AOA_T4_FILTER_SAMPLING_TIME
#define AOA_T4_FILTER_SAMPLING_TIME 0.0005       // 200Hz
#endif
#endif

#ifndef AOA_T4_USE_COMPENSATION
#define AOA_T4_USE_COMPENSATION FALSE
#endif

// linear fn; aoa += A*theta + B;
// aoa += A1*theta+B1 when theta >= 0
// aoa += A2*theta+B2 when theta < 0
#ifndef AOA_T4_COMP_A1
#define AOA_T4_COMP_A1 0
#endif
#ifndef AOA_T4_COMP_B1
#define AOA_T4_COMP_B1 0
#endif
#ifndef AOA_T4_COMP_A2
#define AOA_T4_COMP_A2 -0.5457
#endif
#ifndef AOA_T4_COMP_B2
#define AOA_T4_COMP_B2 0
#endif

struct Aoa_T4 aoa_t4;
struct serial_act_t4_in myserial_act_t4_in_local;
static abi_event SERIAL_ACT_T4_IN;

struct FloatEulers eulers_t4;
float aoa_t4_a1 = AOA_T4_COMP_A1;
float aoa_t4_b1 = AOA_T4_COMP_B1;
float aoa_t4_a2 = AOA_T4_COMP_A2;
float aoa_t4_b2 = AOA_T4_COMP_B2;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_aoa(struct transport_tx *trans, struct link_device *dev)
{
    // raw value is not used
    uint32_t _raw = 0;
    // angle in rad
    pprz_msg_send_AOA(trans, dev, AC_ID, &_raw, &aoa_t4.angle, &aoa_t4.angle_raw);
}
#endif

static void serial_act_t4_abi_in(uint8_t sender_id __attribute__((unused)), struct serial_act_t4_in *myserial_act_t4_in_ptr, float *serial_act_t4_extra_data_in_ptr){
    memcpy(&myserial_act_t4_in_local,myserial_act_t4_in_ptr,sizeof(struct serial_act_t4_in));
//    memcpy(&serial_act_t4_extra_data_in_local,serial_act_t4_extra_data_in_ptr,255 * sizeof(float));

// TODO: servo ID
    int16_t angle_int = myserial_act_t4_in_ptr->servo_8_angle_int;
    aoa_t4.angle_raw = RadOfDeg(angle_int*0.01);
}

void aoa_t4_init_filters(void){
    init_butterworth_2_low_pass(&aoa_t4_lowpass_filter, AOA_T4_LOWPASS_TAU,
                              AOA_T4_FILTER_SAMPLING_TIME, 0);
}

void aoa_t4_init(void)
{
  aoa_t4.offset = AOA_T4_OFFSET;
  aoa_t4.angle = 0.0;
  aoa_t4.angle_raw = 0.0;
  aoa_t4.sign = AOA_T4_SIGN;

#if AOA_T4_USE_LOWPASS_FILTER
  aoa_t4_init_filters();
#endif

    AbiBindMsgSERIAL_ACT_T4_IN(ABI_BROADCAST, &SERIAL_ACT_T4_IN, serial_act_t4_abi_in);

#if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AOA, send_aoa);
#endif
}

void aoa_t4_update(void)
{
    aoa_t4.angle = (aoa_t4.sign*aoa_t4.angle_raw)+aoa_t4.offset;

#if AOA_T4_USE_COMPENSATION
    float_eulers_of_quat_zxy(&eulers_t4, stateGetNedToBodyQuat_f());
    // first order fn
    if (eulers_t4.theta >= 0) {
        aoa_t4.angle += aoa_t4_a1*eulers_t4.theta + aoa_t4_b1;
    } else {
        aoa_t4.angle += aoa_t4_a2*eulers_t4.theta + aoa_t4_b2;
    }
#endif

#if AOA_T4_USE_LOWPASS_FILTER
    aoa_t4.angle = update_butterworth_2_low_pass(&aoa_t4_lowpass_filter, aoa_t4.angle);
//#else
//    aoa_t4.angle = (aoa_t4.sign*aoa_t4.angle_raw)+aoa_t4.offset;
#endif

#ifdef USE_AOA
  uint8_t flag = 1;
  float foo = 0.f;
  AbiSendMsgINCIDENCE(AOA_T4_ID, flag, aoa_t4.angle, foo);
  stateSetAngleOfAttack_f(aoa_t4.angle);
#endif

//    pprz_msg_send_AOA(trans, dev, AC_ID, &aoa_t4.raw, &aoa_t4.angle);
}

