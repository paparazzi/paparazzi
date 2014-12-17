/*
 * Copyright (C) 2010 ENAC
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

/** \file ins_vn100.h
 *  \brief Interface for the VectorNav VN100 AHRS
 *  use the binary protocal on the SPI link
*/


#ifndef INS_VN100_H
#define INS_VN100_H

#include "std.h"
#include "VN100.h"
#include "math/pprz_algebra_float.h"

/* neutrals */
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
extern float ins_yaw_neutral;

/* state */
extern struct FloatEulers ins_eulers;
extern struct FloatQuat ins_quat;
extern struct FloatRates ins_rates;
extern struct FloatRMat ins_rmat;
extern struct FloatVect3 ins_accel;
extern struct FloatVect3 ins_mag;

extern volatile uint8_t ins_msg_received;

extern void vn100_init(void);
extern void vn100_periodic_task(void);
extern void vn100_event_task(void);
extern void vn100_report_task(void);

/* last received SPI packet */
extern VN100_Res_Packet last_received_packet;
/* last send packet */
extern VN100_Req_Packet last_send_packet;

/* output mode */
extern uint32_t ins_ador;
extern uint32_t ins_adof;
extern uint32_t ins_baud;

#ifndef VN100_ADOR
#define VN100_ADOR VN100_ADOR_OFF
#endif
#ifndef VN100_ADOF
#define VN100_ADOF VN100_ADOF_5HZ
#endif
#ifndef VN100_BAUD
#define VN100_BAUD VN100_Baud_57600
#endif

/* Init sequence */
extern uint8_t ins_init_status;
#define INS_VN100_SET_BAUD      0
#define INS_VN100_SET_ADOR      1
#define INS_VN100_SET_ADOF      2
#define INS_VN100_READY         3

/* Telemetry */
#define PERIODIC_SEND_AHRS(_chan, _dev) DOWNLINK_SEND_AHRS_LKF(_chan, _dev, \
    &ins_eulers.phi, &ins_eulers.theta, &ins_eulers.psi, \
    &ins_quat.qi, &ins_quat.qx, &ins_quat.qy, &ins_quat.qz, \
    &ins_rates.p, &ins_rates.q, &ins_rates.r, \
    &ins_accel.x, &ins_accel.y, &ins_accel.z, \
    &ins_mag.x, &ins_mag.y, &ins_mag.z)

#endif /* INS_VN100_H */
