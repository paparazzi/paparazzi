/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/dragspeed/dragspeed.c"
 * @author Tom van Dijk
 * This module estimates the velocity of rotorcraft by measuring the drag force using the accelerometer.
 */

#include "modules/dragspeed/dragspeed.h"

#include "subsystems/abi.h"
#include "subsystems/abi_common.h"
#include "subsystems/datalink/telemetry.h"

#ifndef DRAGSPEED_ACCEL_ID
#define DRAGSPEED_ACCEL_ID ABI_BROADCAST
#endif

#ifndef DRAGSPEED_COEFF
#define DRAGSPEED_COEFF 1.0 /// Drag coefficient (mu/m) of the linear drag model, where m*a = v*mu
#endif

#ifndef DRAGSPEED_R
#define DRAGSPEED_R 0.25 /// Measurement noise variance [(m/s)^2]
#endif

static struct dragspeed_t {
	struct FloatVect2 vel;
} dragspeed;

static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);

static void send_dragspeed(struct transport_tx *trans, struct link_device *dev);


void dragspeed_init(void) {
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DRAGSPEED,
			send_dragspeed);
	AbiBindMsgIMU_ACCEL_INT32(DRAGSPEED_ACCEL_ID, &accel_ev, accel_cb);
}

static void accel_cb(
		uint8_t sender_id __attribute__((unused)),
		uint32_t stamp,
		struct Int32Vect3 *mag) {
	// Estimate current velocity
	dragspeed.vel.x = ACCEL_FLOAT_OF_BFP(mag->x) / DRAGSPEED_COEFF;
	dragspeed.vel.y = ACCEL_FLOAT_OF_BFP(mag->y) / DRAGSPEED_COEFF;
	// Send as ABI VELOCITY_ESTIMATE message
	// Note: set VEL_DRAGSPEED_ID to ABI_DISABLE to disable
#if VEL_DRAGSPEED_ID
	AbiSendMsgVELOCITY_ESTIMATE(VEL_DRAGSPEED_ID, stamp, dragspeed.vel.x,
			dragspeed.vel.y, 0, DRAGSPEED_R);
#endif
}

static void send_dragspeed(struct transport_tx *trans, struct link_device *dev) {
	// Calculate INS velocity in body frame
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
	struct FloatVect2 vel_ins_body = {
			cos(att->psi) * vel_ins->y + sin(att->psi) * vel_ins->x,
			-sin(att->psi) * vel_ins->y + cos(att->psi) * vel_ins->x
	};
	// Send telemetry message
	pprz_msg_send_DRAGSPEED(trans, dev, AC_ID,
			&dragspeed.vel.x, &dragspeed.vel.y,
			&vel_ins_body.x, &vel_ins_body.y);
}


