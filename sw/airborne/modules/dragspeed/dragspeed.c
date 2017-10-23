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

#ifndef DRAGSPEED_ZERO_X
#define DRAGSPEED_ZERO_X 0.0 /// Accelerometer reading (x axis) when stationary [m/s^2]
#endif

#ifndef DRAGSPEED_ZERO_Y
#define DRAGSPEED_ZERO_Y 0.0 /// Accelerometer reading (y axis) when stationary [m/s^2]
#endif

#ifndef DRAGSPEED_R
#define DRAGSPEED_R 0.25 /// Velocity measurement noise variance [(m/s)^2]
#endif

#ifndef DRAGSPEED_FILTER
#define DRAGSPEED_FILTER 0.8 /// First-order low-pass filter strength [0..1]. Pretty high default value as accelero's tend to be noisy.
#endif

struct dragspeed_t dragspeed;

static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);

static void send_dragspeed(struct transport_tx *trans, struct link_device *dev);

static void calibrate_coeff(struct Int32Vect3 *mag);
static void calibrate_zero(struct Int32Vect3 *mag);

void dragspeed_init(void) {
	// Set initial values
	dragspeed.coeff = DRAGSPEED_COEFF;
	dragspeed.filter = DRAGSPEED_FILTER;
	// Register callbacks
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DRAGSPEED,
			send_dragspeed);
	AbiBindMsgIMU_ACCEL_INT32(DRAGSPEED_ACCEL_ID, &accel_ev, accel_cb);
}

bool dragspeed_calibrate_coeff(void) {
	dragspeed.do_calibrate_coeff = 1;
	return FALSE;
}

bool dragspeed_calibrate_zero(void) {
	dragspeed.do_calibrate_zero = 1;
	return FALSE;
}

bool dragspeed_is_calibrating(void) {
	return dragspeed.do_calibrate_coeff || dragspeed.do_calibrate_zero;
}

static void accel_cb(
		uint8_t sender_id __attribute__((unused)),
		uint32_t stamp,
		struct Int32Vect3 *mag) {
	// Estimate current velocity
	float vx = -(ACCEL_FLOAT_OF_BFP(mag->x) - dragspeed.zero.x)
			/ dragspeed.coeff;
	float vy = -(ACCEL_FLOAT_OF_BFP(mag->y) - dragspeed.zero.y)
			/ dragspeed.coeff;
	// Simple low-pass filter
	dragspeed.vel.x += (1 - dragspeed.filter) * (vx - dragspeed.vel.x);
	dragspeed.vel.y += (1 - dragspeed.filter) * (vy - dragspeed.vel.y);
	// Send as ABI VELOCITY_ESTIMATE message
	// Note: set VEL_DRAGSPEED_ID to ABI_DISABLE to disable
#if VEL_DRAGSPEED_ID
	if (!dragspeed.do_calibrate_coeff && !dragspeed.do_calibrate_zero) {
		AbiSendMsgVELOCITY_ESTIMATE(VEL_DRAGSPEED_ID, stamp,
				dragspeed.vel.x, dragspeed.vel.y, 0, DRAGSPEED_R);
	}
#endif
	// Perform calibration if required
	calibrate_coeff(mag);
	calibrate_zero(mag);
}

/**
 * Calibrate drag coefficient by comparing accelerometer measurements to INS
 * velocities.
 * Should be performed with VEL_DRAGSPEED_ID set to ABI_DISABLE, but for safety
 * the ABI messages are also disabled automatically when calibration is active.
 *
 * This routine assumes that the accelerometers have been zeroed beforehand.
 */
static void calibrate_coeff(struct Int32Vect3 *mag) {
	// Reset when new calibration is started
	static int do_calibrate_prev = 0;
	static float coeff = 0;
	static int num_samples = 0;
	if (dragspeed.do_calibrate_coeff && !do_calibrate_prev) {
		coeff = 0;
		num_samples = 0;
	}
	do_calibrate_prev = dragspeed.do_calibrate_coeff;
	// Return when calibration is not active
	if (!dragspeed.do_calibrate_coeff) {
		return;
	}

	// Average required coefficients when velocity is sufficiently high
	struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
	float ins_speed = sqrt(vel_ins->x * vel_ins->x + vel_ins->y * vel_ins->y);
	float accel_magn = sqrt(
			ACCEL_FLOAT_OF_BFP(mag->x) * ACCEL_FLOAT_OF_BFP(mag->x) +
			ACCEL_FLOAT_OF_BFP(mag->y) * ACCEL_FLOAT_OF_BFP(mag->y));
	if (ins_speed > 0.5 && accel_magn != 0) {
		float this_coeff = accel_magn / ins_speed;
		coeff = (coeff * num_samples + this_coeff) / (num_samples + 1);
		num_samples++;
		// End calibration when enough samples are averaged
		if (num_samples > 1000 && coeff != 0) {
			dragspeed.coeff = coeff;
			dragspeed.do_calibrate_coeff = 0;
		}
	}
}

/**
 * Calibrate zero velocity by measuring the accelerations while the drone
 * hovers in-place.
 *
 * The zero-velocity readings can change between flights, e.g. because of small
 * differences in battery or outer hull positions.
 */
static void calibrate_zero(struct Int32Vect3 *mag) {
	// Reset when new calibration is started
	static int do_calibrate_prev = 0;
	static struct FloatVect2 zero;
	static int num_samples = 0;
	if (dragspeed.do_calibrate_zero && !do_calibrate_prev) {
		zero.x = 0;
		zero.y = 0;
		num_samples = 0;
	}
	do_calibrate_prev = dragspeed.do_calibrate_zero;
	// Return when calibration is not active
	if (!dragspeed.do_calibrate_zero) {
		return;
	}

	// Average accelerometer readings when velocity is sufficiently low
	struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
	float ins_speed = sqrt(vel_ins->x * vel_ins->x + vel_ins->y * vel_ins->y);
	if (ins_speed < 0.1) {
		zero.x = (zero.x * num_samples + ACCEL_FLOAT_OF_BFP(mag->x))
				/ (num_samples + 1);
		zero.y = (zero.y * num_samples + ACCEL_FLOAT_OF_BFP(mag->y))
				/ (num_samples + 1);
		num_samples++;
		// End calibration when enough samples are averaged
		if (num_samples > 1000) {
			dragspeed.zero = zero;
			dragspeed.do_calibrate_zero = 0;
		}
	}
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

