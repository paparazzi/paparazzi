/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "mcu.h"
#include "mcu_periph/uart.h"
#include "sys_time.h"
#include "downlink.h"
#include "booz/booz2_commands.h"
#include "actuators.h"
#include "actuators/actuators_pwm.h"
#include "subsystems/imu.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "subsystems/ins.h"
#include "guidance.h"
#include "navigation.h"
#include "lisa/lisa_overo_link.h"
#include "generated/airframe.h"
#include "subsystems/ahrs.h"
#ifdef PASSTHROUGH_CYGNUS
#include "stabilization.h"
#endif

#include "stm32/can.h"
#include "csc_msg_def.h"
#include "csc_protocol.h"

#include "subsystems/sensors/baro.h"

#include "mcu_periph/adc.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static inline void on_gyro_accel_event(void);
static inline void on_accel_event(void);
static inline void on_mag_event(void);

static inline void on_overo_link_msg_received(void);
static inline void on_overo_link_lost(void);
static inline void on_overo_link_crc_failed(void);

static inline void on_rc_message(void);
static inline void on_vane_msg(void *data);

static bool_t new_radio_msg;
static bool_t new_baro_diff;
static bool_t new_baro_abs;
static bool_t new_vane;
static bool_t new_adc;


struct CscVaneMsg csc_vane_msg;

static struct adc_buf adc0_buf;
static struct adc_buf adc1_buf;
static struct adc_buf adc2_buf;
static struct adc_buf adc3_buf;

extern uint8_t adc_new_data_trigger;

struct CscServoCmd csc_servo_cmd;

#define ActuatorsCommit() actuators_pwm_commit();
#define actuators actuators_pwm_values

#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(i) actuators[i]
#define SERVOS_TICS_OF_USEC(_s) (_s)

int main(void) {

	main_init();

	while (1) {
		if (sys_time_periodic())
			main_periodic();
		main_event();
	}

	return 0;
}

static inline void main_init(void) {

	mcu_init();
	sys_time_init();
	imu_init();
	baro_init();
	radio_control_init();
	actuators_init();
	overo_link_init();
	cscp_init();

#ifdef PASSTHROUGH_CYGNUS
	autopilot_init();
	nav_init();
	guidance_h_init();
	guidance_v_init();
	stabilization_init();

	ahrs_aligner_init();
	ahrs_init();

	ins_init();
#endif

	adc_buf_channel(0, &adc0_buf, 8);
	adc_buf_channel(1, &adc1_buf, 8);
	adc_buf_channel(2, &adc2_buf, 8);
	adc_buf_channel(3, &adc3_buf, 8);

	cscp_register_callback(CSC_VANE_MSG_ID, on_vane_msg, (void *)&csc_vane_msg);
	new_radio_msg = FALSE;
	new_baro_diff = FALSE;
	new_baro_abs = FALSE;
	new_vane = FALSE;
	new_adc = FALSE;

	overo_link.up.msg.imu_tick = 0;
}

static void check_radio_lost(void)
{
#ifdef PASSTHROUGH_CYGNUS
	return;
#endif
	if (radio_control.status == RC_REALLY_LOST) {
		const static int32_t commands_failsafe[COMMANDS_NB] = COMMANDS_FAILSAFE;
		SetActuatorsFromCommands(commands_failsafe);
	}
}

static inline void main_periodic(void) {
	uint16_t v1 = 123;
	uint16_t v2 = 123;

  imu_periodic();
#ifdef PASSTHROUGH_CYGNUS
	autopilot_periodic();
#endif
	OveroLinkPeriodic(on_overo_link_lost);

	RunOnceEvery(10, {
			LED_PERIODIC();
			DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
			radio_control_periodic();
			check_radio_lost();
			DOWNLINK_SEND_BARO_RAW(DefaultChannel, &baro.absolute, &baro.differential);
		});

	RunOnceEvery(2, {baro_periodic();});

	if (adc_new_data_trigger) {
		adc_new_data_trigger = 0;
		new_adc = 1;
		v1 = adc0_buf.sum / adc0_buf.av_nb_sample;
		v2 = adc1_buf.values[0];

		RunOnceEvery(10, { DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &v1, &v2) });
	}
}

static inline void on_rc_message(void) {
	new_radio_msg = TRUE;
	if (radio_control.values[RADIO_MODE] >= 150) {
#ifdef PASSTHROUGH_CYGNUS
		autopilot_on_rc_frame();
#else
		static int32_t commands[COMMANDS_NB];
		SetCommandsFromRC(commands, radio_control.values);
		SetActuatorsFromCommands(commands);
#endif
	}
#ifndef PASSTHROUGH_CYGNUS
	if (radio_control.values[RADIO_KILL] > 150) {
		actuators[SERVO_THROTTLE] = SERVO_THROTTLE_MIN;
		ActuatorsCommit();
	}
#endif
}

static inline void on_overo_link_msg_received(void) {

	/* IMU up */
	overo_link.up.msg.valid.imu = 1;
	RATES_COPY(overo_link.up.msg.gyro, imu.gyro);
	VECT3_COPY(overo_link.up.msg.accel, imu.accel);
	VECT3_COPY(overo_link.up.msg.mag, imu.mag);

	/* RC up */
	overo_link.up.msg.valid.rc  = new_radio_msg;
	new_radio_msg = FALSE;

	overo_link.up.msg.rc_pitch  = radio_control.values[RADIO_PITCH];
	overo_link.up.msg.rc_roll   = radio_control.values[RADIO_ROLL];
	overo_link.up.msg.rc_yaw    = radio_control.values[RADIO_YAW];
	overo_link.up.msg.rc_thrust = radio_control.values[RADIO_THROTTLE];
	overo_link.up.msg.rc_mode   = radio_control.values[RADIO_MODE];
#ifdef RADIO_CONTROL_KILL
	overo_link.up.msg.rc_kill   = radio_control.values[RADIO_KILL];
#endif
#ifdef RADIO_CONTROL_GEAR
	overo_link.up.msg.rc_gear   = radio_control.values[RADIO_GEAR];
#endif

	overo_link.up.msg.rc_aux2   = radio_control.values[RADIO_AUX2];
	overo_link.up.msg.rc_aux3   = radio_control.values[RADIO_AUX3];
	overo_link.up.msg.rc_status = radio_control.status;

	overo_link.up.msg.stm_msg_cnt     = overo_link.msg_cnt;
	overo_link.up.msg.stm_crc_err_cnt = overo_link.crc_err_cnt;

	/* baro up */
	overo_link.up.msg.valid.pressure_differential = new_baro_diff;
	overo_link.up.msg.valid.pressure_absolute     = new_baro_abs;
	new_baro_diff = FALSE;
	new_baro_abs  = FALSE;
	overo_link.up.msg.pressure_differential = baro.differential;
	overo_link.up.msg.pressure_absolute     = baro.absolute;

	/* vane up */
	overo_link.up.msg.valid.vane =  new_vane;
	new_vane = FALSE;
	overo_link.up.msg.vane_angle1 = csc_vane_msg.vane_angle1;
	overo_link.up.msg.vane_angle2 = csc_vane_msg.vane_angle2;

	/* adc up */
	overo_link.up.msg.adc.channels[0] = adc0_buf.sum / adc0_buf.av_nb_sample;
	overo_link.up.msg.adc.channels[1] = adc1_buf.sum / adc1_buf.av_nb_sample;
	overo_link.up.msg.adc.channels[2] = adc2_buf.sum / adc2_buf.av_nb_sample;
	overo_link.up.msg.adc.channels[3] = adc3_buf.sum / adc3_buf.av_nb_sample;
	overo_link.up.msg.valid.adc =  new_adc;
	new_adc = FALSE;

#ifdef PASSTHROUGH_CYGNUS
	for (int i = 0; i < 6; i++) {
		actuators_pwm_values[i] = overo_link.down.msg.pwm_outputs_usecs[i];
	}
	actuators_pwm_commit();

	for (int i = 6; i < 10; i++) {
		csc_servo_cmd.servos[i-6] = overo_link.down.msg.pwm_outputs_usecs[i];
	}
	cscp_transmit(0, CSC_SERVO_CMD_ID, (uint8_t *)&csc_servo_cmd, sizeof(csc_servo_cmd));
#else
	/* pwm acuators down */
	if (radio_control.values[RADIO_MODE] <= 150) {
		for (int i = 0; i < 6; i++) {
			actuators_pwm_values[i] = overo_link.down.msg.pwm_outputs_usecs[i];
		}
		if (radio_control.values[RADIO_KILL] > 150) {
			actuators[SERVO_THROTTLE] = SERVO_THROTTLE_MIN;
		}
		actuators_pwm_commit();
	}
#endif
}

static inline void on_overo_link_lost(void) {
}

static inline void on_overo_link_crc_failed(void) {
}

static inline void on_accel_event(void) {
}

static inline void on_gyro_accel_event(void) {
	ImuScaleGyro(imu);
	ImuScaleAccel(imu);
	overo_link.up.msg.imu_tick++;

#ifdef PASSTHROUGH_CYGNUS
	if (ahrs.status == AHRS_UNINIT) {
		ahrs_aligner_run();
		if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
			ahrs_align();
	} else {
		ahrs_propagate();
		ahrs_update_accel();
		ins_propagate();
	}
#endif
}

static inline void on_mag_event(void) {
	ImuScaleMag(imu);

#ifdef PASSTHROUGH_CYGNUS
	if (ahrs.status == AHRS_RUNNING)
		ahrs_update_mag();
#endif
}

static inline void on_vane_msg(void *data) {
	new_vane = TRUE;
	int zero = 0;
	DOWNLINK_SEND_VANE_SENSOR(DefaultChannel,
				&(csc_vane_msg.vane_angle1),
				&zero,
				&zero,
				&zero,
				&zero,
				&csc_vane_msg.vane_angle2,
				&zero,
				&zero,
				&zero,
				&zero);
}

static inline void main_on_baro_diff(void) {
	new_baro_diff = TRUE;
}

static inline void main_on_baro_abs(void) {
	new_baro_abs = TRUE;
}

static inline void main_event(void) {

	ImuEvent(on_gyro_accel_event, on_accel_event, on_mag_event);
	BaroEvent(main_on_baro_abs, main_on_baro_diff);
	OveroLinkEvent(on_overo_link_msg_received, on_overo_link_crc_failed);
	RadioControlEvent(on_rc_message);
	cscp_event();
}
