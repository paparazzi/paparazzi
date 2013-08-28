/*
 * Copyright (C) 2011 The Paparazzi Team
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

#include <inttypes.h>

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );
static inline void main_report(void);

static inline void on_gyro_event(void);
static inline void on_accel_event(void);
static inline void on_mag_event(void);

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  imu_init();
  ahrs_aligner_init();
  ahrs_init();

  DEBUG_SERVO1_INIT();
  //  DEBUG_SERVO2_INIT();

  mcu_int_enable();
}

static inline void main_periodic_task( void ) {

  if (sys_time.nb_sec > 1) imu_periodic();
  RunOnceEvery(10, { LED_PERIODIC();});
  main_report();
}

static inline void main_event_task( void ) {

  ImuEvent(on_gyro_event, on_accel_event, on_mag_event);

}

static inline void on_gyro_event(void) {
  ImuScaleGyro(imu);
  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
    DEBUG_S1_ON();
    ahrs_propagate();
    DEBUG_S1_OFF();
  }
}

static inline void on_accel_event(void) {
  ImuScaleAccel(imu);
  if (ahrs.status != AHRS_UNINIT) {
    DEBUG_S2_ON();
    ahrs_update_accel();
    DEBUG_S2_OFF();
  }
}

static inline void on_mag_event(void) {
  ImuScaleMag(imu);
  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag();
  }
}


static inline void main_report(void) {

  PeriodicPrescaleBy10(
		       {
			 DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
						     &imu.accel_unscaled.x,
						     &imu.accel_unscaled.y,
						     &imu.accel_unscaled.z);
		       },
		       {
			 DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
						    &imu.gyro_unscaled.p,
						    &imu.gyro_unscaled.q,
						    &imu.gyro_unscaled.r);
		       },
		       {
			 DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
						   &imu.mag_unscaled.x,
						   &imu.mag_unscaled.y,
						   &imu.mag_unscaled.z);
		       },
		       {
			 DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
						   &imu.accel.x,
						   &imu.accel.y,
						   &imu.accel.z);
		       },
		       {
			 DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
						  &imu.gyro.p,
						  &imu.gyro.q,
						  &imu.gyro.r);
		       },

		       {
			 DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel, DefaultDevice,
						 &imu.mag.x,
						 &imu.mag.y,
						 &imu.mag.z);
		       },

		       {
			 DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
		       },
		       {
#ifdef USE_I2C2
                 uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
                 uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
                 uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
                 uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
                 uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
                 uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
                 uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
                 uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
                 uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
                 uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
                 const uint8_t _bus2 = 2;
                 DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                                          &i2c2_queue_full_cnt,
                                          &i2c2_ack_fail_cnt,
                                          &i2c2_miss_start_stop_cnt,
                                          &i2c2_arb_lost_cnt,
                                          &i2c2_over_under_cnt,
                                          &i2c2_pec_recep_cnt,
                                          &i2c2_timeout_tlow_cnt,
                                          &i2c2_smbus_alert_cnt,
                                          &i2c2_unexpected_event_cnt,
                                          &i2c2_last_unexpected_event,
                                          &_bus2);
#endif
		       },
		       {
			 DOWNLINK_SEND_AHRS_EULER_INT(DefaultChannel, DefaultDevice,
							&ahrs.ltp_to_imu_euler.phi,
							&ahrs.ltp_to_imu_euler.theta,
							&ahrs.ltp_to_imu_euler.psi,
							&ahrs.ltp_to_body_euler.phi,
							&ahrs.ltp_to_body_euler.theta,
							&ahrs.ltp_to_body_euler.psi);
		       },
		       {
			 DOWNLINK_SEND_AHRS_GYRO_BIAS_INT(DefaultChannel, DefaultDevice,
						      &ahrs_impl.gyro_bias.p,
						      &ahrs_impl.gyro_bias.q,
						      &ahrs_impl.gyro_bias.r);

		       });
}
