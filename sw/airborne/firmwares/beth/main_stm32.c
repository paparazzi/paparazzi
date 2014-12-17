/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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


#include BOARD_CONFIG
#include "std.h"
#include "mcu.h"
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/commands.h"
#include "firmwares/rotorcraft/actuators.h"
//#include "booz/booz_radio_control.h"
#include "subsystems/imu.h"
#include "lisa/lisa_overo_link.h"
#include "bench_sensors.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static inline void on_gyro_accel_event(void);
static inline void on_accel_event(void);
static inline void on_mag_event(void);

static inline void main_on_overo_msg_received(void);
static inline void main_on_overo_link_lost(void);
static inline void main_on_overo_link_error(void);

static uint32_t spi_msg_cnt = 0;
static uint16_t spi_errors = 0;

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  actuators_init();
  //radio_control_init();
  imu_init();
  overo_link_init();
  bench_sensors_init();
  commands[COMMAND_ROLL] = 0;
  commands[COMMAND_YAW] = 0;
}

#define PITCH_MAGIC_NUMBER (121)

static inline void main_periodic(void)
{
  int8_t pitch_out, thrust_out;
  imu_periodic();

  OveroLinkPeriodic(main_on_overo_link_lost)

  RunOnceEvery(10, {LED_PERIODIC(); DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  //RunOnceEvery(5, {DOWNLINK_SEND_BETH(DefaultChannel, &bench_sensors.angle_1,
  //  &bench_sensors.angle_2,&bench_sensors.angle_3, &bench_sensors.current);});

  //RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &overo_link.down.msg.foo,&overo_link.down.msg.bar);});

  read_bench_sensors();

  pitch_out = (int8_t)((0xFF) & overo_link.down.msg.pitch);
  thrust_out = (int8_t)((0xFF) & overo_link.down.msg.thrust);

  Bound(pitch_out, -80, 80);
  Bound(thrust_out, 0, 100);

  overo_link.up.msg.thrust_out = thrust_out;
  overo_link.up.msg.pitch_out = pitch_out;

  //stop the motors if we've lost SPI or CAN link
  //If SPI has had CRC error we don't stop motors
  if ((spi_msg_cnt == 0) || (can_err_flags != 0)) {
    commands[COMMAND_PITCH] = 0;
    commands[COMMAND_THRUST] = 0;
    actuators_set(FALSE);
    overo_link.up.msg.can_errs = can_err_flags;
    overo_link.up.msg.pitch_out = PITCH_MAGIC_NUMBER;
  } else {
    commands[COMMAND_PITCH] = pitch_out;
    commands[COMMAND_THRUST] = thrust_out;
    actuators_set(TRUE);
  }
}

static inline void main_event(void)
{
  ImuEvent(on_gyro_accel_event, on_accel_event, on_mag_event);
  OveroLinkEvent(main_on_overo_msg_received, main_on_overo_link_error);
}

static inline void main_on_overo_msg_received(void)
{

  overo_link.up.msg.bench_sensor.x = bench_sensors.angle_1;
  overo_link.up.msg.bench_sensor.y = bench_sensors.angle_2;
  overo_link.up.msg.bench_sensor.z = bench_sensors.angle_3;

  overo_link.up.msg.accel.x = imu.accel_unscaled.x;
  overo_link.up.msg.accel.y = imu.accel_unscaled.y;
  overo_link.up.msg.accel.z = imu.accel_unscaled.z;

  overo_link.up.msg.gyro.p = imu.gyro_unscaled.p;
  overo_link.up.msg.gyro.q = imu.gyro_unscaled.q;
  overo_link.up.msg.gyro.r = imu.gyro_unscaled.r;

  //can_err_flags (uint16) represents the board number that is not communicating regularly
  //spi_errors (uint16) reflects the number of crc errors on the spi link
  //TODO: if >10% of messages are coming in with crc errors, assume something is really wrong
  //and disable the motors.
  overo_link.up.msg.can_errs = can_err_flags;
  overo_link.up.msg.spi_errs = spi_errors;

  //spi_msg_cnt shows number of spi transfers since last link lost event
  overo_link.up.msg.cnt = spi_msg_cnt++;

}

static inline void main_on_overo_link_lost(void)
{
  //actuators_set(FALSE);
  spi_msg_cnt = 0;
}


static inline void on_accel_event(void)
{

}

static inline void on_gyro_accel_event(void)
{
  imu_scale_gyro(&imu);
  imu_scale_accel(&imu);

  //LED_TOGGLE(2);
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) { cnt = 0; }

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,
                               &imu.gyro_unscaled.p,
                               &imu.gyro_unscaled.q,
                               &imu.gyro_unscaled.r);

    DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,
                                &imu.accel_unscaled.x,
                                &imu.accel_unscaled.y,
                                &imu.accel_unscaled.z);
  } else if (cnt == 7) {
    DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel,
                                  &imu.gyro.p,
                                  &imu.gyro.q,
                                  &imu.gyro.r);

    DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel,
                                   &imu.accel.x,
                                   &imu.accel.y,
                                   &imu.accel.z);
  }
}


static inline void on_mag_event(void)
{
  imu_scale_mag(&imu);
  static uint8_t cnt;
  cnt++;
  if (cnt > 1) { cnt = 0; }

  if (cnt % 2) {
    DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel,
                                 &imu.mag.x,
                                 &imu.mag.y,
                                 &imu.mag.z);
  } else {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,
                              &imu.mag_unscaled.x,
                              &imu.mag_unscaled.y,
                              &imu.mag_unscaled.z);
  }
}

static inline void main_on_overo_link_error(void)
{
  spi_errors++;
}
