/*
 * Copyright (C) 2011 Gautier Hattenberger
 * based on ArduIMU driver:
 *   Autoren@ZHAW:  schmiemi
 *                  chaneren
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

#include <stdbool.h>
#include "modules/ins/ins_arduimu_basic.h"
#include "mcu_periph/i2c.h"

// Estimator interface
#include "state.h"

// GPS data for ArduIMU
#include "subsystems/gps.h"

// Command vector for thrust
#include "generated/airframe.h"
#include "inter_mcu.h"

#define NB_DATA 9

#ifndef ARDUIMU_I2C_DEV
#define ARDUIMU_I2C_DEV i2c0
#endif

// Adresse des I2C Slaves:  0001 0110 letztes Bit ist für Read/Write
// einzugebende Adresse im ArduIMU ist 0000 1011
// da ArduIMU das Read/Write Bit selber anfügt.
#define ArduIMU_SLAVE_ADDR 0x22

#ifdef ARDUIMU_SYNC_SEND
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#endif

struct i2c_transaction ardu_gps_trans;
struct i2c_transaction ardu_ins_trans;

static int16_t recievedData[NB_DATA];

struct FloatEulers arduimu_eulers;
struct FloatRates arduimu_rates;
struct FloatVect3 arduimu_accel;

float ins_roll_neutral;
float ins_pitch_neutral;

// Ask the arduimu to recalibrate the gyros and accels neutrals
// After calibration, values are store in the arduimu eeprom
bool arduimu_calibrate_neutrals;

// High Accel Flag
#define HIGH_ACCEL_LOW_SPEED 15.0
#define HIGH_ACCEL_LOW_SPEED_RESUME 4.0 // Hysteresis
#define HIGH_ACCEL_HIGH_THRUST (0.8*MAX_PPRZ)
#define HIGH_ACCEL_HIGH_THRUST_RESUME (0.1*MAX_PPRZ) // Hysteresis
bool high_accel_done;
bool high_accel_flag;

void ArduIMU_init(void)
{
  FLOAT_EULERS_ZERO(arduimu_eulers);
  FLOAT_RATES_ZERO(arduimu_rates);
  FLOAT_VECT3_ZERO(arduimu_accel);

  ardu_ins_trans.status = I2CTransDone;
  ardu_gps_trans.status = I2CTransDone;

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  arduimu_calibrate_neutrals = false;

  high_accel_done = false;
  high_accel_flag = false;
}

#define FillBufWith32bit(_buf, _index, _value) {  \
    _buf[_index] = (uint8_t) (_value);              \
    _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
    _buf[_index+2] = (uint8_t) ((_value) >> 16);    \
    _buf[_index+3] = (uint8_t) ((_value) >> 24);    \
  }

void ArduIMU_periodicGPS(void)
{

  if (ardu_gps_trans.status != I2CTransDone) { return; }

#if USE_HIGH_ACCEL_FLAG
  // Test for high acceleration:
  //  - low speed
  //  - high thrust
  float speed = stateGetHorizontalSpeedNorm_f();
  if (speed < HIGH_ACCEL_LOW_SPEED && ap_state->commands[COMMAND_THROTTLE] > HIGH_ACCEL_HIGH_THRUST && !high_accel_done) {
    high_accel_flag = true;
  } else {
    high_accel_flag = false;
    if (speed > HIGH_ACCEL_LOW_SPEED && !high_accel_done) {
      high_accel_done = true; // After takeoff, don't use high accel before landing (GS small, Throttle small)
    }
    if (speed < HIGH_ACCEL_HIGH_THRUST_RESUME && ap_state->commands[COMMAND_THROTTLE] < HIGH_ACCEL_HIGH_THRUST_RESUME) {
      high_accel_done = false; // Activate high accel after landing
    }
  }
#endif

  FillBufWith32bit(ardu_gps_trans.buf, 0, (int32_t)gps.speed_3d); // speed 3D
  FillBufWith32bit(ardu_gps_trans.buf, 4, (int32_t)gps.gspeed);   // ground speed
  FillBufWith32bit(ardu_gps_trans.buf, 8, (int32_t)gps.course);   // course
  ardu_gps_trans.buf[12] = gps.fix;                               // status gps fix
  ardu_gps_trans.buf[13] = (uint8_t)arduimu_calibrate_neutrals;   // calibration flag
  ardu_gps_trans.buf[14] = (uint8_t)
                           high_accel_flag;              // high acceleration flag (disable accelerometers in the arduimu filter)
  i2c_transmit(&ARDUIMU_I2C_DEV, &ardu_gps_trans, ArduIMU_SLAVE_ADDR, 15);

  // Reset calibration flag
  if (arduimu_calibrate_neutrals) { arduimu_calibrate_neutrals = false; }
}

void ArduIMU_periodic(void)
{
  //Frequence defined in conf/modules/ins_arduimu.xml

  if (ardu_ins_trans.status == I2CTransDone) {
    i2c_receive(&ARDUIMU_I2C_DEV, &ardu_ins_trans, ArduIMU_SLAVE_ADDR, NB_DATA * 2);
  }

}

#include "math/pprz_algebra_int.h"
/*
   Buffer O:  Roll
   Buffer 1:  Pitch
   Buffer 2:  Yaw
   Buffer 3:  Gyro X
   Buffer 4:  Gyro Y
   Buffer 5:  Gyro Z
   Buffer 6:  Accel X
   Buffer 7:  Accel Y
   Buffer 8:  Accel Z
   */

void ArduIMU_event(void)
{
  // Handle INS I2C event
  if (ardu_ins_trans.status == I2CTransSuccess) {
    // received data from I2C transaction
    recievedData[0] = (ardu_ins_trans.buf[1] << 8) | ardu_ins_trans.buf[0];
    recievedData[1] = (ardu_ins_trans.buf[3] << 8) | ardu_ins_trans.buf[2];
    recievedData[2] = (ardu_ins_trans.buf[5] << 8) | ardu_ins_trans.buf[4];
    recievedData[3] = (ardu_ins_trans.buf[7] << 8) | ardu_ins_trans.buf[6];
    recievedData[4] = (ardu_ins_trans.buf[9] << 8) | ardu_ins_trans.buf[8];
    recievedData[5] = (ardu_ins_trans.buf[11] << 8) | ardu_ins_trans.buf[10];
    recievedData[6] = (ardu_ins_trans.buf[13] << 8) | ardu_ins_trans.buf[12];
    recievedData[7] = (ardu_ins_trans.buf[15] << 8) | ardu_ins_trans.buf[14];
    recievedData[8] = (ardu_ins_trans.buf[17] << 8) | ardu_ins_trans.buf[16];

    // Update ArduIMU data
    arduimu_eulers.phi = ANGLE_FLOAT_OF_BFP(recievedData[0]) - ins_roll_neutral;
    arduimu_eulers.theta = ANGLE_FLOAT_OF_BFP(recievedData[1]) - ins_pitch_neutral;
    arduimu_eulers.psi = ANGLE_FLOAT_OF_BFP(recievedData[2]);
    arduimu_rates.p = RATE_FLOAT_OF_BFP(recievedData[3]);
    arduimu_rates.q = RATE_FLOAT_OF_BFP(recievedData[4]);
    arduimu_rates.r = RATE_FLOAT_OF_BFP(recievedData[5]);
    arduimu_accel.x = ACCEL_FLOAT_OF_BFP(recievedData[6]);
    arduimu_accel.y = ACCEL_FLOAT_OF_BFP(recievedData[7]);
    arduimu_accel.z = ACCEL_FLOAT_OF_BFP(recievedData[8]);

    // Update estimator
    stateSetNedToBodyEulers_f(&arduimu_eulers);
    stateSetBodyRates_f(&arduimu_rates);
    stateSetAccelNed_f(&((struct NedCoor_f)arduimu_accel));
    ardu_ins_trans.status = I2CTransDone;

#ifdef ARDUIMU_SYNC_SEND
    // uint8_t arduimu_id = 102;
    //RunOnceEvery(15, DOWNLINK_SEND_AHRS_EULER(DefaultChannel, DefaultDevice, &arduimu_eulers.phi, &arduimu_eulers.theta, &arduimu_eulers.psi, &arduimu_id));
    RunOnceEvery(15, DOWNLINK_SEND_IMU_GYRO(DefaultChannel, DefaultDevice, &arduimu_rates.p, &arduimu_rates.q,
                                            &arduimu_rates.r));
    RunOnceEvery(15, DOWNLINK_SEND_IMU_ACCEL(DefaultChannel, DefaultDevice, &arduimu_accel.x, &arduimu_accel.y,
                 &arduimu_accel.z));
#endif
  } else if (ardu_ins_trans.status == I2CTransFailed) {
    ardu_ins_trans.status = I2CTransDone;
  }
  // Handle GPS I2C event
  if (ardu_gps_trans.status == I2CTransSuccess || ardu_gps_trans.status == I2CTransFailed) {
    ardu_gps_trans.status = I2CTransDone;
  }
}

void ahrs_update_gps(void)
{

}
