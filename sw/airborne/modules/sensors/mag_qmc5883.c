/*
 * Copyright (C) 2020 Paparazzi Team
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

/**
 * @file modules/sensors/mag_qmc5883.c
 *
 * Module for QST QMC5883 magnetometer, the DB vesion
 */

#include "modules/sensors/mag_qmc5883.h"
#include "modules/sensors/mag_qmc5883_regs.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

#if MODULE_QMC5883_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"

#ifndef QMC5883_CHAN_X
#define QMC5883_CHAN_X 0
#endif
#ifndef QMC5883_CHAN_Y
#define QMC5883_CHAN_Y 1
#endif
#ifndef QMC5883_CHAN_Z
#define QMC5883_CHAN_Z 2
#endif
#ifndef QMC5883_CHAN_X_SIGN
#define QMC5883_CHAN_X_SIGN +
#endif
#ifndef QMC5883_CHAN_Y_SIGN
#define QMC5883_CHAN_Y_SIGN +
#endif
#ifndef QMC5883_CHAN_Z_SIGN
#define QMC5883_CHAN_Z_SIGN +
#endif

#if defined QMC5883_MAG_TO_IMU_PHI && defined QMC5883_MAG_TO_IMU_THETA && defined QMC5883_MAG_TO_IMU_PSI
#define USE_MAG_TO_IMU 1
static struct Int32RMat mag_to_imu; // rotation from mag to imu frame
#else
#define USE_MAG_TO_IMU 0
#endif
#endif

struct Qmc5883 qmc;
struct Int32Vect3 mag;

#if PERIODIC_TELEMETRY

// static void mag_qmc5883_send_imu_mag_raw(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID,
//                             &qmc.mag_data[0],
//                             &qmc.mag_data[1],
//                             &qmc.mag_data[2]);
// }

static void mag_qmc5883_send_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_QMC5883_DEBUG(trans, dev, AC_ID,
                              &qmc.status_debug,
                              &qmc.trans_status,
                              &qmc.raw_mag_data[0],
                              &qmc.raw_mag_data[1],
                              &qmc.raw_mag_data[2],
                              &qmc.raw_mag_data[3],
                              &qmc.raw_mag_data[4],
                              &qmc.raw_mag_data[5],
                              &qmc.mag_data[0],
                              &qmc.mag_data[1],
                              &qmc.mag_data[2]);
}

#endif

/**
 * Initialization Function
 */
void mag_qmc5883_init(void)
{
  qmc.trans.status = I2CTransDone;
  qmc.waddr = QMC5883_WRITE_ADDR;
  qmc.raddr = QMC5883_READ_ADDR;
  qmc.status = QMC5883_INIT;
  qmc.set_mode = true;

  qmc.mag_data[0] = 0;
  qmc.mag_data[1] = 0;
  qmc.mag_data[2] = 0;

  #if MODULE_QMC5883_UPDATE_AHRS && USE_MAG_TO_IMU
    struct Int32Eulers mag_to_imu_eulers = {
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_PHI),
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_THETA),
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_PSI)
    };
    int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);
  #endif

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_QMC5883_DEBUG, mag_qmc5883_send_debug);
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, mag_qmc5883_send_imu_mag_raw);
  #endif
}

/**
 * Magneto event function
 * Basically just check the progress of the transation
 * to prevent overruns during high speed operation
 * (ie. polling the magneto at >100Hz)
 */
void mag_qmc5883_event(void)
{
  switch (qmc.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      qmc.trans.status = I2CTransDone;
      break;
    case I2CTransFailed:
      // set to done
      qmc.trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}

/**
 * QMC periodic function
 */
void mag_qmc5883_periodic(void)
{
  switch (qmc.status) {
    case QMC5883_INIT:
    qmc.status_debug = 0;
      if (qmc.trans.status == I2CTransDone) {
      // ask for i2c frame for set/reset period
      qmc.trans.buf[0] = QMC5883_REG_RESET_PERIOD;
      qmc.trans.buf[1] = 0x01;

        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 2)){
          // transaction OK, increment status
          qmc.status = QMC5883_SET_MODE;
        }
      }
      break;

    case QMC5883_SET_MODE:
    qmc.status_debug = 1;
      if (qmc.trans.status == I2CTransDone && qmc.set_mode) {
      // ask for i2c frame for set/reset period
      qmc.trans.buf[0] = QMC5883_REG_CONTROL_1;
      qmc.trans.buf[1] = (QMC5883_MODE_CONT|QMC5883_ODR_200|QMC5883_RNG_8G|QMC5883_OSR_512);

        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 2)){
          // transaction OK, increment status
          qmc.status = QMC5883_REQUEST;
          qmc.set_mode = false;
        }
      } else if (qmc.trans.status == I2CTransDone) {
        qmc.status = QMC5883_REQUEST;
      }
      break;

    case QMC5883_REQUEST:
    qmc.status_debug = 2;
      if (qmc.trans.status == I2CTransDone) {
        qmc.trans.buf[0] = QMC5883_REG_DATXL; // sets register pointer to results register
        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 1)){
          // transaction OK, increment status
          qmc.status = QMC5883_ACQUIRE;
        }
      }
      break;

    case QMC5883_ACQUIRE:
    qmc.status_debug = 3;
      if (qmc.trans.status == I2CTransDone) {
        // clear buffer
        qmc.trans.buf[0] = 0;
        qmc.trans.buf[1] = 0;
        if (i2c_receive(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.raddr, 6)){
          // transaction OK, increment status
          qmc.status = QMC5883_PARSE;
        }
      }
      break;

    case QMC5883_PARSE: {
      qmc.status_debug = 4;

      // get raw data for debug
      qmc.raw_mag_data[0] = (uint8_t)(qmc.trans.buf[0]);
      qmc.raw_mag_data[1] = (uint8_t)(qmc.trans.buf[1]);
      qmc.raw_mag_data[2] = (uint8_t)(qmc.trans.buf[2]);
      qmc.raw_mag_data[3] = (uint8_t)(qmc.trans.buf[3]);
      qmc.raw_mag_data[4] = (uint8_t)(qmc.trans.buf[4]);
      qmc.raw_mag_data[5] = (uint8_t)(qmc.trans.buf[5]);
      
      // get xyz (first data, then sign)
      qmc.mag_data[0] = (int16_t)((qmc.trans.buf[0] | qmc.trans.buf[1] << 8));
      qmc.mag_data[1] = (int16_t)((qmc.trans.buf[2] | qmc.trans.buf[3] << 8));
      qmc.mag_data[2] = (int16_t)((qmc.trans.buf[4] | qmc.trans.buf[5] << 8));

      // mag.x = QMC5883_CHAN_X_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_X]);
      // mag.y = QMC5883_CHAN_Y_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Y]);
      // mag.z = QMC5883_CHAN_Z_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Z]);

      //   if (mag_qmc5883.data_available) {
      // #if MODULE_QMC5883_UPDATE_AHRS
      //     // current timestamp
      //     uint32_t now_ts = get_sys_time_usec();

      //     // set channel order
      //     // mag.x = QMC5883_CHAN_X_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_X]);
      //     // mag.y = QMC5883_CHAN_Y_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Y]);
      //     // mag.z = QMC5883_CHAN_Z_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Z]);

      //     // only rotate if needed
      // #if USE_MAG_TO_IMU
      //     struct Int32Vect3 imu_mag;
      //     // rotate data from mag frame to imu frame
      //     int32_rmat_vmult(&imu_mag, &mag_to_imu, &mag);
      //     // unscaled vector
      //     VECT3_COPY(imu.mag_unscaled, imu_mag);
      // #else
      //     // unscaled vector
      //     VECT3_COPY(imu.mag_unscaled, mag);
      // #endif
      //     // scale vector
      //     imu_scale_mag(&imu);

      //     AbiSendMsgIMU_MAG_INT32(MAG_QMC5883_SENDER_ID, now_ts, &imu.mag);
      // #endif
      // #if MODULE_QMC5883_UPDATE_AHRS ||  MODULE_QMC5883_SYNC_SEND
      //     mag_qmc5883.data_available = false;
      // #endif
      //   }

      qmc.status = QMC5883_INIT;
      break;
    }
    default:
      break;
  }
}
