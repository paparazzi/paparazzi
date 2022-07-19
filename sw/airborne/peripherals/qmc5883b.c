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
 * @file peripherals/qmc5883b.c
 *
 * Driver for QMC5883 magnetometer.
 * @todo DRDY/IRQ handling not implemented
 */

#include "generated/airframe.h"
#include "peripherals/qmc5883b.h"
#include "modules/core/abi.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

struct Qmc5883B qmc;

// default address 0D, needs to be shifted for R/W
#define QMC5883B_READ_ADDR 0x1B
#define QMC5883B_WRITE_ADDR 0x1A
#define QMC5883B_REG_ADDR 0x0B
#define QMC5883B_REG_VAL 0x01

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

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
void qmc5883b_init(void)
{
  qmc.trans.status = I2CTransDone;
  qmc.waddr = QMC5883B_WRITE_ADDR;
  qmc.raddr = QMC5883B_READ_ADDR;
  qmc.status = QMC5883B_INIT;
  qmc.set_mode = true;

  qmc.mag_data[0] = 0;
  qmc.mag_data[1] = 0;
  qmc.mag_data[2] = 0;

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
void qmc5883b_event(void)
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
 * QMC periodic function at 50 Hz
 */
void qmc5883b_periodic(void)
{
  switch (qmc.status) {
    case QMC5883B_INIT:
    qmc.status_debug = 0;
      if (qmc.trans.status == I2CTransDone) {
      // ask for i2c frame for set/reset period
      qmc.trans.buf[0] = QMC5883B_REG_ADDR;
      qmc.trans.buf[1] = QMC5883B_REG_VAL;

        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 2)){
          // transaction OK, increment status
          qmc.status = QMC5883B_SET_MODE;
        }
      }
      break;

    case QMC5883B_SET_MODE:
    qmc.status_debug = 1;
      if (qmc.trans.status == I2CTransDone && qmc.set_mode) {
      // ask for i2c frame for set/reset period
      qmc.trans.buf[0] = 0x10;
      qmc.trans.buf[1] = 0x0D;

        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 2)){
          // transaction OK, increment status
          qmc.status = QMC5883B_REQUEST;
          qmc.set_mode = false;
        }
      } else if (qmc.trans.status == I2CTransDone) {
        qmc.status = QMC5883B_REQUEST;
      }
      break;

    case QMC5883B_REQUEST:
    qmc.status_debug = 2;
      if (qmc.trans.status == I2CTransDone) {
        qmc.trans.buf[0] = 0x00; // sets register pointer to results register
        if (i2c_transmit(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.waddr, 1)){
          // transaction OK, increment status
          qmc.status = QMC5883B_ACQUIRE;
        }
      }
      break;

    case QMC5883B_ACQUIRE:
    qmc.status_debug = 3;
      if (qmc.trans.status == I2CTransDone) {
        // clear buffer
        qmc.trans.buf[0] = 0;
        qmc.trans.buf[1] = 0;
        if (i2c_receive(&MAG_QMC5883_I2C_DEV, &qmc.trans, qmc.raddr, 6)){
          // transaction OK, increment status
          qmc.status = QMC5883B_PARSE;
        }
      }
      break;

    case QMC5883B_PARSE: {
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
      qmc.status = QMC5883B_INIT;
      break;
    }
    default:
      break;
  }
}

