/*
 * Copyright (C) 2022 Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/qmc5883l.c
 *
 * QST QMC5883L 3-axis magnetometer driver interface (I2C).
 */

#include "peripherals/qmc5883l.h"

/* Registers Axis X,Y,Z */
#define QMC5883L_REG_DATXL  0x00
#define QMC5883L_REG_DATXM  0x01
#define QMC5883L_REG_DATYL  0x02
#define QMC5883L_REG_DATYM  0x03
#define QMC5883L_REG_DATZL  0x04
#define QMC5883L_REG_DATZM  0x05

/* Register I2C bus transaction Status */
#define QMC5883L_REG_STATUS 0x06

/* Registers Temperature, relative thus not so useful ATM, therefore not implemented in reading */
#define QMC5883L_REG_TEMPM  0x07
#define QMC5883L_REG_TEMPL  0x08 

/* Registers Config */
#define QMC5883L_REG_CONTROL_1    0x09  /* settings for MODE */
#define QMC5883L_REG_CONTROL_2    0x0A  /* settings for INT_ENB */
#define QMC5883L_REG_RESET_PERIOD 0x0B

#define QMC5883L_REG_IDC    0x0C  /* OEM reserved */
#define QMC5883L_REG_IDD    0x0D  /* OEM reserved */

/* Options for CONTROL_1 */
#define QMC5883L_MODE_STBY 0x00
#define QMC5883L_MODE_CONT 0x01

/* Options for scale RaNGe(RNG) Gauss */
#define QMC5883L_RNG_2G 0x00
#define QMC5883L_RNG_8G 0x10

/* options for Over-Sample Ratio (OSR) */
#define QMC5883L_OSR_512 0x00  /* Use 512 if powerusage of chip is not an issue */
#define QMC5883L_OSR_256 0x40
#define QMC5883L_OSR_128 0x80
#define QMC5883L_OSR_64  0xC0

void qmc5883l_init(struct Qmc5883l *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate)
{
  /* set i2c_peripheral */
  mag->i2c_p = i2c_p;
  /* set i2c address */
  mag->i2c_trans.slave_addr = addr;
  mag->i2c_trans.status = I2CTransDone;
  /* store data rate */
  mag->data_rate = data_rate;
  mag->initialized = false;
  mag->status = QMC5883L_CONF_UNINIT;
  mag->data_available = false;
}

void qmc5883l_configure(struct Qmc5883l *mag)
{
  // Only configure when not busy
  if (mag->i2c_trans.status != I2CTransSuccess && mag->i2c_trans.status != I2CTransFailed
      && mag->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when successful continue with next
  if (mag->i2c_trans.status == I2CTransSuccess) {
    mag->status++; //Here the Enum Counter goes to the next one
  }

  mag->i2c_trans.status = I2CTransDone;
  switch (mag->status) {

    case QMC5883L_CONF_UNINIT:
      /* prepare config request */
      mag->i2c_trans.buf[0] = QMC5883L_REG_RESET_PERIOD;
      mag->i2c_trans.buf[0] = 0x01;
      /* send config request, ask for i2c frame for set/reset period */
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_CCR_DONE:
      mag->i2c_trans.buf[0] = QMC5883L_REG_CONTROL_1;
      mag->i2c_trans.buf[1] = (QMC5883L_MODE_CONT|QMC5883L_ODR_200|QMC5883L_RNG_8G|QMC5883L_OSR_512);//todo datarate from settings mag->data_rate; 
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_TMRC_DONE:
      mag->i2c_trans.buf[0] = QMC5883L_REG_CONTROL_1;
      mag->i2c_trans.buf[1] = (QMC5883L_MODE_CONT|QMC5883L_ODR_200|QMC5883L_RNG_8G|QMC5883L_OSR_512);//todo datarate from settings //mag->data_rate;
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case QMC5883L_CONF_CCM_DONE:
      mag->status = QMC5883L_STATUS_IDLE;
      mag->initialized = true;
      break;

    default:
      break;
  }
}

void qmc5883l_read(struct Qmc5883l *mag)
{
  if (mag->status != QMC5883L_STATUS_IDLE) {
    return;
  }

  /* get 3 x 2 bytes data = 6 Bytes and one status byte = 7 */
  mag->i2c_trans.buf[0] = QMC5883L_REG_DATXL;
  mag->i2c_trans.buf[1] = QMC5883L_REG_DATXM;
  mag->i2c_trans.buf[2] = QMC5883L_REG_DATYL;
  mag->i2c_trans.buf[3] = QMC5883L_REG_DATYM;
  mag->i2c_trans.buf[4] = QMC5883L_REG_DATZL;
  mag->i2c_trans.buf[5] = QMC5883L_REG_DATZM;

  /* Chip transaction status, not used ATM in case of driver mishap once can considder using it */
  mag->i2c_trans.buf[6] = QMC5883L_REG_STATUS;
  // Add some code if you experience reading issue
  // DRDY = ((mag->i2c_trans.buf[6]) >> 0) & 1;
  // OVL = ((mag->i2c_trans.buf[6]) >> 1) & 1;
  // DOR = ((uint8_t)(mag->i2c_trans.buf[6]) >> 2) & 1;

  i2c_transceive(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 1, 7);
  mag->status = QMC5883L_STATUS_MEAS;
}
/* Convert and align raw values */
#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))


void qmc5883l_event(struct Qmc5883l *mag)
{
  if (!mag->initialized) {
    return;
  }

  switch (mag->status) {

    case QMC5883L_STATUS_MEAS:
      if (mag->i2c_trans.status == I2CTransSuccess) {
        mag->data.vect.x = Int16FromBuf(mag->i2c_trans.buf, 0);
        mag->data.vect.y = Int16FromBuf(mag->i2c_trans.buf, 2);
        mag->data.vect.z = Int16FromBuf(mag->i2c_trans.buf, 4);

        /* only set available if measurements valid: -4096 if ADC under/overflow in sensor 
        The sensor sends out 12 bit wrapped in 16 bit, that sometimes gets corrupded */
        if (mag->data.vect.x >= 4096 || mag->data.vect.y >= 4096 || mag->data.vect.z >= 4096) {
          //mag->data.adc_overflow_cnt++;
          mag->data_available = false;
        }
        else { 
          mag->data_available = true;
        }
        /* End of measure reading, go back to idle */
        mag->status = QMC5883L_STATUS_IDLE;
      }
      break;

    default:
      if (mag->i2c_trans.status == I2CTransSuccess || mag->i2c_trans.status == I2CTransFailed) {
        /* Per default set to idle */
        mag->i2c_trans.status = I2CTransDone;
        mag->status = QMC5883L_STATUS_IDLE;
      }
      break;
  }
}

