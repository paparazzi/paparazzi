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
 * @file peripherals/qmc5883.c
 *
 * Driver for QMC5883 magnetometer.
 * @todo DRDY/IRQ handling not implemented
 */

#include "peripherals/qmc5883.h"
#include "mcu_periph/sys_time.h"
#include "std.h"

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"

int32_t debuggy;

/* QMC5883 default conf */

// When all correctly shifted outcome should be 0x1D
// only reason for lower ODR and OSR is for a few mA of power saving, not really relevant for UAS IMU to be honest
#ifndef QMC5883_DEFAULT_OSR 
#define QMC5883_DEFAULT_OSR  0x00 ///< 0b00000000 512 Over Sample Ratio
#endif
#ifndef QMC5883_DEFAULT_RNG
#define QMC5883_DEFAULT_RNG  0x10 ///< 0b00010000 8 Gauss Full Scale Range
#endif
#ifndef QMC5883_DEFAULT_ODR
#define QMC5883_DEFAULT_ODR  0x0C ///< 0b00001100 200Hz Output Data Rate, can be 10,50,100 or 200Hz 
#endif
#ifndef QMC5883_DEFAULT_MODE
#define QMC5883_DEFAULT_MODE 0x01 ///< 0b00000001 Continuous Measurement mode
#endif

 /* Control Register 1 */
 #define QMC5883_MODE_REG_STANDBY        (0 << 0)
 #define QMC5883_MODE_REG_CONTINOUS_MODE (1 << 0)
 #define QMC5883_OUTPUT_DATA_RATE_10     (0 << 2) /* Hz */
 #define QMC5883_OUTPUT_DATA_RATE_50     (1 << 2)
 #define QMC5883_OUTPUT_DATA_RATE_100    (2 << 2)
 #define QMC5883_OUTPUT_DATA_RATE_200    (3 << 2)
 #define QMC5883_OUTPUT_RANGE_2G         (0 << 4)  /* +/- 2 gauss */
 #define QMC5883_OUTPUT_RANGE_8G         (1 << 4)  /* +/- 8 gauss */
 #define QMC5883_OVERSAMPLE_512          (0 << 6)  /* controls digital filter bw - larger OSR -> smaller bw */
 #define QMC5883_OVERSAMPLE_256          (1 << 6)
 #define QMC5883_OVERSAMPLE_128          (2 << 6)
 #define QMC5883_OVERSAMPLE_64           (3 << 6)

 //QMC5883_MODE_REG_CONTINOUS_MODE
 //QMC5883_OUTPUT_DATA_RATE_200
 //QMC5883_OVERSAMPLE_512
 //QMC5883_OUTPUT_RANGE_2G

/* Control Register 2 */
 #define QMC5883_INT_ENB    (1 << 0)
 #define QMC5883_ROL_PNT    (1 << 6)
 #define QMC5883_SOFT_RESET (1 << 7)

/** QMC5883 startup delay
 *
 *  On startup, the qmc is making a first conversion in single mode.
 *  Trying to configure the mode register before the end of this conversion
 *  seems to void the configuration.
 *  Default conversion rate is 15 Hz (66ms) and worst case is O.75Hz (1.3s).
 *  Therefore we set the default delay to 1.5s afer boot time.
 */
#ifndef QMC5883_STARTUP_DELAY
#define QMC5883_STARTUP_DELAY 1.7
#endif

static void qmc5883_set_default_config(struct Qmc5883Config *c)
{
  c->allconfigbits=0x1D;
  /*c->osr  = QMC5883_DEFAULT_OSR;
  c->rng  = QMC5883_DEFAULT_RNG;
  c->odr  = QMC5883_DEFAULT_ODR;
  c->mode = QMC5883_DEFAULT_MODE;*/
}

/**
 * Initialize Qmc5883 struct and set default config options.
 * @param qmc   Qmc5883 struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of QMC5883
 */
void qmc5883_init(struct Qmc5883 *qmc, struct i2c_periph *i2c_p, uint8_t addr)
{
  qmc->type = QMC_TYPE_DB5883;//No other support for DA
  qmc->init_status = QMC_CONF_UNINIT;
  /* set i2c_peripheral */
  qmc->i2c_p = i2c_p;
  /* set i2c address */
  qmc->i2c_trans.slave_addr = addr;
  qmc->initialized = false;
  qmc->i2c_trans.status = I2CTransDone;
  /* set default config options */
  qmc5883_set_default_config(&(qmc->config));//Only the values not the device
  qmc->adc_overflow_cnt = 0;
}

static void qmc5883_i2c_tx_reg(struct Qmc5883 *qmc, uint8_t reg, uint8_t val)
{
  qmc->i2c_trans.type = I2CTransTx;
  qmc->i2c_trans.buf[0] = reg;
  qmc->i2c_trans.buf[1] = val;
  qmc->i2c_trans.len_r = 0;
  qmc->i2c_trans.len_w = 2;
  i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
}

/*
     uint8_t data_bits_in = 0;
     read_reg(QMC5883_ADDR_DATA_OUT_X_LSB, data_bits_in);
     write_reg(QMC5883_ADDR_CONTROL_2, QMC5883_SOFT_RESET);
     write_reg(QMC5883_ADDR_SET_RESET, QMC5883_SET_DEFAULT);
     _range_scale = 1.0f / 12000.0f;   // 12000 LSB/Gauss at +/- 2G range
     _range_ga = 2.00f;
     _range_bits = 0x00;
 
     _conf_reg = QMC5883_MODE_REG_CONTINOUS_MODE |
  int32_t debuggy;   write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);
     */

/// Configuration function called once before normal use
static void qmc5883_send_config(struct Qmc5883 *qmc)
{
  switch (qmc->init_status) {
    case QMC_CONF_1:  
      //qmc5883_i2c_tx_reg(qmc, QMC5883_REG_RESET_PERIOD, 0x01); //Set Reset period
      //qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_2, 0x80);//QMC5883_SOFT_RESET); //No Soft reset, Disable Pointer roll-over, Enable interrupt PIN
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_RESET_PERIOD, 0x01); //Set Reset period
      qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_1, 0x1D);//(qmc->config.osr | qmc->config.rng | qmc->config.odr | qmc->config.mode));
      //qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_1, 0x1D);
      //qmc5883_i2c_tx_reg(qmc, QMC5883_REG_CONTROL_1, 0x1D);
       qmc->init_status++;
       debuggy = 3;
       break;
     case QMC_CONF_DONE:
      debuggy = 6;
      qmc->initialized = true;
      qmc->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void qmc5883_start_configure(struct Qmc5883 *qmc)
{
  // wait before starting the configuration
  // doing to early may void the mode configuration
  
  if (qmc->init_status == QMC_CONF_UNINIT && get_sys_time_float() > QMC5883_STARTUP_DELAY) {
    qmc->init_status++;
    if ((qmc->i2c_trans.status == I2CTransSuccess) || (qmc->i2c_trans.status == I2CTransDone)) {
      qmc5883_send_config(qmc);
    }
  }
}

// Normal reading
void qmc5883_read(struct Qmc5883 *qmc)
{
  if (qmc->initialized && qmc->i2c_trans.status == I2CTransDone) {
    debuggy = 0 + rand() % 10; //stuffed in Z as debug to see if we get here every time 
    qmc->i2c_trans.buf[0] = QMC5883_REG_DATXL;
    qmc->i2c_trans.type = I2CTransRx;
    qmc->i2c_trans.len_r = 6;
    qmc->i2c_trans.len_w = 0;
    i2c_submit(qmc->i2c_p, &(qmc->i2c_trans));
  }
}

//Use One of them feel free to debug ;)
#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
//#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void qmc5883_event(struct Qmc5883 *qmc)
{
  debuggy = 50+rand() % 10;
  if (qmc->initialized) {
    debuggy = 100 + rand() % 10;
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->i2c_trans.status = I2CTransDone;
    } else if (qmc->i2c_trans.status == I2CTransSuccess) {
      debuggy = 150 + rand() % 10;
      //if (qmc->type == QMC_TYPE_DB5883) {
        qmc->data.vect.x = Int16FromBuf(qmc->i2c_trans.buf, 0);
        qmc->data.vect.y = Int16FromBuf(qmc->i2c_trans.buf, 2);
        qmc->data.vect.z = 44;//Int16FromBuf(qmc->i2c_trans.buf, 4);
      //}
      /* QMC_DA_5883 has xzy order of axes in returned data */
      //else {
      //  qmc->data.vect.x = Int16FromBuf(qmc->i2c_trans.buf, 0);
      //  qmc->data.vect.y = Int16FromBuf(qmc->i2c_trans.buf, 4); //Mind the difference with DB version
      //  qmc->data.vect.z = Int16FromBuf(qmc->i2c_trans.buf, 2); //Mind the difference with DB version
      //}
      /* only set available if measurements valid: -4096 if ADC under/overflow in sensor */
      if (qmc->data.vect.x != -4096 && qmc->data.vect.y != -4096 && qmc->data.vect.z != -4096) { //Todo datsheet stated 32xxx
        qmc->data_available = true;
      }
      else {
        qmc->adc_overflow_cnt++;
      }
      qmc->i2c_trans.status = I2CTransDone;
    }
  } else if (qmc->init_status != QMC_CONF_UNINIT) { // Configuring but not yet initialized
    if (qmc->i2c_trans.status == I2CTransSuccess || qmc->i2c_trans.status == I2CTransDone) {
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc);
    }
    if (qmc->i2c_trans.status == I2CTransFailed) {
      qmc->init_status--;
      qmc->i2c_trans.status = I2CTransDone;
      qmc5883_send_config(qmc); // Retry config (TODO: add max retry code)
    }
  }
}