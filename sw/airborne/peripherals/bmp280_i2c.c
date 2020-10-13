/*
 * Chris Efstathiou hendrixgr@gmail.com
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
 * @file peripherals/bmp280_i2c.c
 * @brief Sensor driver for BMP280 sensor via I2C
 *
 *
 */

#include "peripherals/bmp280_i2c.h"

/** local function to extract raw data from i2c buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct Bmp280_I2c *bmp);
static void parse_calib_data(struct Bmp280_I2c *bmp);
PRINT_CONFIG_MSG("BMP280 uses double precision compensation")
static double compensate_pressure(struct Bmp280_I2c *bmp);
static double  compensate_temperature(struct Bmp280_I2c *bmp);

int64_t t_fine;

// init function
void bmp280_i2c_init(struct Bmp280_I2c *bmp, struct i2c_periph *i2c_p, uint8_t addr)
{
  // set i2c_peripheral
  bmp->i2c_p = i2c_p;

  // slave address
  bmp->i2c_trans.slave_addr = addr;
  // set initial status: Done
  bmp->i2c_trans.status = I2CTransDone;

  bmp->data_available = false;
  bmp->initialized = false;
  bmp->status = BMP280_STATUS_UNINIT;
}

// Start new measurement if sensor ready
void bmp280_i2c_periodic(struct Bmp280_I2c *bmp)
{
  if (bmp->i2c_trans.status != I2CTransDone) {
    return; // transaction not finished
  }

  switch (bmp->status) {
    case BMP280_STATUS_UNINIT:
      bmp->data_available = false;
      bmp->initialized = false;
      bmp->status = BMP280_STATUS_GET_CALIB;
      break;

    case BMP280_STATUS_GET_CALIB:
      // request calibration data
      bmp->i2c_trans.buf[0] = BMP280_CALIB_LSB_DATA_ADDR;
      i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, BMP280_CALIB_DATA_LEN);
      break;

    case BMP280_STATUS_CONFIGURE:
      // From datasheet, recommended config for drone usecase:
      // osrs_p = 8, osrs_t = 1
      // IIR filter = 2 (note: this one doesn't exist...)
      bmp->i2c_trans.buf[0] = BMP280_CTRL_MEAS_REG_ADDR;
      bmp->i2c_trans.buf[1] = (BMP280_OVERSAMPLING_1X_T | BMP280_OVERSAMPLING_8X_P | BMP280_POWER_NORMAL_MODE);
      bmp->i2c_trans.buf[2] = BMP280_CONFIG_REG_ADDR;
      bmp->i2c_trans.buf[3] = (BMP280_INACTIVITY_62_5_MS | BMP280_IIR_FILTER_COEFF_2);
      i2c_transmit(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, (BMP280_CONFIG_LEN * 2));
      break;

    case  BMP280_STATUS_READ_STATUS_REG:
      // READ THE STATUS BYTE
      bmp->i2c_trans.buf[0] = BMP280_STATUS_REG_ADDR;
      i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, 1);
      break;

    case  BMP280_STATUS_READ_DATA_REGS:
      // READ ALL 6 DATA REGISTERS
      bmp->i2c_trans.buf[0] = BMP280_DATA_START_REG_ADDR;
      i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, BMP280_P_T_DATA_LEN);
      break;

    default:
      break;
  }
}

void bmp280_i2c_event(struct Bmp280_I2c *bmp)
{
  if (bmp->i2c_trans.status == I2CTransSuccess) {
    switch (bmp->status) {

      case BMP280_STATUS_GET_CALIB:
        // compute calib
        parse_calib_data(bmp);
        bmp->status = BMP280_STATUS_CONFIGURE;
        break;

      case BMP280_STATUS_CONFIGURE:
        // nothing else to do, start reading
        bmp->status = BMP280_STATUS_READ_STATUS_REG;
        bmp->initialized = true;
        break;

      case BMP280_STATUS_READ_STATUS_REG:
        // check status byte
        if ((bmp->i2c_trans.buf[0] & (BMP280_EOC_BIT | BMP280_NVRAM_COPY_BIT)) == 0) {
          bmp->status = BMP280_STATUS_READ_DATA_REGS;
        }
        break;

      case BMP280_STATUS_READ_DATA_REGS:
        // parse sensor data, compensate temperature first, then pressure
        parse_sensor_data(bmp);
        compensate_temperature(bmp);
        compensate_pressure(bmp);
        bmp->data_available = true;
        bmp->status = BMP280_STATUS_READ_STATUS_REG;
        break;

      default:
        bmp->status = BMP280_STATUS_GET_CALIB; // just to avoid the compiler's warning message
        break;
    }
    bmp->i2c_trans.status = I2CTransDone;
  } else if (bmp->i2c_trans.status == I2CTransFailed) {
    /* try again */
    if (!bmp->initialized) {
      bmp->status = BMP280_STATUS_UNINIT;
    }
    bmp->i2c_trans.status = I2CTransDone;
  }

  return;
}

static void parse_sensor_data(struct Bmp280_I2c *bmp)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  // BMP280 HAS THE 6 DATA REGISTERS START AT F7 AND GOING UP TO FC MSB FIRST THEN LSB AND LAST THE XLSB BYTE.
  // THE FIRST THREE BYTES ARE THE PRESSURE AND THE NEXT 3 THE TEMPERATURE.
  /* Store the parsed register values for pressure data */
  data_msb = (uint32_t)bmp->i2c_trans.buf[0] << 16;
  data_lsb = (uint32_t)bmp->i2c_trans.buf[1] << 8;
  data_xlsb = (uint32_t)bmp->i2c_trans.buf[2];
  bmp->raw_pressure = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);

  /* Store the parsed register values for temperature data */
  data_msb = (uint32_t)bmp->i2c_trans.buf[3] << 16;
  data_lsb = (uint32_t)bmp->i2c_trans.buf[4] << 8;
  data_xlsb = (uint32_t)bmp->i2c_trans.buf[5];
  bmp->raw_temperature = (int32_t)((data_msb | data_lsb | data_xlsb) >> 4);
}


/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure (float version)
 */
static void parse_calib_data(struct Bmp280_I2c *bmp)
{
  uint8_t *data = (uint8_t *)bmp->i2c_trans.buf; // we know that this buffer will not be modified during this call

  bmp->calib.dig_t1 = BMP280_CONCAT_BYTES(data[1], data[0]);
  bmp->calib.dig_t2 = (int16_t)BMP280_CONCAT_BYTES(data[3], data[2]);
  bmp->calib.dig_t3 = (int16_t)BMP280_CONCAT_BYTES(data[5], data[4]);

  bmp->calib.dig_p1 = BMP280_CONCAT_BYTES(data[7], data[6]);
  bmp->calib.dig_p2 = (int16_t)BMP280_CONCAT_BYTES(data[9], data[8]);
  bmp->calib.dig_p3 = (int16_t)BMP280_CONCAT_BYTES(data[11], data[10]);
  bmp->calib.dig_p4 = (int16_t)BMP280_CONCAT_BYTES(data[13], data[12]);
  bmp->calib.dig_p5 = (int16_t)BMP280_CONCAT_BYTES(data[15], data[14]);
  bmp->calib.dig_p6 = (int16_t)BMP280_CONCAT_BYTES(data[17], data[16]);
  bmp->calib.dig_p7 = (int16_t)BMP280_CONCAT_BYTES(data[19], data[18]);
  bmp->calib.dig_p8 = (int16_t)BMP280_CONCAT_BYTES(data[21], data[20]);
  bmp->calib.dig_p9 = (int16_t)BMP280_CONCAT_BYTES(data[23], data[22]);

  return;
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in float data type.
 */
static double compensate_temperature(struct Bmp280_I2c *bmp)
{

  double var1;
  double var2;

  var1 = (((double)bmp->raw_temperature / 16384.0) - ((double)bmp->calib.dig_t1 / 1024.0)) * ((double)bmp->calib.dig_t2);
  var2 = ((double)bmp->raw_temperature / 131072.0) - ((double)bmp->calib.dig_t1 / 8192.0);
  var2 = (var2 * var2) * (double)bmp->calib.dig_t3;
  t_fine = (int64_t)(var1 + var2);

  /* Store t_lin in dev. structure for pressure calculation */
  bmp->calib.t_fine = t_fine;
  /* Store compensated temperature in float in structure */
  bmp->temperature = (((var1 + var2) / 5120.f) * 100);

  return (double)bmp->temperature;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static double compensate_pressure(struct Bmp280_I2c *bmp)
{
  double var1;
  double var2;
  double p;

  var1 = ((double)t_fine / 2) - 64000.0;
  var2 = (var1 * var1 * (double)bmp->calib.dig_p5) / 32768.0;
  var2 = var2 + (var1 * (double)bmp->calib.dig_p5 * 2.0);
  var2 = (var2 / 4.0) + ((double)bmp->calib.dig_p4 * 65536.0);
  var1 = (((double)bmp->calib.dig_p3 * var1 * (var1 / 524288.0)) + ((double)bmp->calib.dig_p2 * var1)) / 524288.0;
  var1 = (1 + (var1 / 32768.0)) * (double)bmp->calib.dig_p1;
  p = 1048576.0 - (double)bmp->raw_pressure;
  p = (p - (var2 / 4096.0)) * (6250.0 / var1);
  var1 = ((double)bmp->calib.dig_p9 * p) * (p / 2147483648.0);
  var2 = (p * ((double)bmp->calib.dig_p8)) / 32768.0;
  p = p + ((var1 + var2 + (double)bmp->calib.dig_p7) / 16.0);
  bmp->pressure = p;

  return (p);
}
