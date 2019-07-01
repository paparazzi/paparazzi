/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file peripherals/bmp3_i2c.c
 * @brief Sensor driver for BMP3 sensor via I2C
 *
 * Modified for Paparazzi from SDP3 driver from BoshSensortec
 * see https://github.com/BoschSensortec/BMP3-Sensor-API
 * for original code and license
 *
 */

#include "peripherals/bmp3_i2c.h"

/** local function to extract raw data from i2c buffer
 *  and compute compensation with selected precision
 */
static void parse_sensor_data(struct Bmp3_I2c *bmp);
static void parse_calib_data(struct Bmp3_I2c *bmp);
#if BMP3_COMPENSATION == BMP3_DOUBLE_PRECISION_COMPENSATION
PRINT_CONFIG_MSG("BMP3 double precision compensation")
static double compensate_pressure(struct Bmp3_I2c *bmp);
static double compensate_temperature(struct Bmp3_I2c *bmp);
static double bmp3_pow(double base, uint8_t power);
#elif BMP3_COMPENSATION == BMP3_SINGLE_PRECISION_COMPENSATION
PRINT_CONFIG_MSG("BMP3 single precision compensation")
static float compensate_pressure(struct Bmp3_I2c *bmp);
static float compensate_temperature(struct Bmp3_I2c *bmp);
static float bmp3_pow(float base, uint8_t power);
#elif BMP3_COMPENSATION == BMP3_INTEGER_COMPENSATION
PRINT_CONFIG_MSG("BMP3 integer compensation")
static int64_t compensate_temperature(struct Bmp3_I2c *bmp);
static uint64_t compensate_pressure(struct Bmp3_I2c *bmp);
#else
#error "BMP3: Unknown compensation type"
#endif


/**
 * init function
 */
void bmp3_i2c_init(struct Bmp3_I2c *bmp, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  bmp->i2c_p = i2c_p;

  /* slave address */
  bmp->i2c_trans.slave_addr = addr;
  /* set initial status: Done */
  bmp->i2c_trans.status = I2CTransDone;

  bmp->data_available = false;
  bmp->initialized = false;
  bmp->status = BMP3_STATUS_UNINIT;
}

/**
 * Start new measurement if sensor ready
 */
void bmp3_i2c_periodic(struct Bmp3_I2c *bmp)
{
  if (bmp->i2c_trans.status != I2CTransDone) {
    return; // transaction not finished
  }

  switch (bmp->status) {
    case BMP3_STATUS_UNINIT:
      bmp->data_available = false;
      bmp->initialized = false;
      bmp->status = BMP3_STATUS_GET_CALIB;
      break;

    case BMP3_STATUS_GET_CALIB:
      // request calibration data
      bmp->i2c_trans.buf[0] = BMP3_CALIB_DATA_ADDR;
      i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, BMP3_CALIB_DATA_LEN);
      break;

    case BMP3_STATUS_CONFIGURE:
      // From datasheet, recommended config for drone usecase:
      // osrs_p = 8, osrs_t = 1
      // IIR filter = 2 (note: this one doesn't exist...)
      // ODR = 50
      bmp->i2c_trans.buf[0] = BMP3_PWR_CTRL_ADDR;
      bmp->i2c_trans.buf[1] = BMP3_ALL | BMP3_NORMAL_MODE << 4;
      bmp->i2c_trans.buf[2] = BMP3_OSR_ADDR;
      bmp->i2c_trans.buf[3] = BMP3_OVERSAMPLING_8X | BMP3_NO_OVERSAMPLING << 3;
      bmp->i2c_trans.buf[4] = BMP3_ODR_ADDR;
      bmp->i2c_trans.buf[5] = BMP3_ODR_50_HZ;
      bmp->i2c_trans.buf[6] = BMP3_CONFIG_ADDR;
      bmp->i2c_trans.buf[7] = BMP3_IIR_FILTER_COEFF_3 << 1;
      i2c_transmit(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 8);
      break;

    case BMP3_STATUS_READ_DATA:
      /* read data */
      bmp->i2c_trans.buf[0] = BMP3_SENS_STATUS_REG_ADDR;
      i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, BMP3_P_AND_T_HEADER_DATA_LEN);
      break;

    default:
      break;
  }
}


void bmp3_i2c_event(struct Bmp3_I2c *bmp)
{
  if (bmp->i2c_trans.status == I2CTransSuccess) {
    switch (bmp->status) {
      case BMP3_STATUS_GET_CALIB:
        // compute calib
        parse_calib_data(bmp);
        bmp->status = BMP3_STATUS_CONFIGURE;
        break;

      case BMP3_STATUS_CONFIGURE:
        // nothing else to do, start reading
        bmp->status = BMP3_STATUS_READ_DATA;
        bmp->initialized = true;
        break;

      case BMP3_STATUS_READ_DATA:
        // check status byte
        if (bmp->i2c_trans.buf[0] & (BMP3_ALL << 5)) {
          // parse sensor data, compensate temperature first, then pressure
          parse_sensor_data(bmp);
          compensate_temperature(bmp);
          compensate_pressure(bmp);
          bmp->data_available = true;
        }
        break;

      default:
        break;
    }
    bmp->i2c_trans.status = I2CTransDone;
  } else if (bmp->i2c_trans.status == I2CTransFailed) {
    /* try again */
    if (!bmp->initialized) {
      bmp->status = BMP3_STATUS_UNINIT;
    }
    bmp->i2c_trans.status = I2CTransDone;
  }
}


static void parse_sensor_data(struct Bmp3_I2c *bmp)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  /* Store the parsed register values for pressure data */
  data_xlsb = (uint32_t)bmp->i2c_trans.buf[1];
  data_lsb = (uint32_t)bmp->i2c_trans.buf[2] << 8;
  data_msb = (uint32_t)bmp->i2c_trans.buf[3] << 16;
  bmp->raw_pressure = data_msb | data_lsb | data_xlsb;

  /* Store the parsed register values for temperature data */
  data_xlsb = (uint32_t)bmp->i2c_trans.buf[4];
  data_lsb = (uint32_t)bmp->i2c_trans.buf[5] << 8;
  data_msb = (uint32_t)bmp->i2c_trans.buf[6] << 16;
  bmp->raw_temperature = data_msb | data_lsb | data_xlsb;
}


#if BMP3_COMPENSATION == BMP3_DOUBLE_PRECISION_COMPENSATION

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_data(struct Bmp3_I2c *bmp)
{
  double temp_var;
  uint8_t *data = (uint8_t *)bmp->i2c_trans.buf; // we know that this buffer will not be modified during this call

  /* 1 / 2^8 */
  temp_var = 0.00390625;
  bmp->calib.par_t1 = BMP3_CONCAT_BYTES(data[1], data[0]);
  bmp->quant_calib.par_t1 = ((double)bmp->calib.par_t1 / temp_var);

  bmp->calib.par_t2 = BMP3_CONCAT_BYTES(data[3], data[2]);
  temp_var = 1073741824.0;
  bmp->quant_calib.par_t2 = ((double)bmp->calib.par_t2 / temp_var);

  bmp->calib.par_t3 = (int8_t)data[4];
  temp_var = 281474976710656.0;
  bmp->quant_calib.par_t3 = ((double)bmp->calib.par_t3 / temp_var);

  bmp->calib.par_p1 = (int16_t)BMP3_CONCAT_BYTES(data[6], data[5]);
  temp_var = 1048576.0;
  bmp->quant_calib.par_p1 = ((double)(bmp->calib.par_p1 - (16384)) / temp_var);

  bmp->calib.par_p2 = (int16_t)BMP3_CONCAT_BYTES(data[8], data[7]);
  temp_var = 536870912.0;
  bmp->quant_calib.par_p2 = ((double)(bmp->calib.par_p2 - (16384)) / temp_var);

  bmp->calib.par_p3 = (int8_t)data[9];
  temp_var = 4294967296.0;
  bmp->quant_calib.par_p3 = ((double)bmp->calib.par_p3 / temp_var);

  bmp->calib.par_p4 = (int8_t)data[10];
  temp_var = 137438953472.0;
  bmp->quant_calib.par_p4 = ((double)bmp->calib.par_p4 / temp_var);

  bmp->calib.par_p5 = BMP3_CONCAT_BYTES(data[12], data[11]);
  /* 1 / 2^3 */
  temp_var = 0.125;
  bmp->quant_calib.par_p5 = ((double)bmp->calib.par_p5 / temp_var);

  bmp->calib.par_p6 = BMP3_CONCAT_BYTES(data[14],  data[13]);
  temp_var = 64.0;
  bmp->quant_calib.par_p6 = ((double)bmp->calib.par_p6 / temp_var);

  bmp->calib.par_p7 = (int8_t)data[15];
  temp_var = 256.0;
  bmp->quant_calib.par_p7 = ((double)bmp->calib.par_p7 / temp_var);

  bmp->calib.par_p8 = (int8_t)data[16];
  temp_var = 32768.0;
  bmp->quant_calib.par_p8 = ((double)bmp->calib.par_p8 / temp_var);

  bmp->calib.par_p9 = (int16_t)BMP3_CONCAT_BYTES(data[18], data[17]);
  temp_var = 281474976710656.0;
  bmp->quant_calib.par_p9 = ((double)bmp->calib.par_p9 / temp_var);

  bmp->calib.par_p10 = (int8_t)data[19];
  temp_var = 281474976710656.0;
  bmp->quant_calib.par_p10 = ((double)bmp->calib.par_p10 / temp_var);

  bmp->calib.par_p11 = (int8_t)data[20];
  temp_var = 36893488147419103232.0;
  bmp->quant_calib.par_p11 = ((double)bmp->calib.par_p11 / temp_var);
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(struct Bmp3_I2c *bmp)
{
  double partial_data1 = (double)(bmp->raw_temperature - bmp->quant_calib.par_t1);
  double partial_data2 = (double)(partial_data1 * bmp->quant_calib.par_t2);
  /* Update the compensated temperature in structure since this is
     needed for pressure calculation */
  bmp->quant_calib.t_lin = partial_data2 + (partial_data1 * partial_data1)
      * bmp->quant_calib.par_t3;

  /* Store compensated temperature in float in structure */
  bmp->temperature = (float)bmp->quant_calib.t_lin;

  /* Returns compensated temperature */
  return bmp->quant_calib.t_lin;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure(struct Bmp3_I2c *bmp)
{
  /* Variable to store the compensated pressure */
  double comp_press;
  /* Temporary variables used for compensation */
  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;
  double partial_out1;
  double partial_out2;

  partial_data1 = bmp->quant_calib.par_p6 * bmp->quant_calib.t_lin;
  partial_data2 = bmp->quant_calib.par_p7 * bmp3_pow(bmp->quant_calib.t_lin, 2);
  partial_data3 = bmp->quant_calib.par_p8 * bmp3_pow(bmp->quant_calib.t_lin, 3);
  partial_out1 = bmp->quant_calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = bmp->quant_calib.par_p2 * bmp->quant_calib.t_lin;
  partial_data2 = bmp->quant_calib.par_p3 * bmp3_pow(bmp->quant_calib.t_lin, 2);
  partial_data3 = bmp->quant_calib.par_p4 * bmp3_pow(bmp->quant_calib.t_lin, 3);
  partial_out2 = bmp->raw_pressure *
                 (bmp->quant_calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = bmp3_pow((double)bmp->raw_pressure, 2);
  partial_data2 = bmp->quant_calib.par_p9 + bmp->quant_calib.par_p10 * bmp->quant_calib.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + bmp3_pow((double)bmp->raw_pressure, 3) * bmp->quant_calib.par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;

  /* Store compensated temperature in float in structure */
  bmp->pressure = (float)comp_press;

  return comp_press;
}

/**
 * @brief This internal API is used to calculate the power functionality for
 * double precision floating point values.
 */
static double bmp3_pow(double base, uint8_t power)
{
  double pow_output = 1;

  while (power != 0) {
    pow_output = base * pow_output;
    power--;
  }

  return pow_output;
}

#elif BMP3_COMPENSATION == BMP3_SINGLE_PRECISION_COMPENSATION

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure (float version)
 */
static void parse_calib_data(struct Bmp3_I2c *bmp)
{
  float temp_var;
  uint8_t *data = (uint8_t *)bmp->i2c_trans.buf; // we know that this buffer will not be modified during this call

  /* 1 / 2^8 */
  temp_var = 0.00390625f;
  bmp->calib.par_t1 = BMP3_CONCAT_BYTES(data[1], data[0]);
  bmp->quant_calib.par_t1 = ((float)bmp->calib.par_t1 / temp_var);

  bmp->calib.par_t2 = BMP3_CONCAT_BYTES(data[3], data[2]);
  temp_var = 1073741824.0f;
  bmp->quant_calib.par_t2 = ((float)bmp->calib.par_t2 / temp_var);

  bmp->calib.par_t3 = (int8_t)data[4];
  temp_var = 281474976710656.0f;
  bmp->quant_calib.par_t3 = ((float)bmp->calib.par_t3 / temp_var);

  bmp->calib.par_p1 = (int16_t)BMP3_CONCAT_BYTES(data[6], data[5]);
  temp_var = 1048576.0f;
  bmp->quant_calib.par_p1 = ((float)(bmp->calib.par_p1 - (16384)) / temp_var);

  bmp->calib.par_p2 = (int16_t)BMP3_CONCAT_BYTES(data[8], data[7]);
  temp_var = 536870912.0f;
  bmp->quant_calib.par_p2 = ((float)(bmp->calib.par_p2 - (16384)) / temp_var);

  bmp->calib.par_p3 = (int8_t)data[9];
  temp_var = 4294967296.0f;
  bmp->quant_calib.par_p3 = ((float)bmp->calib.par_p3 / temp_var);

  bmp->calib.par_p4 = (int8_t)data[10];
  temp_var = 137438953472.0f;
  bmp->quant_calib.par_p4 = ((float)bmp->calib.par_p4 / temp_var);

  bmp->calib.par_p5 = BMP3_CONCAT_BYTES(data[12], data[11]);
  /* 1 / 2^3 */
  temp_var = 0.125f;
  bmp->quant_calib.par_p5 = ((float)bmp->calib.par_p5 / temp_var);

  bmp->calib.par_p6 = BMP3_CONCAT_BYTES(data[14],  data[13]);
  temp_var = 64.0f;
  bmp->quant_calib.par_p6 = ((float)bmp->calib.par_p6 / temp_var);

  bmp->calib.par_p7 = (int8_t)data[15];
  temp_var = 256.0f;
  bmp->quant_calib.par_p7 = ((float)bmp->calib.par_p7 / temp_var);

  bmp->calib.par_p8 = (int8_t)data[16];
  temp_var = 32768.0f;
  bmp->quant_calib.par_p8 = ((float)bmp->calib.par_p8 / temp_var);

  bmp->calib.par_p9 = (int16_t)BMP3_CONCAT_BYTES(data[18], data[17]);
  temp_var = 281474976710656.0f;
  bmp->quant_calib.par_p9 = ((float)bmp->calib.par_p9 / temp_var);

  bmp->calib.par_p10 = (int8_t)data[19];
  temp_var = 281474976710656.0f;
  bmp->quant_calib.par_p10 = ((float)bmp->calib.par_p10 / temp_var);

  bmp->calib.par_p11 = (int8_t)data[20];
  temp_var = 36893488147419103232.0f;
  bmp->quant_calib.par_p11 = ((float)bmp->calib.par_p11 / temp_var);
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in float data type.
 */
static float compensate_temperature(struct Bmp3_I2c *bmp)
{
  float partial_data1 = (float)(bmp->raw_temperature - bmp->quant_calib.par_t1);
  float partial_data2 = (float)(partial_data1 * bmp->quant_calib.par_t2);
  /* Update the compensated temperature in structure since this is
     needed for pressure calculation */
  bmp->quant_calib.t_lin = partial_data2 + (partial_data1 * partial_data1)
      * bmp->quant_calib.par_t3;

  /* Store compensated temperature in float in structure */
  bmp->temperature = (float)bmp->quant_calib.t_lin;

  /* Returns compensated temperature */
  return bmp->quant_calib.t_lin;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in float data type.
 */
static float compensate_pressure(struct Bmp3_I2c *bmp)
{
  /* Variable to store the compensated pressure */
  float comp_press;
  /* Temporary variables used for compensation */
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;

  partial_data1 = bmp->quant_calib.par_p6 * bmp->quant_calib.t_lin;
  partial_data2 = bmp->quant_calib.par_p7 * bmp3_pow(bmp->quant_calib.t_lin, 2);
  partial_data3 = bmp->quant_calib.par_p8 * bmp3_pow(bmp->quant_calib.t_lin, 3);
  partial_out1 = bmp->quant_calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = bmp->quant_calib.par_p2 * bmp->quant_calib.t_lin;
  partial_data2 = bmp->quant_calib.par_p3 * bmp3_pow(bmp->quant_calib.t_lin, 2);
  partial_data3 = bmp->quant_calib.par_p4 * bmp3_pow(bmp->quant_calib.t_lin, 3);
  partial_out2 = bmp->raw_pressure *
                 (bmp->quant_calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = bmp3_pow((float)bmp->raw_pressure, 2);
  partial_data2 = bmp->quant_calib.par_p9 + bmp->quant_calib.par_p10 * bmp->quant_calib.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + bmp3_pow((float)bmp->raw_pressure, 3) * bmp->quant_calib.par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;

  /* Store compensated temperature in float in structure */
  bmp->pressure = (float)comp_press;

  return comp_press;
}

/**
 * @brief This internal API is used to calculate the power functionality for
 * float precision floating point values.
 */
static float bmp3_pow(float base, uint8_t power)
{
  float pow_output = 1;

  while (power != 0) {
    pow_output = base * pow_output;
    power--;
  }

  return pow_output;
}

#elif BMP3_COMPENSATION == BMP3_INTEGER_COMPENSATION

/**
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_data(struct Bmp3_I2c *bmp)
{
  uint8_t *data = (uint8_t *)bmp->i2c_trans.buf; // we know that this buffer will not be modified during this call

  bmp->calib.par_t1 = BMP3_CONCAT_BYTES(data[1], data[0]);
  bmp->calib.par_t2 = BMP3_CONCAT_BYTES(data[3], data[2]);
  bmp->calib.par_t3 = (int8_t)data[4];
  bmp->calib.par_p1 = (int16_t)BMP3_CONCAT_BYTES(data[6], data[5]);
  bmp->calib.par_p2 = (int16_t)BMP3_CONCAT_BYTES(data[8], data[7]);
  bmp->calib.par_p3 = (int8_t)data[9];
  bmp->calib.par_p4 = (int8_t)data[10];
  bmp->calib.par_p5 = BMP3_CONCAT_BYTES(data[12], data[11]);
  bmp->calib.par_p6 = BMP3_CONCAT_BYTES(data[14],  data[13]);
  bmp->calib.par_p7 = (int8_t)data[15];
  bmp->calib.par_p8 = (int8_t)data[16];
  bmp->calib.par_p9 = (int16_t)BMP3_CONCAT_BYTES(data[18], data[17]);
  bmp->calib.par_p10 = (int8_t)data[19];
  bmp->calib.par_p11 = (int8_t)data[20];
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
static int64_t compensate_temperature(struct Bmp3_I2c *bmp)
{
  uint64_t partial_data1;
  uint64_t partial_data2;
  uint64_t partial_data3;
  int64_t partial_data4;
  int64_t partial_data5;
  int64_t partial_data6;
  int64_t comp_temp;

  partial_data1 = bmp->raw_temperature - (256 * bmp->calib.par_t1);
  partial_data2 = bmp->calib.par_t2 * partial_data1;
  partial_data3 = partial_data1 * partial_data1;
  partial_data4 = (int64_t)partial_data3 * bmp->calib.par_t3;
  partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
  partial_data6 = partial_data5 / 4294967296;
  /* Store t_lin in dev. structure for pressure calculation */
  bmp->calib.t_lin = partial_data6;
  comp_temp = (int64_t)((partial_data6 * 25)  / 16384);

  /* Store compensated temperature in float in structure */
  bmp->temperature = ((float)comp_temp) / 100.f;

  return comp_temp;
}

/**
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static uint64_t compensate_pressure(struct Bmp3_I2c *bmp)
{
  int64_t partial_data1;
  int64_t partial_data2;
  int64_t partial_data3;
  int64_t partial_data4;
  int64_t partial_data5;
  int64_t partial_data6;
  int64_t offset;
  int64_t sensitivity;
  uint64_t comp_press;

  partial_data1 = bmp->calib.t_lin * bmp->calib.t_lin;
  partial_data2 = partial_data1 / 64;
  partial_data3 = (partial_data2 * bmp->calib.t_lin) / 256;
  partial_data4 = (bmp->calib.par_p8 * partial_data3) / 32;
  partial_data5 = (bmp->calib.par_p7 * partial_data1) * 16;
  partial_data6 = (bmp->calib.par_p6 * bmp->calib.t_lin) * 4194304;
  offset = (bmp->calib.par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

  partial_data2 = (bmp->calib.par_p4 * partial_data3) / 32;
  partial_data4 = (bmp->calib.par_p3 * partial_data1) * 4;
  partial_data5 = (bmp->calib.par_p2 - 16384) * bmp->calib.t_lin * 2097152;
  sensitivity = ((bmp->calib.par_p1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;

  partial_data1 = (sensitivity / 16777216) * bmp->raw_pressure;
  partial_data2 = bmp->calib.par_p10 * bmp->calib.t_lin;
  partial_data3 = partial_data2 + (65536 * bmp->calib.par_p9);
  partial_data4 = (partial_data3 * bmp->raw_pressure) / 8192;
  partial_data5 = (partial_data4 * bmp->raw_pressure) / 512;
  partial_data6 = (int64_t)((uint64_t)bmp->raw_pressure * (uint64_t)bmp->raw_pressure);
  partial_data2 = (bmp->calib.par_p11 * partial_data6) / 65536;
  partial_data3 = (partial_data2 * bmp->raw_pressure) / 128;
  partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
  comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

  /* Store compensated temperature in float in structure */
  bmp->pressure = ((float)comp_press) / 100.f;

  return comp_press;
}

#endif

