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
 * @file peripherals/bmi088.c
 *
 * BMI088 driver common functions (I2C and SPI).
 *
 * Still needs the either I2C or SPI specific implementation.
 */

#include "peripherals/bmi088.h"

const float BMI088_GYRO_SENS[5] = {
  BMI088_GYRO_SENS_2000,
  BMI088_GYRO_SENS_1000,
  BMI088_GYRO_SENS_500,
  BMI088_GYRO_SENS_250,
  BMI088_GYRO_SENS_125,
};

const struct Int32Rates BMI088_GYRO_SENS_FRAC[5][2] = {
  { {BMI088_GYRO_SENS_2000_NUM, BMI088_GYRO_SENS_2000_NUM, BMI088_GYRO_SENS_2000_NUM},
    {BMI088_GYRO_SENS_2000_DEN, BMI088_GYRO_SENS_2000_DEN, BMI088_GYRO_SENS_2000_DEN} },
  { {BMI088_GYRO_SENS_1000_NUM, BMI088_GYRO_SENS_1000_NUM, BMI088_GYRO_SENS_1000_NUM},
    {BMI088_GYRO_SENS_1000_DEN, BMI088_GYRO_SENS_1000_DEN, BMI088_GYRO_SENS_1000_DEN} },
  { {BMI088_GYRO_SENS_500_NUM, BMI088_GYRO_SENS_500_NUM, BMI088_GYRO_SENS_500_NUM},
    {BMI088_GYRO_SENS_500_DEN, BMI088_GYRO_SENS_500_DEN, BMI088_GYRO_SENS_500_DEN} },
  { {BMI088_GYRO_SENS_250_NUM, BMI088_GYRO_SENS_250_NUM, BMI088_GYRO_SENS_250_NUM},
    {BMI088_GYRO_SENS_250_DEN, BMI088_GYRO_SENS_250_DEN, BMI088_GYRO_SENS_250_DEN} },
  { {BMI088_GYRO_SENS_125_NUM, BMI088_GYRO_SENS_125_NUM, BMI088_GYRO_SENS_125_NUM},
    {BMI088_GYRO_SENS_125_DEN, BMI088_GYRO_SENS_125_DEN, BMI088_GYRO_SENS_125_DEN} }
};

const float BMI088_ACCEL_SENS[4] = {
  BMI088_ACCEL_SENS_3G,
  BMI088_ACCEL_SENS_6G,
  BMI088_ACCEL_SENS_12G,
  BMI088_ACCEL_SENS_24G
};

const struct Int32Vect3 BMI088_ACCEL_SENS_FRAC[4][2] = {
  { {BMI088_ACCEL_SENS_3G_NUM, BMI088_ACCEL_SENS_3G_NUM, BMI088_ACCEL_SENS_3G_NUM},
    {BMI088_ACCEL_SENS_3G_DEN, BMI088_ACCEL_SENS_3G_DEN, BMI088_ACCEL_SENS_3G_DEN} },
  { {BMI088_ACCEL_SENS_6G_NUM, BMI088_ACCEL_SENS_6G_NUM, BMI088_ACCEL_SENS_6G_NUM},
    {BMI088_ACCEL_SENS_6G_DEN, BMI088_ACCEL_SENS_6G_DEN, BMI088_ACCEL_SENS_6G_DEN} },
  { {BMI088_ACCEL_SENS_12G_NUM, BMI088_ACCEL_SENS_12G_NUM, BMI088_ACCEL_SENS_12G_NUM},
    {BMI088_ACCEL_SENS_12G_DEN, BMI088_ACCEL_SENS_12G_DEN, BMI088_ACCEL_SENS_12G_DEN} },
  { {BMI088_ACCEL_SENS_24G_NUM, BMI088_ACCEL_SENS_24G_NUM, BMI088_ACCEL_SENS_24G_NUM},
    {BMI088_ACCEL_SENS_24G_DEN, BMI088_ACCEL_SENS_24G_DEN, BMI088_ACCEL_SENS_24G_DEN} }
};

void bmi088_set_default_config(struct Bmi088Config *c)
{
  c->gyro_range = BMI088_DEFAULT_GYRO_RANGE;
  c->gyro_odr = BMI088_DEFAULT_GYRO_ODR;
  c->accel_range = BMI088_DEFAULT_ACCEL_RANGE;
  c->accel_odr = BMI088_DEFAULT_ACCEL_ODR;
  c->accel_bw = BMI088_DEFAULT_ACCEL_BW;
}

void bmi088_send_config(Bmi088ConfigSet bmi_set, void *bmi, struct Bmi088Config *config)
{
  switch (config->init_status) {
    case BMI088_CONF_ACCEL_RANGE:
      /* configure accelerometer range */
      bmi_set(bmi, BMI088_ACCEL_RANGE, config->accel_range, BMI088_CONFIG_ACCEL);
      config->init_status++;
      break;
    case BMI088_CONF_ACCEL_ODR:
      /* configure accelerometer odr and bw */
      bmi_set(bmi, BMI088_ACCEL_CONF, ((config->accel_bw << 4) | config->accel_odr), BMI088_CONFIG_ACCEL);
      config->init_status++;
      break;
    case BMI088_CONF_ACCEL_PWR_CONF:
      /* switch on accel */
      bmi_set(bmi, BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_ACTIVE, BMI088_CONFIG_ACCEL);
      config->init_status++;
      break;
    case BMI088_CONF_ACCEL_PWR_CTRL:
      /* switch on accel */
      bmi_set(bmi, BMI088_ACCEL_PWR_CTRl, BMI088_ACCEL_POWER_ON, BMI088_CONFIG_ACCEL);
      config->init_status++;
      break;
    case BMI088_CONF_GYRO_RANGE:
      /* configure gyro range */
      bmi_set(bmi, BMI088_GYRO_RANGE, config->gyro_range, BMI088_CONFIG_GYRO);
      config->init_status++;
      break;
    case BMI088_CONF_GYRO_ODR:
      /* configure gyro odr */
      bmi_set(bmi, BMI088_GYRO_BAND_WIDTH, config->gyro_odr, BMI088_CONFIG_GYRO);
      config->init_status++;
      break;
    case BMI088_CONF_GYRO_PWR:
      /* switch on gyro */
      bmi_set(bmi, BMI088_GYRO_LPM_1, BMI088_GYRO_NORMAL, BMI088_CONFIG_GYRO);
      config->init_status++;
      break;
    case BMI088_CONF_DONE:
      config->initialized = true;
      break;
    default:
      break;
  }
}

