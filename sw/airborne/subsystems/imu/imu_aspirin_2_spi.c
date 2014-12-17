/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_aspirin_2_spi.c
 * Driver for the Aspirin v2.x IMU using SPI for the MPU6000.
 */

#include "subsystems/imu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"
#include "peripherals/hmc58xx_regs.h"

/* defaults suitable for Lisa */
#ifndef ASPIRIN_2_SPI_SLAVE_IDX
#define ASPIRIN_2_SPI_SLAVE_IDX SPI_SLAVE2
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_SPI_SLAVE_IDX)

#ifndef ASPIRIN_2_SPI_DEV
#define ASPIRIN_2_SPI_DEV spi2
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_SPI_DEV)

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined ASPIRIN_2_LOWPASS_FILTER && !defined  ASPIRIN_2_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define ASPIRIN_2_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define ASPIRIN_2_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define ASPIRIN_2_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define ASPIRIN_2_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define ASPIRIN_2_LOWPASS_FILTER and ASPIRIN_2_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_LOWPASS_FILTER)
PRINT_CONFIG_VAR(ASPIRIN_2_SMPLRT_DIV)

#ifndef ASPIRIN_2_GYRO_RANGE
#define ASPIRIN_2_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_GYRO_RANGE)

#ifndef ASPIRIN_2_ACCEL_RANGE
#define ASPIRIN_2_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_ACCEL_RANGE)


/* HMC58XX default conf */
#ifndef HMC58XX_DO
#define HMC58XX_DO 0x6 // Data Output Rate (6 -> 75Hz with HMC5883)
#endif
#ifndef HMC58XX_MS
#define HMC58XX_MS 0x0 // Measurement configuration
#endif
#ifndef HMC58XX_GN
#define HMC58XX_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)
#endif
#ifndef HMC58XX_MD
#define HMC58XX_MD 0x0 // Continious measurement mode
#endif

#define HMC58XX_CRA ((HMC58XX_DO<<2)|(HMC58XX_MS))
#define HMC58XX_CRB (HMC58XX_GN<<5)

/** delay in seconds before starting to configure HMC58xx mag slave */
#ifndef ASPIRIN_2_MAG_STARTUP_DELAY
#define ASPIRIN_2_MAG_STARTUP_DELAY 1.5
#endif

struct ImuAspirin2Spi imu_aspirin2;

void mpu_wait_slave4_ready(void);
void mpu_wait_slave4_ready_cb(struct spi_transaction *t);
bool_t imu_aspirin2_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu);

void imu_impl_init(void)
{
  imu_aspirin2.accel_valid = FALSE;
  imu_aspirin2.gyro_valid = FALSE;
  imu_aspirin2.mag_valid = FALSE;

  mpu60x0_spi_init(&imu_aspirin2.mpu, &(ASPIRIN_2_SPI_DEV), ASPIRIN_2_SPI_SLAVE_IDX);
  // change the default configuration
  imu_aspirin2.mpu.config.smplrt_div = ASPIRIN_2_SMPLRT_DIV;
  imu_aspirin2.mpu.config.dlpf_cfg = ASPIRIN_2_LOWPASS_FILTER;
  imu_aspirin2.mpu.config.gyro_range = ASPIRIN_2_GYRO_RANGE;
  imu_aspirin2.mpu.config.accel_range = ASPIRIN_2_ACCEL_RANGE;

  /* read 15 bytes for status, accel, gyro + 6 bytes for mag slave */
  imu_aspirin2.mpu.config.nb_bytes = 21;

  /* HMC5883 magnetometer as I2C slave */
  imu_aspirin2.mpu.config.nb_slaves = 1;

  /* set function to configure mag */
  imu_aspirin2.mpu.config.slaves[0].configure = &imu_aspirin2_configure_mag_slave;


  /* Set MPU I2C master clock */
  imu_aspirin2.mpu.config.i2c_mst_clk = MPU60X0_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_aspirin2.mpu.config.i2c_mst_delay = 1;


  /* configure spi transaction for wait_slave4 */
  imu_aspirin2.wait_slave4_trans.cpol = SPICpolIdleHigh;
  imu_aspirin2.wait_slave4_trans.cpha = SPICphaEdge2;
  imu_aspirin2.wait_slave4_trans.dss = SPIDss8bit;
  imu_aspirin2.wait_slave4_trans.bitorder = SPIMSBFirst;
  imu_aspirin2.wait_slave4_trans.cdiv = SPIDiv64;

  imu_aspirin2.wait_slave4_trans.select = SPISelectUnselect;
  imu_aspirin2.wait_slave4_trans.slave_idx = ASPIRIN_2_SPI_SLAVE_IDX;
  imu_aspirin2.wait_slave4_trans.output_length = 1;
  imu_aspirin2.wait_slave4_trans.input_length = 2;
  imu_aspirin2.wait_slave4_trans.before_cb = NULL;
  imu_aspirin2.wait_slave4_trans.after_cb = mpu_wait_slave4_ready_cb;
  imu_aspirin2.wait_slave4_trans.input_buf = &(imu_aspirin2.wait_slave4_rx_buf[0]);
  imu_aspirin2.wait_slave4_trans.output_buf = &(imu_aspirin2.wait_slave4_tx_buf[0]);

  imu_aspirin2.wait_slave4_trans.status = SPITransDone;
  imu_aspirin2.slave4_ready = FALSE;
}


void imu_periodic(void)
{
  mpu60x0_spi_periodic(&imu_aspirin2.mpu);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void imu_aspirin2_event(void)
{
  mpu60x0_spi_event(&imu_aspirin2.mpu);
  if (imu_aspirin2.mpu.data_available) {
    /* HMC5883 has xzy order of axes in returned data */
    struct Int32Vect3 mag;
    mag.x = Int16FromBuf(imu_aspirin2.mpu.data_ext, 0);
    mag.z = Int16FromBuf(imu_aspirin2.mpu.data_ext, 2);
    mag.y = Int16FromBuf(imu_aspirin2.mpu.data_ext, 4);

    /* Handle axis assignement for Lisa/S integrated Aspirin like IMU. */
#ifdef LISA_S
#ifdef LISA_S_UPSIDE_DOWN
    RATES_ASSIGN(imu.gyro_unscaled,
                 imu_aspirin2.mpu.data_rates.rates.p,
                 -imu_aspirin2.mpu.data_rates.rates.q,
                 -imu_aspirin2.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,
                 imu_aspirin2.mpu.data_accel.vect.x,
                 -imu_aspirin2.mpu.data_accel.vect.y,
                 -imu_aspirin2.mpu.data_accel.vect.z);
    VECT3_ASSIGN(imu.mag_unscaled, mag.x, -mag.y, -mag.z);
#else
    RATES_COPY(imu.gyro_unscaled, imu_aspirin2.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_aspirin2.mpu.data_accel.vect);
    VECT3_COPY(imu.mag_unscaled, mag);
#endif
#else

    /* Handle axis assignement for Lisa/M or Lisa/MX V2.1 integrated Aspirin like
     * IMU.
     */
#ifdef LISA_M_OR_MX_21
    RATES_ASSIGN(imu.gyro_unscaled,
                 -imu_aspirin2.mpu.data_rates.rates.q,
                 imu_aspirin2.mpu.data_rates.rates.p,
                 imu_aspirin2.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,
                 -imu_aspirin2.mpu.data_accel.vect.y,
                 imu_aspirin2.mpu.data_accel.vect.x,
                 imu_aspirin2.mpu.data_accel.vect.z);
    VECT3_ASSIGN(imu.mag_unscaled, -mag.y, mag.x, mag.z);
#else

    /* Handle real Aspirin IMU axis assignement. */
#ifdef LISA_M_LONGITUDINAL_X
    RATES_ASSIGN(imu.gyro_unscaled,
                 imu_aspirin2.mpu.data_rates.rates.q,
                 -imu_aspirin2.mpu.data_rates.rates.p,
                 imu_aspirin2.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,
                 imu_aspirin2.mpu.data_accel.vect.y,
                 -imu_aspirin2.mpu.data_accel.vect.x,
                 imu_aspirin2.mpu.data_accel.vect.z);
    VECT3_ASSIGN(imu.mag_unscaled, -mag.x, -mag.y, mag.z);
#else
    RATES_COPY(imu.gyro_unscaled, imu_aspirin2.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_aspirin2.mpu.data_accel.vect);
    VECT3_ASSIGN(imu.mag_unscaled, mag.y, -mag.x, mag.z)
#endif
#endif
#endif

    imu_aspirin2.mpu.data_available = FALSE;
    imu_aspirin2.gyro_valid = TRUE;
    imu_aspirin2.accel_valid = TRUE;
    imu_aspirin2.mag_valid = TRUE;
  }
}

// hack with waiting to avoid creating another event loop to check the mag config status
static inline void mpu_set_and_wait(Mpu60x0ConfigSet mpu_set, void *mpu, uint8_t _reg, uint8_t _val)
{
  mpu_set(mpu, _reg, _val);
  while (imu_aspirin2.mpu.spi_trans.status != SPITransSuccess);
}

/** function to configure hmc5883 mag
 * @return TRUE if mag configuration finished
 */
bool_t imu_aspirin2_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu)
{
  // wait before starting the configuration of the HMC58xx mag
  // doing to early may void the mode configuration
  if (get_sys_time_float() < ASPIRIN_2_MAG_STARTUP_DELAY) {
    return FALSE;
  }

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_CFGA);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_CRA);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_CFGB);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_CRB);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_MODE);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_MD);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_ADDR, (HMC58XX_ADDR >> 1) | MPU60X0_SPI_READ);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_REG, HMC58XX_REG_DATXM);
  // Put the enable command as last.
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_CTRL,
                   (1 << 7) |    // Slave 0 enable
                   (6 << 0));    // Read 6 bytes

  return TRUE;
}

void mpu_wait_slave4_ready(void)
{
  while (!imu_aspirin2.slave4_ready) {
    if (imu_aspirin2.wait_slave4_trans.status == SPITransDone) {
      imu_aspirin2.wait_slave4_tx_buf[0] = MPU60X0_REG_I2C_MST_STATUS | MPU60X0_SPI_READ;
      spi_submit(imu_aspirin2.mpu.spi_p, &(imu_aspirin2.wait_slave4_trans));
    }
  }
}

void mpu_wait_slave4_ready_cb(struct spi_transaction *t)
{
  if (bit_is_set(t->input_buf[1], MPU60X0_I2C_SLV4_DONE)) {
    imu_aspirin2.slave4_ready = TRUE;
  } else {
    imu_aspirin2.slave4_ready = FALSE;
  }
  t->status = SPITransDone;
}
