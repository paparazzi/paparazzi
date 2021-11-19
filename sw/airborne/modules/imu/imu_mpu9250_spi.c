/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/imu/imu_mpu9250_spi.c
 *
 * IMU driver for the MPU9250 using SPI
 *
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"
#include "peripherals/ak8963_regs.h"

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_MPU9250_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_MPU9250_SPI_DEV)


#if !defined IMU_MPU9250_GYRO_LOWPASS_FILTER && !defined IMU_MPU9250_ACCEL_LOWPASS_FILTER && !defined  IMU_MPU9250_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 41Hz, Delay 5.9ms
 * Gyroscope: Bandwidth 41Hz, Delay 5.9ms sampling 1kHz
 * Output rate: 100Hz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
#define IMU_MPU9250_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 184Hz, Delay 5.8ms
 * Gyroscope: Bandwidth 250Hz, Delay 0.97ms sampling 8kHz
 * Output rate: 2kHz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_250HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_184HZ
#define IMU_MPU9250_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
/* By default, don't go too fast */
#define IMU_MPU9250_SMPLRT_DIV 9
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_SMPLRT_DIV)
PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_LOWPASS_FILTER)

PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_RANGE)
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_RANGE)

// Default channels order
#ifndef IMU_MPU9250_CHAN_X
#define IMU_MPU9250_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_X)
#ifndef IMU_MPU9250_CHAN_Y
#define IMU_MPU9250_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Y)
#ifndef IMU_MPU9250_CHAN_Z
#define IMU_MPU9250_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Z)

#ifndef IMU_MPU9250_X_SIGN
#define IMU_MPU9250_X_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_X_SIGN)
#ifndef IMU_MPU9250_Y_SIGN
#define IMU_MPU9250_Y_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_Y_SIGN)
#ifndef IMU_MPU9250_Z_SIGN
#define IMU_MPU9250_Z_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_Z_SIGN)

#ifndef IMU_MPU9250_READ_MAG
#define IMU_MPU9250_READ_MAG TRUE
#endif

#ifndef IMU_MPU9250_MAG_STARTUP_DELAY
#define IMU_MPU9250_MAG_STARTUP_DELAY 1
#endif

struct ImuMpu9250 imu_mpu9250;

void mpu_wait_slave4_ready(void);
void mpu_wait_slave4_ready_cb(struct spi_transaction *t);
bool imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set, void *mpu);

void imu_mpu9250_init(void)
{
  /* MPU9250 */
  mpu9250_spi_init(&imu_mpu9250.mpu, &(IMU_MPU9250_SPI_DEV), IMU_MPU9250_SPI_SLAVE_IDX);
  // change the default configuration
  imu_mpu9250.mpu.config.smplrt_div = IMU_MPU9250_SMPLRT_DIV;
  imu_mpu9250.mpu.config.dlpf_gyro_cfg = IMU_MPU9250_GYRO_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.dlpf_accel_cfg = IMU_MPU9250_ACCEL_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.gyro_range = IMU_MPU9250_GYRO_RANGE;
  imu_mpu9250.mpu.config.accel_range = IMU_MPU9250_ACCEL_RANGE;


  /* "internal" ak8963 magnetometer as I2C slave */
#if IMU_MPU9250_READ_MAG
  /* read 15 bytes for status, accel, gyro + 7 bytes for mag slave */
  imu_mpu9250.mpu.config.nb_bytes = 22;
  imu_mpu9250.mpu.config.nb_slaves = 1;
#endif
  /* set callback function to configure mag */
  imu_mpu9250.mpu.config.slaves[0].configure = &imu_mpu9250_configure_mag_slave;

  /* Set MPU I2C master clock */
  imu_mpu9250.mpu.config.i2c_mst_clk = MPU9250_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_mpu9250.mpu.config.i2c_mst_delay = 1;


  /* configure spi transaction for wait_slave4 */
  imu_mpu9250.wait_slave4_trans.cpol = SPICpolIdleHigh;
  imu_mpu9250.wait_slave4_trans.cpha = SPICphaEdge2;
  imu_mpu9250.wait_slave4_trans.dss = SPIDss8bit;
  imu_mpu9250.wait_slave4_trans.bitorder = SPIMSBFirst;
  imu_mpu9250.wait_slave4_trans.cdiv = SPIDiv64;

  imu_mpu9250.wait_slave4_trans.select = SPISelectUnselect;
  imu_mpu9250.wait_slave4_trans.slave_idx = IMU_MPU9250_SPI_SLAVE_IDX;
  imu_mpu9250.wait_slave4_trans.output_length = 1;
  imu_mpu9250.wait_slave4_trans.input_length = 2;
  imu_mpu9250.wait_slave4_trans.before_cb = NULL;
  imu_mpu9250.wait_slave4_trans.after_cb = mpu_wait_slave4_ready_cb;
  imu_mpu9250.wait_slave4_trans.input_buf = &(imu_mpu9250.wait_slave4_rx_buf[0]);
  imu_mpu9250.wait_slave4_trans.output_buf = &(imu_mpu9250.wait_slave4_tx_buf[0]);

  imu_mpu9250.wait_slave4_trans.status = SPITransDone;
  imu_mpu9250.slave4_ready = false;
}

void imu_mpu9250_periodic(void)
{
  mpu9250_spi_periodic(&imu_mpu9250.mpu);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))
void imu_mpu9250_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU9250 SPI transaction has succeeded: convert the data
  mpu9250_spi_event(&imu_mpu9250.mpu);

  if (imu_mpu9250.mpu.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      IMU_MPU9250_X_SIGN * (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_X]),
      IMU_MPU9250_Y_SIGN * (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Y]),
      IMU_MPU9250_Z_SIGN * (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Z])
    };
    struct Int32Rates rates = {
      IMU_MPU9250_X_SIGN * (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_X]),
      IMU_MPU9250_Y_SIGN * (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Y]),
      IMU_MPU9250_Z_SIGN * (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Z])
    };
    // unscaled vector
    VECT3_COPY(imu.accel_unscaled, accel);
    RATES_COPY(imu.gyro_unscaled, rates);

#if IMU_MPU9250_READ_MAG
    if (!bit_is_set(imu_mpu9250.mpu.data_ext[6], 3)) { //mag valid just HOFL == 0
      /** FIXME: assumes that we get new mag data each time instead of reading drdy bit */
      struct Int32Vect3 mag;
      mag.x =  (IMU_MPU9250_X_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_Y);
      mag.y =  (IMU_MPU9250_Y_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_X);
      mag.z = -(IMU_MPU9250_Z_SIGN) * Int16FromBuf(imu_mpu9250.mpu.data_ext, 2 * IMU_MPU9250_CHAN_Z);
      VECT3_COPY(imu.mag_unscaled, mag);
      imu_scale_mag(&imu);
      AbiSendMsgIMU_MAG_INT32(IMU_MPU9250_ID, now_ts, &imu.mag);
    }
#endif

    imu_mpu9250.mpu.data_available = false;

    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU9250_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU9250_ID, now_ts, &imu.accel);
  }

}

// hack with waiting to avoid creating another event loop to check the mag config status
static inline void mpu_set_and_wait(Mpu9250ConfigSet mpu_set, void *mpu, uint8_t _reg, uint8_t _val)
{
  mpu_set(mpu, _reg, _val);
  while (imu_mpu9250.mpu.spi_trans.status != SPITransSuccess);
}

/** function to configure akm8963 mag
 * @return TRUE if mag configuration finished
 */
bool imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set, void *mpu)
{
  // wait before starting the configuration of the mag
  // doing to early may void the mode configuration
  if (get_sys_time_float() < IMU_MPU9250_MAG_STARTUP_DELAY) {
    return false;
  }

  //config AK8963 soft reset
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_ADDR, (MPU9250_MAG_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_REG, AK8963_REG_CNTL2);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_DO, 1);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  // Set it to continious measuring mode 2
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_ADDR, (MPU9250_MAG_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_REG, AK8963_REG_CNTL1);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_DO, AK8963_CNTL1_CM_2);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  //Config SLV0 for continus Read
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_ADDR, (MPU9250_MAG_ADDR >> 1) | MPU9250_SPI_READ);
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_HXL);
  // Put the enable command as last.
  mpu_set_and_wait(mpu_set, mpu, MPU9250_REG_I2C_SLV0_CTRL,
                   (1 << 7) |    // Slave 0 enable
                   (7 << 0));    // Read 7 bytes (mag x,y,z + status)

  return true;
}

void mpu_wait_slave4_ready(void)
{
  while (!imu_mpu9250.slave4_ready) {
    if (imu_mpu9250.wait_slave4_trans.status == SPITransDone) {
      imu_mpu9250.wait_slave4_tx_buf[0] = MPU9250_REG_I2C_MST_STATUS | MPU9250_SPI_READ;
      spi_submit(imu_mpu9250.mpu.spi_p, &(imu_mpu9250.wait_slave4_trans));
    }
  }
}

void mpu_wait_slave4_ready_cb(struct spi_transaction *t)
{
  if (bit_is_set(t->input_buf[1], MPU9250_I2C_SLV4_DONE)) {
    imu_mpu9250.slave4_ready = true;
  } else {
    imu_mpu9250.slave4_ready = false;
  }
  t->status = SPITransDone;
}
