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
 * @file peripherals/mpu9250_i2c.c
 *
 * Driver for the MPU-9250 using I2C.
 *
 */

#include "peripherals/mpu9250_i2c.h"

bool imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set __attribute__((unused)),
                                     void *mpu __attribute__((unused)));

void mpu9250_i2c_init(struct Mpu9250_I2c *mpu, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  mpu->i2c_p = i2c_p;

  /* slave address */
  mpu->i2c_trans.slave_addr = addr;
  /* set inital status: Success or Done */
  mpu->i2c_trans.status = I2CTransDone;

  /* set default MPU9250 config options */
  mpu9250_set_default_config(&(mpu->config));

  mpu->data_available = false;
  mpu->config.initialized = false;
  mpu->config.init_status = MPU9250_CONF_UNINIT;

#if IMU_MPU9250_READ_MAG
  /* "internal" ak8963 magnetometer */
  ak8963_init(&mpu->akm, i2c_p, MPU9250_MAG_ADDR);

  /* mag is declared as slave to call the configure function,
   * regardless if it is an actual MPU slave or passthrough
   */
  mpu->config.nb_slaves = 1;
  /* set callback function to configure mag */
  mpu->config.slaves[0].configure = &imu_mpu9250_configure_mag_slave;
  /* read the mag directly for now */
  mpu->config.i2c_bypass = true;

  mpu->slave_init_status = MPU9250_I2C_CONF_UNINIT;
#endif
}


static void mpu9250_i2c_write_to_reg(void *mpu, uint8_t _reg, uint8_t _val)
{
  struct Mpu9250_I2c *mpu_i2c = (struct Mpu9250_I2c *)(mpu);
  mpu_i2c->i2c_trans.buf[0] = _reg;
  mpu_i2c->i2c_trans.buf[1] = _val;
  i2c_transmit(mpu_i2c->i2c_p, &(mpu_i2c->i2c_trans), mpu_i2c->i2c_trans.slave_addr, 2);
}

// Configuration function called once before normal use
void mpu9250_i2c_start_configure(struct Mpu9250_I2c *mpu)
{
  if (mpu->config.init_status == MPU9250_CONF_UNINIT) {
    mpu->config.init_status++;
    if (mpu->i2c_trans.status == I2CTransSuccess || mpu->i2c_trans.status == I2CTransDone) {
      mpu9250_send_config(mpu9250_i2c_write_to_reg, (void *)mpu, &(mpu->config));
    }
  }
}

void mpu9250_i2c_read(struct Mpu9250_I2c *mpu)
{
  if (mpu->config.initialized && mpu->i2c_trans.status == I2CTransDone) {
    /* set read bit and multiple byte bit, then address */
    mpu->i2c_trans.buf[0] = MPU9250_REG_INT_STATUS;
    i2c_transceive(mpu->i2c_p, &(mpu->i2c_trans), mpu->i2c_trans.slave_addr, 1, mpu->config.nb_bytes);
    /* read mag */
#if IMU_MPU9250_READ_MAG
#ifdef MPU9250_MAG_PRESCALER
    RunOnceEvery(MPU9250_MAG_PRESCALER, ak8963_read(&mpu->akm));
#else
    ak8963_read(&mpu->akm);
#endif
#endif
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void mpu9250_i2c_event(struct Mpu9250_I2c *mpu)
{
  if (mpu->config.initialized) {
    if (mpu->i2c_trans.status == I2CTransFailed) {
      mpu->i2c_trans.status = I2CTransDone;
    } else if (mpu->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      if (bit_is_set(mpu->i2c_trans.buf[0], 0)) {
        // new data
        mpu->data_accel.vect.x = Int16FromBuf(mpu->i2c_trans.buf, 1);
        mpu->data_accel.vect.y = Int16FromBuf(mpu->i2c_trans.buf, 3);
        mpu->data_accel.vect.z = Int16FromBuf(mpu->i2c_trans.buf, 5);
        mpu->data_rates.rates.p = Int16FromBuf(mpu->i2c_trans.buf, 9);
        mpu->data_rates.rates.q = Int16FromBuf(mpu->i2c_trans.buf, 11);
        mpu->data_rates.rates.r = Int16FromBuf(mpu->i2c_trans.buf, 13);

        // if we are reading slaves through the mpu, copy the ext_sens_data
        if ((mpu->config.i2c_bypass == FALSE) && (mpu->config.nb_slaves > 0)) {
          /* the buffer is volatile, since filled from ISR
           * but we know it's ok to use it here so we silence the warning
           */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
          memcpy(mpu->data_ext, (uint8_t *) & (mpu->i2c_trans.buf[15]), mpu->config.nb_bytes - 15);
#pragma GCC diagnostic pop
        }

        mpu->data_available = true;
      }
      mpu->i2c_trans.status = I2CTransDone;
    }
  } else if (mpu->config.init_status != MPU9250_CONF_UNINIT) { // Configuring but not yet initialized
    switch (mpu->i2c_trans.status) {
      case I2CTransFailed:
        mpu->config.init_status--; // Retry config (TODO max retry)
        /* Falls through. */
      case I2CTransSuccess:
      case I2CTransDone:
        mpu9250_send_config(mpu9250_i2c_write_to_reg, (void *)mpu, &(mpu->config));
        if (mpu->config.initialized) {
          mpu->i2c_trans.status = I2CTransDone;
        }
        break;
      default:
        break;
    }
  }
#if IMU_MPU9250_READ_MAG
  // Ak8963 event function
  ak8963_event(&mpu->akm);
#endif
}

/** callback function to configure ak8963 mag
 * @return TRUE if mag configuration finished
 */
bool imu_mpu9250_configure_mag_slave(Mpu9250ConfigSet mpu_set __attribute__((unused)), void *mpu)
{
  struct Mpu9250_I2c *mpu_i2c = (struct Mpu9250_I2c *)(mpu);

  ak8963_configure(&mpu_i2c->akm);
  if (mpu_i2c->akm.initialized) {
    return true;
  } else {
    return false;
  }
}

/** configure the registered I2C slaves */
bool mpu9250_configure_i2c_slaves(Mpu9250ConfigSet mpu_set, void *mpu)
{
  struct Mpu9250_I2c *mpu_i2c = (struct Mpu9250_I2c *)(mpu);

  if (mpu_i2c->slave_init_status == MPU9250_I2C_CONF_UNINIT) {
    mpu_i2c->slave_init_status++;
  }

  switch (mpu_i2c->slave_init_status) {
    case MPU9250_I2C_CONF_I2C_MST_DIS:
      mpu_set(mpu, MPU9250_REG_USER_CTRL, 0);
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_I2C_BYPASS_EN:
      /* switch to I2C passthrough */
      mpu_set(mpu, MPU9250_REG_INT_PIN_CFG, (1 << 1));
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_SLAVES_CONFIGURE:
      /* configure each slave until all nb_slaves are done */
      if (mpu_i2c->config.nb_slave_init < mpu_i2c->config.nb_slaves && mpu_i2c->config.nb_slave_init < MPU9250_I2C_NB_SLAVES) {
        // proceed to next slave if configure for current one returns true
        if (mpu_i2c->config.slaves[mpu_i2c->config.nb_slave_init].configure(mpu_set, mpu)) {
          mpu_i2c->config.nb_slave_init++;
        }
      } else {
        /* all slave devies configured, continue MPU side configuration of I2C slave stuff */
        mpu_i2c->slave_init_status++;
      }
      break;
    case MPU9250_I2C_CONF_I2C_BYPASS_DIS:
      if (mpu_i2c->config.i2c_bypass) {
        /* if bypassing I2C skip MPU I2C master setup */
        mpu_i2c->slave_init_status = MPU9250_I2C_CONF_DONE;
      } else {
        /* disable I2C passthrough again */
        mpu_set(mpu, MPU9250_REG_INT_PIN_CFG, (0 << 1));
        mpu_i2c->slave_init_status++;
      }
      break;
    case MPU9250_I2C_CONF_I2C_MST_CLK:
      /* configure MPU I2C master clock and stop/start between slave reads */
      mpu_set(mpu, MPU9250_REG_I2C_MST_CTRL,
              ((1 << 4) | mpu_i2c->config.i2c_mst_clk));
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_I2C_MST_DELAY:
      /* Set I2C slaves delayed sample rate */
      mpu_set(mpu, MPU9250_REG_I2C_MST_DELAY, mpu_i2c->config.i2c_mst_delay);
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_I2C_SMPLRT:
      /* I2C slave0 sample rate/2 = 100/2 = 50Hz */
      mpu_set(mpu, MPU9250_REG_I2C_SLV4_CTRL, 0);
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_I2C_MST_EN:
      /* enable internal I2C master */
      mpu_set(mpu, MPU9250_REG_USER_CTRL, (1 << MPU9250_I2C_MST_EN));
      mpu_i2c->slave_init_status++;
      break;
    case MPU9250_I2C_CONF_DONE:
      return true;
    default:
      break;
  }
  return false;
}
