/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu60x0_spi.c
 *
 * Driver for the MPU-60X0 using SPI.
 *
 */

#include "peripherals/mpu60x0_spi.h"

void mpu60x0_spi_init(struct Mpu60x0_Spi *mpu, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  mpu->spi_p = spi_p;

  /* configure spi transaction */
  mpu->spi_trans.cpol = SPICpolIdleHigh;
  mpu->spi_trans.cpha = SPICphaEdge2;
  mpu->spi_trans.dss = SPIDss8bit;
  mpu->spi_trans.bitorder = SPIMSBFirst;
  mpu->spi_trans.cdiv = SPIDiv64;

  mpu->spi_trans.select = SPISelectUnselect;
  mpu->spi_trans.slave_idx = slave_idx;
  mpu->spi_trans.output_length = 2;
  mpu->spi_trans.input_length = MPU60X0_BUFFER_LEN;
  mpu->spi_trans.before_cb = NULL;
  mpu->spi_trans.after_cb = NULL;
  mpu->spi_trans.input_buf = &(mpu->rx_buf[0]);
  mpu->spi_trans.output_buf = &(mpu->tx_buf[0]);

  /* set inital status: Success or Done */
  mpu->spi_trans.status = SPITransDone;

  /* set default MPU60X0 config options */
  mpu60x0_set_default_config(&(mpu->config));

  mpu->data_available = false;
  mpu->config.initialized = false;
  mpu->config.init_status = MPU60X0_CONF_UNINIT;

  mpu->slave_init_status = MPU60X0_SPI_CONF_UNINIT;
}


static void mpu60x0_spi_write_to_reg(void *mpu, uint8_t _reg, uint8_t _val)
{
  struct Mpu60x0_Spi *mpu_spi = (struct Mpu60x0_Spi *)(mpu);
  mpu_spi->spi_trans.output_length = 2;
  mpu_spi->spi_trans.input_length = 0;
  mpu_spi->tx_buf[0] = _reg;
  mpu_spi->tx_buf[1] = _val;
  spi_submit(mpu_spi->spi_p, &(mpu_spi->spi_trans));
}

// Configuration function called once before normal use
void mpu60x0_spi_start_configure(struct Mpu60x0_Spi *mpu)
{
  if (mpu->config.init_status == MPU60X0_CONF_UNINIT) {

    // First check if we found the chip (succesfull WHO_AM_I response)
    if (mpu->spi_trans.status == SPITransSuccess &&
        (mpu->rx_buf[1] == MPU60X0_WHOAMI_REPLY ||
         mpu->rx_buf[1] == ICM20600_WHOAMI_REPLY ||
         mpu->rx_buf[1] == ICM20608_WHOAMI_REPLY ||
         mpu->rx_buf[1] == ICM20602_WHOAMI_REPLY ||
         mpu->rx_buf[1] == ICM20689_WHOAMI_REPLY)) {

      if (mpu->rx_buf[1] == MPU60X0_WHOAMI_REPLY) {
        mpu->config.type = MPU60X0;
      } else if (mpu->rx_buf[1] == ICM20600_WHOAMI_REPLY) {
        mpu->config.type = ICM20600;
      } else if (mpu->rx_buf[1] == ICM20608_WHOAMI_REPLY) {
        mpu->config.type = ICM20608;
      } else if (mpu->rx_buf[1] == ICM20602_WHOAMI_REPLY) {
        mpu->config.type = ICM20602;
      } else if (mpu->rx_buf[1] == ICM20689_WHOAMI_REPLY) {
        mpu->config.type = ICM20689;
      }

      mpu->config.init_status++;
      mpu->spi_trans.status = SPITransDone;
      mpu60x0_send_config(mpu60x0_spi_write_to_reg, (void *)mpu, &(mpu->config));
    }
    // Send WHO_AM_I to check if chip is there
    else if (mpu->spi_trans.status != SPITransRunning && mpu->spi_trans.status != SPITransPending) {
      mpu->spi_trans.output_length = 1;
      mpu->spi_trans.input_length = 2;
      mpu->tx_buf[0] = MPU60X0_REG_WHO_AM_I | MPU60X0_SPI_READ;
      spi_submit(mpu->spi_p, &(mpu->spi_trans));
    }
  }
}

void mpu60x0_spi_read(struct Mpu60x0_Spi *mpu)
{
  if (mpu->config.initialized && mpu->spi_trans.status == SPITransDone) {
    mpu->spi_trans.output_length = 1;
    mpu->spi_trans.input_length = 1 + mpu->config.nb_bytes;
    /* set read bit and multiple byte bit, then address */
    mpu->tx_buf[0] = MPU60X0_REG_INT_STATUS | MPU60X0_SPI_READ;
    spi_submit(mpu->spi_p, &(mpu->spi_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void mpu60x0_spi_event(struct Mpu60x0_Spi *mpu)
{
  if (mpu->config.initialized) {
    if (mpu->spi_trans.status == SPITransFailed) {
      mpu->spi_trans.status = SPITransDone;
    } else if (mpu->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      if (bit_is_set(mpu->rx_buf[1], 0)) {
        // new data
        mpu->data_accel.vect.x = Int16FromBuf(mpu->rx_buf, 2);
        mpu->data_accel.vect.y = Int16FromBuf(mpu->rx_buf, 4);
        mpu->data_accel.vect.z = Int16FromBuf(mpu->rx_buf, 6);
        mpu->data_rates.rates.p = Int16FromBuf(mpu->rx_buf, 10);
        mpu->data_rates.rates.q = Int16FromBuf(mpu->rx_buf, 12);
        mpu->data_rates.rates.r = Int16FromBuf(mpu->rx_buf, 14);

        int16_t temp_raw = Int16FromBuf(mpu->rx_buf, 8);
        if (mpu->config.type == MPU60X0) {
          mpu->temp = (float)temp_raw / 361.0f + 35.0f;
        } else {
          mpu->temp = (float)temp_raw / 326.8f + 25.0f;
        }

        // if we are reading slaves, copy the ext_sens_data
        if (mpu->config.nb_slaves > 0) {
          /* the buffer is volatile, since filled from ISR
           * but we know it's ok to use it here so we silence the warning
           */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
          memcpy(mpu->data_ext, (uint8_t *) & (mpu->rx_buf[16]), mpu->config.nb_bytes - 15);
#pragma GCC diagnostic pop
        }

        mpu->data_available = true;
      }
      mpu->spi_trans.status = SPITransDone;
    }
  } else if (mpu->config.init_status != MPU60X0_CONF_UNINIT) { // Configuring but not yet initialized
    switch (mpu->spi_trans.status) {
      case SPITransFailed:
        mpu->config.init_status--; // Retry config (TODO max retry)
        /* Falls through. */
      case SPITransSuccess:
      case SPITransDone:
        mpu60x0_send_config(mpu60x0_spi_write_to_reg, (void *)mpu, &(mpu->config));
        if (mpu->config.initialized) {
          mpu->spi_trans.status = SPITransDone;
        }
        break;
      default:
        break;
    }
  }
}

/** configure the registered I2C slaves */
bool mpu60x0_configure_i2c_slaves(Mpu60x0ConfigSet mpu_set, void *mpu)
{
  struct Mpu60x0_Spi *mpu_spi = (struct Mpu60x0_Spi *)(mpu);

  if (mpu_spi->slave_init_status == MPU60X0_SPI_CONF_UNINIT) {
    mpu_spi->slave_init_status++;
  }

  switch (mpu_spi->slave_init_status) {
    case MPU60X0_SPI_CONF_I2C_MST_CLK:
      /* configure MPU I2C master clock and stop/start between slave reads */
      mpu_set(mpu, MPU60X0_REG_I2C_MST_CTRL, ((1 << 4) | mpu_spi->config.i2c_mst_clk));
      mpu_spi->slave_init_status++;
      break;
    case MPU60X0_SPI_CONF_I2C_MST_DELAY:
      /* Set I2C slaves delayed sample rate */
      mpu_set(mpu, MPU60X0_REG_I2C_MST_DELAY, mpu_spi->config.i2c_mst_delay);
      mpu_spi->slave_init_status++;
      break;
    case MPU60X0_SPI_CONF_I2C_MST_EN:
      /* enable internal I2C master and disable primary I2C interface */
      mpu_set(mpu, MPU60X0_REG_USER_CTRL, ((1 << MPU60X0_I2C_IF_DIS) |
                                           (1 << MPU60X0_I2C_MST_EN)));
      mpu_spi->slave_init_status++;
      break;
    case MPU60X0_SPI_CONF_SLAVES_CONFIGURE:
      /* configure each slave until all nb_slaves are done */
      if (mpu_spi->config.nb_slave_init < mpu_spi->config.nb_slaves && mpu_spi->config.nb_slave_init < MPU60X0_I2C_NB_SLAVES) {
         // proceed to next slave if configure for current one returns true
        if (mpu_spi->config.slaves[mpu_spi->config.nb_slave_init].configure(mpu_set, mpu)) {
          mpu_spi->config.nb_slave_init++;
        }
      }
      else {
        /* all slave devies configured, continue MPU side configuration of I2C slave stuff */
        mpu_spi->slave_init_status++;
      }
      break;
    case MPU60X0_SPI_CONF_DONE:
      return true;
    default:
      break;
  }
  return false;
}
