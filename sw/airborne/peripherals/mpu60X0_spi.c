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

static void trans_cb( struct spi_transaction *trans );

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
  mpu->spi_trans.output_length = MPU60X0_BUFFER_LEN; //FIXME
  mpu->spi_trans.input_length = MPU60X0_BUFFER_LEN;
  mpu->spi_trans.before_cb = NULL;
  mpu->spi_trans.after_cb = trans_cb;
  mpu->spi_trans.input_buf = &(mpu->rx_buf[0]);
  mpu->spi_trans.output_buf = &(mpu->tx_buf[0]);

  /* set inital status: Success or Done */
  mpu->spi_trans.status = SPITransDone;

  /* set default MPU60X0 config options */
  mpu60x0_set_default_config(&(mpu->config));

  mpu->initialized = FALSE;
  mpu->data_available = FALSE;
  mpu->init_status = MPU60X0_CONF_UNINIT;
}


static void mpu60x0_spi_write_to_reg(struct Mpu60x0_Spi *mpu, uint8_t _reg, uint8_t _val) {
  mpu->spi_trans.output_length = 2;
  mpu->spi_trans.input_length = 0;
  mpu->tx_buf[0] = _reg;
  mpu->tx_buf[1] = _val;
  spi_submit(mpu->spi_p, &(mpu->spi_trans));
}

// Configuration function called once before normal use
static void mpu60x0_spi_send_config(struct Mpu60x0_Spi *mpu)
{
  switch (mpu->init_status) {
    case MPU60X0_CONF_SD:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_SMPLRT_DIV, mpu->config.smplrt_div);
      mpu->init_status++;
      break;
    case MPU60X0_CONF_CONFIG:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_CONFIG, mpu->config.dlpf_cfg);
      mpu->init_status++;
      break;
    case MPU60X0_CONF_GYRO:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_GYRO_CONFIG, (mpu->config.gyro_range<<3));
      mpu->init_status++;
      break;
    case MPU60X0_CONF_ACCEL:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_ACCEL_CONFIG, (mpu->config.accel_range<<3));
      mpu->init_status++;
      break;
    case MPU60X0_CONF_INT_PIN:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_INT_PIN_CFG, (mpu->config.i2c_bypass<<1));
      mpu->init_status++;
      break;
    case MPU60X0_CONF_INT_ENABLE:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_INT_ENABLE, (mpu->config.drdy_int_enable<<0));
      mpu->init_status++;
      break;
    case MPU60X0_CONF_PWR:
      mpu60x0_spi_write_to_reg(mpu, MPU60X0_REG_PWR_MGMT_1, ((mpu->config.clk_sel)|(0<<6));
      mpu->init_status++;
      break;
    case MPU60X0_CONF_DONE:
      mpu->initialized = TRUE;
      mpu->spi_trans.status = SPITransDone;
      break;
    default:
      break;
  }
}

void mpu60x0_spi_start_configure(struct Mpu60x0_Spi *mpu)
{
  if (mpu->init_status == MPU60X0_CONF_UNINIT) {
    mpu->init_status++;
    if (mpu->spi_trans.status == SPITransSuccess || mpu->spi_trans.status == SPITransDone) {
      mpu60x0_spi_send_config(mpu);
    }
  }
}

void mpu60x0_spi_read(struct Mpu60x0_Spi *mpu)
{
  if (mpu->initialized && mpu->spi_trans.status == SPITransDone) {
    mpu->spi_trans.output_length = 1;
    mpu->spi_trans.input_length = 15; // FIXME external data
    /* set read bit and multiple byte bit, then address */
    mpu->tx_buf[0] = MPU60X0_REG_INT_STATUS + MPU60X0_SPI_READ;
    spi_submit(mpu->spi_p, &(mpu->spi_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void mpu60x0_spi_event(struct Mpu60x0_Spi *mpu)
{
  if (mpu->initialized) {
    if (mpu->spi_trans.status == SPITransFailed) {
      mpu->spi_trans.status = SPITransDone;
    }
    else if (mpu->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      if (bit_is_set(mpu->rx_buf,0)) {
        // new data
        mpu->data_accel.vect.x = Int16FromBuf(mpu->rx_buf,1);
        mpu->data_accel.vect.y = Int16FromBuf(mpu->rx_buf,3);
        mpu->data_accel.vect.z = Int16FromBuf(mpu->rx_buf,5);
        mpu->data_rates.rates.p = Int16FromBuf(mpu->rx_buf,9);
        mpu->data_rates.rates.q = Int16FromBuf(mpu->rx_buf,11);
        mpu->data_rates.rates.r = Int16FromBuf(mpu->rx_buf,13);
        mpu->data_available = TRUE;
      }
      mpu->spi_trans.status = SPITransDone;
    }
  }
  else if (mpu->init_status != MPU60X0_CONF_UNINIT) { // Configuring but not yet initialized
    switch (mpu->spi_trans.status) {
      case SPITransFailed:
        mpu->init_status--; // Retry config (TODO max retry)
      case SPITransSuccess:
      case SPITransDone:
        mpu->spi_trans.status = SPITransDone;
        mpu60x0_spi_send_config(mpu);
        break;
      default:
        break;
    }
  }
}
