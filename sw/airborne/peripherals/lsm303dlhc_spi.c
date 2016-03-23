/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lsm303dlhc_spi.c
 *
 * Driver for ST LSM303DLHC 3D accelerometer and magnetometer.
 */

#include "peripherals/lsm303dlhc_spi.h"
#include "std.h"

void lsm303dlhc_spi_init(struct Lsm303dlhc_Spi *lsm, struct spi_periph *spi_p, uint8_t slave_idx,
                         enum Lsm303dlhcTarget target)
{
  /* set spi_peripheral */
  lsm->spi_p = spi_p;

  /* set internal target mag/acc*/
  lsm->target = target;

  /* configure spi transaction */
  lsm->spi_trans.cpol = SPICpolIdleHigh;
  lsm->spi_trans.cpha = SPICphaEdge2;
  lsm->spi_trans.dss = SPIDss8bit;
  lsm->spi_trans.bitorder = SPIMSBFirst;
  lsm->spi_trans.cdiv = SPIDiv64;

  lsm->spi_trans.select = SPISelectUnselect;
  lsm->spi_trans.slave_idx = slave_idx;
  lsm->spi_trans.output_length = 2;
  lsm->spi_trans.input_length = 8;
  // callback currently unused
  lsm->spi_trans.before_cb = NULL;
  lsm->spi_trans.after_cb = NULL;
  lsm->spi_trans.input_buf = &(lsm->rx_buf[0]);
  lsm->spi_trans.output_buf = &(lsm->tx_buf[0]);

  /* set inital status: Success or Done */
  lsm->spi_trans.status = SPITransDone;

  /* set default LSM303D config options */
  lsm303dlhc_acc_set_default_config(&(lsm->config.acc));
  lsm303dlhc_mag_set_default_config(&(lsm->config.mag));
  lsm->init_status = LSM_CONF_UNINIT;

  lsm->initialized = FALSE;
  lsm->data_available_acc = FALSE;
  lsm->data_available_mag = FALSE;
}

static void lsm303dlhc_spi_tx_reg(struct Lsm303dlhc_Spi *lsm, uint8_t reg, uint8_t val)
{
  lsm->spi_trans.output_length = 2;
  lsm->spi_trans.input_length = 0;
  lsm->tx_buf[0] = reg;
  lsm->tx_buf[1] = val;
  spi_submit(lsm->spi_p, &(lsm->spi_trans));
}

/// Configuration function called once before normal use
static void lsm303dlhc_spi_send_config(struct Lsm303dlhc_Spi *lsm)
{
  if (lsm->target ==
      LSM_TARGET_ACC) { // the complete config done below currently is one shot for both acc and mag. So, only do it for one of the devices.
    switch (lsm->init_status) {
      case LSM_CONF_WHO_AM_I:
        /* query device id */
        lsm->spi_trans.output_length = 1;
        lsm->spi_trans.input_length = 2;
        /* set read bit then reg address */
        lsm->tx_buf[0] = (1 << 7 | LSM303DLHC_REG_WHO_AM_I);
        if (spi_submit(lsm->spi_p, &(lsm->spi_trans))) {
          if (lsm->rx_buf[1] == LSM303DLHC_WHO_I_AM) {
            lsm->init_status++;
          }
        }
        break;
      case LSM_CONF_CTRL_REG1:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL1,
                              (lsm->config.acc.rate & LSM303DLHC_AODR_MASK) |
                              LSM303DLHC_Xen | LSM303DLHC_Yen | LSM303DLHC_Zen);
        lsm->init_status++;
        break;
      case LSM_CONF_CTRL_REG2:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL2, (lsm->config.acc.scale & LSM303DLHC_FS_MASK));
        lsm->init_status++;
        break;
      case LSM_CONF_CTRL_REG3:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL3, LSM303DLHC_I1_DRDY_A);
        lsm->init_status++;
        break;
      case LSM_CONF_CTRL_REG4:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL4, LSM303DLHC_I2_DRDY_M);
        lsm->init_status++;
        return;
        break;
      case LSM_CONF_CTRL_REG5:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL5,
                              (lsm->config.mag.rate & LSM303DLHC_M_ODR_MASK));
        lsm->init_status++;
        return;
        break;
      case LSM_CONF_CTRL_REG6:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL6,
                              (lsm->config.mag.scale & LSM303DLHC_MFS_MASK));
        lsm->init_status++;
        break;
      case LSM_CONF_CTRL_REG7:
        lsm303dlhc_spi_tx_reg(lsm, LSM303DLHC_REG_CTRL7, (lsm->config.mag.mode & LSM303DLHC_AHPM_MASK));
        lsm->init_status++;
        break;
      case LSM_CONF_DONE:
        lsm->initialized = TRUE;
        lsm->spi_trans.status = SPITransDone;
        return;
        break;
      default:
        break;
    }
  } else {
    lsm->initialized = TRUE;
    lsm->spi_trans.status = SPITransDone;
  }
}

// Configure
void lsm303dlhc_spi_start_configure(struct Lsm303dlhc_Spi *lsm)
{
  if (lsm->init_status == LSM_CONF_UNINIT) {
    lsm->init_status++;
    if (lsm->spi_trans.status == SPITransSuccess || lsm->spi_trans.status == SPITransDone) {
      lsm303dlhc_spi_send_config(lsm);
    }
  }
}

// Normal reading
void lsm303dlhc_spi_read(struct Lsm303dlhc_Spi *lsm)
{
  if (lsm->target == LSM_TARGET_ACC) {
    if (!(lsm->initialized) || (lsm->initialized && lsm->spi_trans.status == SPITransDone)) {
      lsm->spi_trans.output_length = 1;
      lsm->spi_trans.input_length = 8;
      /* set read bit and multiple byte bit, then address */
      lsm->tx_buf[0] = LSM303DLHC_REG_STATUS_REG_A | 1 << 7 | 1 << 6;
      spi_submit(lsm->spi_p, &(lsm->spi_trans));
    }
  } else {
    if (lsm->initialized && lsm->spi_trans.status == SPITransDone) {
      lsm->spi_trans.output_length = 1;
      lsm->spi_trans.input_length = 8;
      /* set read bit and multiple byte bit, then address */
      lsm->tx_buf[0] = LSM303DLHC_REG_STATUS_REG_M | 1 << 7 | 1 << 6;
      spi_submit(lsm->spi_p, &(lsm->spi_trans));
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lsm303dlhc_spi_event(struct Lsm303dlhc_Spi *lsm)
{
  if (lsm->initialized) {
    if (lsm->spi_trans.status == SPITransFailed) {
      lsm->spi_trans.status = SPITransDone;
    } else if (lsm->spi_trans.status == SPITransSuccess) {
      if (lsm->target == LSM_TARGET_ACC) {
        if (!(lsm->rx_buf[1] & LSM303DLHC_REG_STATUS_ZYXADA)) {
          lsm->spi_trans.status = SPITransDone;
          return;
        }
        lsm->data_accel.vect.x = Int16FromBuf(lsm->rx_buf, 2);
        lsm->data_accel.vect.y = Int16FromBuf(lsm->rx_buf, 4);
        lsm->data_accel.vect.z = Int16FromBuf(lsm->rx_buf, 6);
        lsm->data_available_acc = TRUE;
        lsm->spi_trans.status = SPITransDone;
      } else { //magneto
        if (!(lsm->rx_buf[1] & LSM303DLHC_REG_STATUS_ZYXMDA)) {
          lsm->spi_trans.status = SPITransDone;
          return;
        }
        lsm->data_mag.vect.x = Int16FromBuf(lsm->rx_buf, 2);
        lsm->data_mag.vect.y = Int16FromBuf(lsm->rx_buf, 4);
        lsm->data_mag.vect.z = Int16FromBuf(lsm->rx_buf, 6);
        lsm->data_available_mag = TRUE;
        lsm->spi_trans.status = SPITransDone;
      }
    }
  } else {
    if (lsm->init_status != LSM_CONF_UNINIT) { // Configuring but not yet initialized
      if (lsm->spi_trans.status == SPITransSuccess || lsm->spi_trans.status == SPITransDone) {
        lsm->spi_trans.status = SPITransDone;
        lsm303dlhc_spi_send_config(lsm);
      }
      if (lsm->spi_trans.status == SPITransFailed) {
        lsm->init_status--;
        lsm->spi_trans.status = SPITransDone;
        lsm303dlhc_spi_send_config(lsm); // Retry config (TODO max retry)
      }
    }
  }
}
