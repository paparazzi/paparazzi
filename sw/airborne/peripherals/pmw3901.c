/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
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
 * @file "modules/peripherals/pmw3901.c"
 * @author Tom van Dijk
 * Low-level driver for PMW3901 optical flow sensor
 *
 * Code based on the PixArt reference firmware
 * https://os.mbed.com/teams/PixArt/code/3901_referenceFirmware//file/10365086d44e/commHeaders/SPIcommFunctions.h/
 *
 */

#include "pmw3901.h"

#include "mcu_periph/sys_time.h"


// Based on crazyflie-firmware
#ifndef PMW3901_RAD_PER_PX
#define PMW3901_RAD_PER_PX 0.002443389
#endif

// SPI divisor, to adjust the clock speed according to the PCLK
// Don't exceed 2MHz
#ifndef PMW3901_SPI_CDIV
#define PMW3901_SPI_CDIV SPIDiv256
#endif

#define PMW3901_REG_MOTION     0x02
#define PMW3901_REG_DELTA_X_L  0x03
#define PMW3901_REG_DELTA_X_H  0x04
#define PMW3901_REG_DELTA_Y_L  0x05
#define PMW3901_REG_DELTA_Y_H  0x06


// Non-blocking read function
// returns true upon completion
static bool readRegister_nonblocking(struct pmw3901_t *pmw, uint8_t addr, uint8_t *value) {
  switch (pmw->readwrite_state) {
    case 0:
      if (get_sys_time_usec() < pmw->readwrite_timeout) return false;
      pmw->trans.output_buf[0] = addr & 0x7F;  // MSB 0 => read
      pmw->trans.output_length = 1;
      pmw->trans.input_length = 0;
      pmw->trans.select = SPISelect;
      spi_submit(pmw->periph, &pmw->trans);
      pmw->readwrite_state++;
      /* Falls through. */
    case 1:
      if (pmw->trans.status == SPITransPending || pmw->trans.status == SPITransRunning) return false;
      // Write addr complete
      pmw->readwrite_timeout = get_sys_time_usec() + 35;
      pmw->readwrite_state++;
      /* Falls through. */
    case 2:
      if (get_sys_time_usec() < pmw->readwrite_timeout) return false;
      // Addr-read delay passed
      pmw->trans.output_length = 0;
      pmw->trans.input_length = 1;
      pmw->trans.select = SPIUnselect;
      spi_submit(pmw->periph, &pmw->trans);
      pmw->readwrite_state++;
      /* Falls through. */
    case 3:
      if (pmw->trans.status == SPITransPending || pmw->trans.status == SPITransRunning) return false;
      // Read complete
      pmw->trans.select = SPISelectUnselect;
      *value = pmw->trans.input_buf[0];
      pmw->readwrite_timeout = get_sys_time_usec() + 20;
      pmw->readwrite_state = 0;
      return true;
    default: return false;
  }
}


// Blocking read/write functions
static uint8_t readRegister_blocking(struct pmw3901_t *pmw, uint8_t addr) {
  pmw->trans.output_buf[0] = addr & 0x7F;  // MSB 0 => read
  pmw->trans.output_length = 1;
  pmw->trans.input_length = 0;
  pmw->trans.select = SPISelect;
  spi_blocking_transceive(pmw->periph, &pmw->trans);
  sys_time_usleep(35);  // See ref firmware and datasheet
  pmw->trans.output_length = 0;
  pmw->trans.input_length = 1;
  pmw->trans.select = SPIUnselect;
  spi_blocking_transceive(pmw->periph, &pmw->trans);
  pmw->trans.select = SPISelectUnselect;
  return pmw->trans.input_buf[0];
}

static void writeRegister_blocking(struct pmw3901_t *pmw, uint8_t addr, uint8_t data) {
  pmw->trans.output_buf[0] = addr | 0x80;  // MSB 1 => write
  pmw->trans.output_buf[1] = data;
  pmw->trans.output_length = 2;
  pmw->trans.input_length = 0;
  spi_blocking_transceive(pmw->periph, &pmw->trans);
}

// For PixArt firmware compatibility:
#define writeRegister(_addr, _data)  writeRegister_blocking(pmw, (_addr), (_data))
#define readRegister(_addr)  readRegister_blocking(pmw, (_addr))
#define wait_ms(_ms)  sys_time_usleep((_ms) * 1000)


static void initializeSensor(struct pmw3901_t *pmw) {
  // Try to detect sensor before initializing
  int tries = 0;
  while (readRegister(0x00) != 0x49 && tries < 100) {
    sys_time_usleep(50);
    tries++;
  }

  // From reference firmware
  writeRegister(0x7F, 0x00);
  writeRegister(0x55, 0x01);
  writeRegister(0x50, 0x07);
  writeRegister(0x7F, 0x0E);
  writeRegister(0x43, 0x10);

  if (readRegister(0x67) & 0x40)
    writeRegister(0x48, 0x04);

  else
    writeRegister(0x48, 0x02);

  writeRegister(0x7F, 0x00);
  writeRegister(0x51, 0x7B);
  writeRegister(0x50, 0x00);
  writeRegister(0x55, 0x00);
  writeRegister(0x7F, 0x0E);

  if (readRegister(0x73) == 0x00) {
    writeRegister(0x7F, 0x00);
    writeRegister(0x61, 0xAD);
    writeRegister(0x51, 0x70);
    writeRegister(0x7F, 0x0E);

    if (readRegister(0x70) <= 28)
      writeRegister(0x70, readRegister(0x70) + 14);

    else
      writeRegister(0x70, readRegister(0x70) + 11);

    writeRegister(0x71, readRegister(0x71) * 45/100);
  }

  writeRegister(0x7F, 0x00);
  writeRegister(0x61, 0xAD);
  writeRegister(0x7F, 0x03);
  writeRegister(0x40, 0x00);
  writeRegister(0x7F, 0x05);
  writeRegister(0x41, 0xB3);
  writeRegister(0x43, 0xF1);
  writeRegister(0x45, 0x14);
  writeRegister(0x5B, 0x32);
  writeRegister(0x5F, 0x34);
  writeRegister(0x7B, 0x08);
  writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B);
  writeRegister(0x40, 0xBF);
  writeRegister(0x4E, 0x3F);
  writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B);
  writeRegister(0x40, 0xBF);
  writeRegister(0x4E, 0x3F);
  writeRegister(0x7F, 0x08);
  writeRegister(0x65, 0x20);
  writeRegister(0x6A, 0x18);
  writeRegister(0x7F, 0x09);
  writeRegister(0x4F, 0xAF);
  writeRegister(0x5F, 0x40);
  writeRegister(0x48, 0x80);
  writeRegister(0x49, 0x80);
  writeRegister(0x57, 0x77);
  writeRegister(0x60, 0x78);
  writeRegister(0x61, 0x78);
  writeRegister(0x62, 0x08);
  writeRegister(0x63, 0x50);
  writeRegister(0x7F, 0x0A);
  writeRegister(0x45, 0x60);
  writeRegister(0x7F, 0x00);
  writeRegister(0x4D, 0x11);
  writeRegister(0x55, 0x80);
  writeRegister(0x74, 0x21);
  writeRegister(0x75, 0x1F);
  writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78);
  writeRegister(0x44, 0x08);
  writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF);
  writeRegister(0x65, 0x1F);
  writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x67);
  writeRegister(0x66, 0x08);
  writeRegister(0x63, 0x70);
  writeRegister(0x7F, 0x15);
  writeRegister(0x48, 0x48);
  writeRegister(0x7F, 0x07);
  writeRegister(0x41, 0x0D);
  writeRegister(0x43, 0x14);
  writeRegister(0x4B, 0x0E);
  writeRegister(0x45, 0x0F);
  writeRegister(0x44, 0x42);
  writeRegister(0x4C, 0x80);
  writeRegister(0x7F, 0x10);
  writeRegister(0x5B, 0x02);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x41);
  writeRegister(0x70, 0x00);

  wait_ms(10);

  writeRegister(0x32, 0x44);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06);
  writeRegister(0x62, 0xF0);
  writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D);
  writeRegister(0x48, 0xC0);
  writeRegister(0x6F, 0xD5);
  writeRegister(0x7F, 0x00);
  writeRegister(0x5B, 0xA0);
  writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50);
  writeRegister(0x40, 0x80);
}


void pmw3901_init(struct pmw3901_t *pmw, struct spi_periph *periph, uint8_t slave_idx) {
  // Set up SPI peripheral and transaction
  pmw->periph = periph;
  pmw->trans.input_buf = pmw->spi_input_buf;
  pmw->trans.output_buf = pmw->spi_output_buf;
  pmw->trans.slave_idx = slave_idx;
  pmw->trans.select = SPISelectUnselect;
  pmw->trans.cpol = SPICpolIdleLow;
  pmw->trans.cpha = SPICphaEdge1;
  pmw->trans.dss = SPIDss8bit;
  pmw->trans.bitorder = SPIMSBFirst;
  pmw->trans.cdiv = PMW3901_SPI_CDIV;
  pmw->trans.before_cb = NULL;
  pmw->trans.after_cb = NULL;
  pmw->trans.status = SPITransDone;
  // Initialize sensor registers
  initializeSensor(pmw);
  // Set up remaining fields
  pmw->state = PMW3901_IDLE;
  pmw->delta_x = 0;
  pmw->delta_y = 0;
  pmw->data_available = false;
  pmw->rad_per_px = PMW3901_RAD_PER_PX;
}

void pmw3901_event(struct pmw3901_t *pmw) {
  uint8_t temp;
  switch (pmw->state) {
    case PMW3901_IDLE:
      /* Do nothing */
      return;
    case PMW3901_READ_MOTION:
      if (!readRegister_nonblocking(pmw, PMW3901_REG_MOTION, &temp)) return;
      if (!(temp & 0x80)) return;
      pmw->delta_x = 0;
      pmw->delta_y = 0;
      pmw->state++;
      /* Falls through. */
    case PMW3901_READ_DELTAXLOW:
      if (!readRegister_nonblocking(pmw, PMW3901_REG_DELTA_X_L, &temp)) return;
      pmw->delta_x |= temp;
      pmw->state++;
      /* Falls through. */
    case PMW3901_READ_DELTAXHIGH:
      if (!readRegister_nonblocking(pmw, PMW3901_REG_DELTA_X_H, &temp)) return;
      pmw->delta_x |= (temp << 8) & 0xFF00;
      pmw->state++;
      /* Falls through. */
    case PMW3901_READ_DELTAYLOW:
      if (!readRegister_nonblocking(pmw, PMW3901_REG_DELTA_Y_L, &temp)) return;
      pmw->delta_y |= temp;
      pmw->state++;
          /* Falls through. */
    case PMW3901_READ_DELTAYHIGH:
      if (!readRegister_nonblocking(pmw, PMW3901_REG_DELTA_Y_H, &temp)) return;
      pmw->delta_y |= (temp << 8) & 0xFF00;
      pmw->data_available = true;
      pmw->state = PMW3901_IDLE;
      return;
    default: return;
  }
}

bool pmw3901_is_idle(struct pmw3901_t *pmw) {
  return pmw->state == PMW3901_IDLE;
}

void pmw3901_start_read(struct pmw3901_t *pmw) {
  if (pmw3901_is_idle(pmw)) {
    pmw->state = PMW3901_READ_MOTION;
  }
}

bool pmw3901_data_available(struct pmw3901_t *pmw) {
  return pmw->data_available;
}

bool pmw3901_get_data(struct pmw3901_t *pmw, int16_t *delta_x, int16_t *delta_y) {
  if (!pmw->data_available) return false;
  *delta_x = pmw->delta_x;
  *delta_y = pmw->delta_y;
  pmw->data_available = false;
  return true;
}
