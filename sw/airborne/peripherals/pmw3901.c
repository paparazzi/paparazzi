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
#include "mcu_periph/gpio.h"


// Based on crazyflie-firmware
#ifndef PMW3901_RAD_PER_PX
#define PMW3901_RAD_PER_PX 0.002443389
#endif

// SPI divisor, to adjust the clock speed according to the PCLK
// Don't exceed 2MHz
#ifndef PMW3901_SPI_CDIV
#define PMW3901_SPI_CDIV SPIDiv32
#endif

#define PMW3901_REG_MOTION     0x02
#define PMW3901_REG_DELTA_X_L  0x03
#define PMW3901_REG_DELTA_X_H  0x04
#define PMW3901_REG_DELTA_Y_L  0x05
#define PMW3901_REG_DELTA_Y_H  0x06


// Blocking read/write functions
static uint8_t readRegister_blocking(struct pmw3901_t *pmw, uint8_t addr) {
  spi_slave_select(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);

  sys_time_usleep(10);

  pmw->trans.output_buf[0] = addr & 0x7F;  // MSB 0 => read
  pmw->trans.output_length = 1;
  pmw->trans.input_length = 0;
  pmw->trans.select = SPINoSelect;
  spi_blocking_transceive(pmw->periph, &pmw->trans, 0.5);
  
  pmw->trans.output_buf[0] = 0;
  pmw->trans.output_length = 1;
  pmw->trans.input_length = 1;
  pmw->trans.select = SPINoSelect;
  spi_blocking_transceive(pmw->periph, &pmw->trans, 0.5);

  spi_slave_unselect(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  return pmw->trans.input_buf[0];

}

static void writeRegister_blocking(struct pmw3901_t *pmw, uint8_t addr, uint8_t data) {
  spi_slave_select(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);

  sys_time_usleep(35);

  pmw->trans.output_buf[0] = addr | 0x80;  // MSB 1 => write
  pmw->trans.output_length = 1;
  pmw->trans.input_length = 0;
  pmw->trans.select = SPINoSelect;
  spi_blocking_transceive(pmw->periph, &pmw->trans, 0.5);

  pmw->trans.output_buf[0] = data;
  pmw->trans.output_length = 1;
  pmw->trans.input_length = 0;
  pmw->trans.select = SPINoSelect;
  spi_blocking_transceive(pmw->periph, &pmw->trans, 0.5);
  sys_time_usleep(35);
  spi_slave_unselect(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  sys_time_usleep(200);
}

// For PixArt firmware compatibility:
#define writeRegister(_addr, _data)  writeRegister_blocking(pmw, (_addr), (_data))
#define readRegister(_addr)  readRegister_blocking(pmw, (_addr))
#define wait_ms(_ms)  sys_time_usleep((_ms) * 1000)


static void initializeSensor(struct pmw3901_t *pmw) {

  spi_slave_unselect(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  wait_ms(1);
  spi_slave_select(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  wait_ms(1);
  spi_slave_unselect(OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  wait_ms(1);
  
  // Power on reset
  writeRegister(0x3A, 0x5A);

  wait_ms(5);
  
  // Try to detect sensor before initializing
  int tries = 0;
  while (readRegister(0x00) != 0x49 && tries < 100) {
    sys_time_usleep(50);
    tries++;
  }
  readRegister(0x5F);

  readRegister(0x02);
  readRegister(0x03);
  readRegister(0x04);
  readRegister(0x05);
  readRegister(0x06);
  wait_ms(1);

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
  writeRegister(0x74, 0x1F);
  writeRegister(0x75, 0x1F);
  writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78);
  writeRegister(0x44, 0x08);
  writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF);
  writeRegister(0x65, 0x1F);
  writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x60);
  writeRegister(0x66, 0x08);
  writeRegister(0x63, 0x78);
  writeRegister(0x7F, 0x15);
  writeRegister(0x48, 0x58);
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

  wait_ms(100);
  writeRegister(0x32, 0x44);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06);
  writeRegister(0x62, 0xf0);
  writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D);
  writeRegister(0x48, 0xC0);
  writeRegister(0x6F, 0xd5);
  writeRegister(0x7F, 0x00);
  writeRegister(0x5B, 0xa0);
  writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50);
  writeRegister(0x40, 0x80);
}


static void pmw3901_thd(void* arg) {
  struct pmw3901_t* pmw = (struct pmw3901_t*) arg;

  while(true) {
    
    pprz_bsem_wait(&pmw->bsem);  // wait to be woken up by the AP thread

    uint8_t tp = readRegister_blocking(pmw, PMW3901_REG_MOTION);
    uint8_t xh = readRegister_blocking(pmw, PMW3901_REG_DELTA_X_H);
    uint8_t xl = readRegister_blocking(pmw, PMW3901_REG_DELTA_X_L);
    uint8_t yh = readRegister_blocking(pmw, PMW3901_REG_DELTA_Y_H);
    uint8_t yl = readRegister_blocking(pmw, PMW3901_REG_DELTA_Y_L);

    pmw->delta_x = (xh << 8) | xl;
    pmw->delta_y = (yh << 8) | yl;
    pmw->data_available = true;
  }

}


void pmw3901_init(struct pmw3901_t *pmw, struct spi_periph *periph, uint8_t slave_idx) {
  // Set up SPI peripheral and transaction
  pmw->periph = periph;
  pmw->trans.input_buf = pmw->spi_input_buf;
  pmw->trans.output_buf = pmw->spi_output_buf;
  pmw->trans.slave_idx = slave_idx;
  pmw->trans.select = SPINoSelect;
  pmw->trans.cpol = SPICpolIdleHigh;
  pmw->trans.cpha = SPICphaEdge2;
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

  pprz_bsem_init(&pmw->bsem, true);
  pprz_thread_create(&pmw->thd_handle, 1024, "pmw3901", PPRZ_NORMAL_PRIO+1, pmw3901_thd, pmw);
}

bool pmw3901_get_data(struct pmw3901_t *pmw, int16_t *delta_x, int16_t *delta_y) {
  if (!pmw->data_available) return false;
  *delta_x = pmw->delta_x;
  *delta_y = pmw->delta_y;
  pmw->data_available = false;
  return true;
}
