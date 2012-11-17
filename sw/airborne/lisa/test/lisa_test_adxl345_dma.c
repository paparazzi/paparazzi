/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "libopencm3/stm32/f1/gpio.h"
#include "libopencm3/stm32/f1/nvic.h"
#include "libopencm3/stm32/exti.h"

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/spi.h"

#include "peripherals/adxl345.h"
#include "led.h"

#ifndef ADXL345_SLAVE_IDX
#define ADXL345_SLAVE_IDX SPI_SLAVE2
#endif

#ifndef ADXL345_SPI_DEV
#define ADXL345_SPI_DEV spi2
#endif

#define CONFIGURED 6
static uint8_t acc_status=0;
static volatile uint8_t acc_data_available = FALSE;
static volatile uint8_t foo = FALSE;
struct spi_transaction adxl345_spi_trans;

static uint8_t dma_tx_buf[7];
static uint8_t dma_rx_buf[7];

static void write_to_reg(uint8_t addr, uint8_t val);
static void adxl345_trans_cb(struct spi_transaction *trans);
static inline void init_adxl345_spi_trans( void );
static void read_data(void);

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

void hw_init(void);

int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  hw_init();
  init_adxl345_spi_trans();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(10,
               {
                 DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
                 LED_PERIODIC();
               });

  if (acc_status != CONFIGURED) {
    /* set data rate to 800Hz */
    write_to_reg(ADXL345_REG_BW_RATE, 0x0D);
    /* switch to measurememnt mode */
    write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
    /* enable data ready interrupt */
    write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7);
    /* Enable full res and interrupt active low */
    write_to_reg(ADXL345_REG_DATA_FORMAT, 1<<3|1<<5);
    /* reads data once to bring interrupt line up */
    read_data();
    acc_status = CONFIGURED;
  }
  else {
    read_data();
  }
}


static inline void main_event_task( void ) {
  if (foo) {
    foo = FALSE;
    RunOnceEvery(100, {LED_TOGGLE(3);});
  }
  if (acc_status >= CONFIGURED && acc_data_available) {
    acc_data_available = FALSE;
    int16_t ax = dma_rx_buf[1] | (dma_rx_buf[2]<<8);
    int16_t ay = dma_rx_buf[3] | (dma_rx_buf[4]<<8);
    int16_t az = dma_rx_buf[5] | (dma_rx_buf[6]<<8);
    int32_t iax = ax;
    int32_t iay = ay;
    int32_t iaz = az;
    RunOnceEvery(10, {DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &iax, &iay, &iaz);});
  }
}

static void write_to_reg(uint8_t addr, uint8_t val) {
  dma_tx_buf[0] = addr;
  dma_tx_buf[1] = val;
  spi_submit(&(ADXL345_SPI_DEV), &adxl345_spi_trans);
  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while(adxl345_spi_trans.status != SPITransSuccess);
}

static void read_data(void) {
  dma_tx_buf[0] = (1<<7|1<<6|ADXL345_REG_DATA_X0);
  spi_submit(&(ADXL345_SPI_DEV), &adxl345_spi_trans);
}


static void adxl345_trans_cb( struct spi_transaction *trans ) {
  acc_data_available = TRUE;
}

static inline void init_adxl345_spi_trans( void ) {
  adxl345_spi_trans.select = SPISelectUnselect;
  adxl345_spi_trans.cpol = SPICpolIdleHigh;
  adxl345_spi_trans.cpha = SPICphaEdge2;
  adxl345_spi_trans.dss = SPIDss8bit;
  adxl345_spi_trans.bitorder = SPIMSBFirst;
  adxl345_spi_trans.cdiv = SPIDiv64;
  adxl345_spi_trans.slave_idx = ADXL345_SLAVE_IDX;
  adxl345_spi_trans.output_length = 7;
  adxl345_spi_trans.input_length = 7;
  adxl345_spi_trans.after_cb = adxl345_trans_cb;
  adxl345_spi_trans.input_buf = &dma_rx_buf[0];
  adxl345_spi_trans.output_buf = &dma_tx_buf[0];
}

void hw_init(void) {
  /* configure external interrupt exti2 on PB2( accel int ) */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);

  exti_select_source(EXTI2, GPIOB);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI2);

  nvic_set_priority(NVIC_EXTI2_IRQ, 0xF);
  nvic_enable_irq(NVIC_EXTI2_IRQ);
}


void exti2_isr(void) {
  /* clear EXTI */
  exti_reset_request(EXTI2);

  LED_TOGGLE(2);

  foo = TRUE;
}
