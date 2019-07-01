/*
 * Copyright (C) 2019 Xavier Paris <xavier.paris@enac.fr>
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
 * @file    light_ws2812i_arch.c
 * @brief   WS2812 driver based on ChibiOS
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

/****************************************************************************************
https://github.com/joewa/WS2812-LED-Driver_ChibiOS/
****************************************************************************************/

#include <hal.h>
#include "modules/light/light_ws2812_arch.h"
#include "mcu_periph/hal_stm32_dma.h"

#define WS2812_SERVO_HZ       800000
#define WS2812_PWM_FREQUENCY  (STM32_SYSCLK/2)

#define WS2812_DUTYCYCLE_0    (WS2812_PWM_FREQUENCY/(1000000000/350))
#define WS2812_DUTYCYCLE_1    (WS2812_PWM_FREQUENCY/(1000000000/800))

#define WS2812_RESET_BIT_N    (50)
#define WS2812_COLOR_BIT_N    (WS2812_NB_LEDS * 24)
#define WS2812_BIT_N          (WS2812_COLOR_BIT_N + WS2812_RESET_BIT_N)

// Use driver 1 by default
#ifndef WS2812_CFG_DEF
#define WS2812_CFG_DEF WS2812D1_CFG_DEF
#endif

#define WS2812_DMA_IRQ_PRIORITY 6

/**
 * Configuration structure
 */
typedef struct  {
  uint32_t dma_stream;
  uint8_t dma_channel;
  uint8_t pwm_channel;
  uint8_t dma_priority;
  PWMDriver *pwmp;
} WS2812Config;

// Config from board conf
static WS2812Config WS2812CFG = WS2812_CFG_DEF;

/**
 * Driver structure
 */
typedef struct  {
  const WS2812Config *config;
  DMAConfig dma_conf;
  PWMConfig pwm_conf;
  DMADriver dmap;
  uint32_t buf[WS2812_BIT_N + 1];
} WS2812Driver;

// Driver
static WS2812Driver WS2812D;

void light_ws2812_arch_init(void)
{
  uint32_t i;
  for (i = 0; i < WS2812_COLOR_BIT_N; i++) {
    WS2812D.buf[i] = WS2812_DUTYCYCLE_0;
  }
  for (i = 0; i < WS2812_RESET_BIT_N; i++) {
    WS2812D.buf[i + WS2812_COLOR_BIT_N] = 0;
  }

  WS2812D.config = &WS2812CFG;

  WS2812D.dma_conf = (DMAConfig) {
    .stream = WS2812CFG.dma_stream,
    .channel = WS2812CFG.dma_channel,
    .dma_priority = WS2812CFG.dma_priority,
    .irq_priority = WS2812_DMA_IRQ_PRIORITY,
    .direction = DMA_DIR_M2P,
    .psize = 4,
    .msize = 4,
    .inc_peripheral_addr = false,
    .inc_memory_addr = true,
    .circular = true,
    .error_cb = NULL,
    .end_cb = NULL,
    .pburst = 0,
    .mburst = 0,
    .fifo = 0
  };

  WS2812D.pwm_conf = (PWMConfig) {
    .frequency = WS2812_PWM_FREQUENCY,
    .period    = WS2812_PWM_FREQUENCY / WS2812_SERVO_HZ,
    .callback  = NULL,
    .channels  = {
      {
        .mode = PWM_OUTPUT_DISABLED,
        .callback = NULL
      },
      {
        .mode = PWM_OUTPUT_DISABLED,
        .callback = NULL
      },
      {
        .mode = PWM_OUTPUT_DISABLED,
        .callback = NULL
      },
      {
        .mode = PWM_OUTPUT_DISABLED,
        .callback = NULL
      },
    },
    .cr2  =  0,
    .dier =  STM32_TIM_DIER_UDE
  };

  dmaObjectInit(&WS2812D.dmap);
  dmaStart(&WS2812D.dmap, &WS2812D.dma_conf);
  dmaStartTransfert(&WS2812D.dmap, &(WS2812D.config->pwmp->tim->CCR[WS2812D.config->pwm_channel]), &WS2812D.buf, WS2812_BIT_N);

  WS2812D.pwm_conf.channels[WS2812D.config->pwm_channel].mode = PWM_OUTPUT_ACTIVE_HIGH;
  pwmStart(WS2812D.config->pwmp, &WS2812D.pwm_conf);
  pwmEnableChannel(WS2812D.config->pwmp, WS2812D.config->pwm_channel, 0);
}

#define WS2812_BIT(led, byte, bit) (24*(led) + 8*(byte) + (7 - (bit)))
#define WS2812_RED_BIT(led, bit)   WS2812_BIT((led), 1, (bit))
#define WS2812_GREEN_BIT(led, bit) WS2812_BIT((led), 0, (bit))
#define WS2812_BLUE_BIT(led, bit)  WS2812_BIT((led), 2, (bit))

void light_ws2812_arch_set(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t bit;
  if (led_number < WS2812_NB_LEDS) {
    for (bit = 0; bit < 8; bit++) {
      WS2812D.buf[WS2812_RED_BIT(led_number, bit)]    = ((r >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
      WS2812D.buf[WS2812_GREEN_BIT(led_number, bit)]  = ((g >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
      WS2812D.buf[WS2812_BLUE_BIT(led_number, bit)]   = ((b >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
    }
  }
}
