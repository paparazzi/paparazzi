/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file arch/chibios/mcu_periph/gpio_arch.c
 * gpio functions implemented for ChibiOS arch
 */
#include "mcu_periph/gpio.h"
#include "hal.h"

void gpio_setup_output(ioportid_t port, uint16_t gpios)
{
  chSysLock();
  palSetPadMode(port, gpios, PAL_MODE_OUTPUT_PUSHPULL);
  chSysUnlock();
}

void gpio_setup_input(ioportid_t port, uint16_t gpios)
{
  chSysLock();
  palSetPadMode(port, gpios, PAL_MODE_INPUT);
  chSysUnlock();
}

void gpio_setup_input_pullup(ioportid_t port, uint16_t gpios)
{
  chSysLock();
  palSetPadMode(port, gpios, PAL_MODE_INPUT_PULLUP);
  chSysUnlock();
}

void gpio_setup_input_pulldown(ioportid_t port, uint16_t gpios)
{
  chSysLock();
  palSetPadMode(port, gpios, PAL_MODE_INPUT_PULLDOWN);
  chSysUnlock();
}

void gpio_setup_pin_af(ioportid_t port, uint16_t pin, uint8_t af, bool is_output)
{
  chSysLock();
// architecture dependent settings
#if defined(STM32F1XX)
// FIXME: STM32F1xx doesn't support several alternate modes, is it needed for drivers?
  (void)port;
  (void)pin;
  (void)af;
  (void)is_output;
#else
// STM32F4xx, STM32F3xx, STM32F7xx and STM32H7xx
  if (af) {
    palSetPadMode(port, pin, PAL_MODE_ALTERNATE(af));
  } else {
    if (is_output) {
      palSetPadMode(port, pin, PAL_MODE_OUTPUT_PUSHPULL);
    } else {
      palSetPadMode(port, pin, PAL_MODE_INPUT);
    }
  }
#endif
  chSysUnlock();
}

void gpio_setup_pin_analog(ioportid_t port, uint16_t pin)
{
  chSysLock();
  palSetPadMode(port, pin, PAL_MODE_INPUT_ANALOG);
  chSysUnlock();
}

