/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
 *
 */

/**
 * @file mcu_periph/softi2c.c
 * Platform-independent software I2C implementation.
 * Can be used transparently in place of the hardware I2C in i2c.h.
 */

#include "mcu_periph/softi2c.h"

#include "mcu_arch.h"
#include "mcu_periph/gpio.h"


static bool softi2c_idle(struct i2c_periph *periph) __attribute__((unused));
static bool softi2c_submit(struct i2c_periph *periph, struct i2c_transaction *t) __attribute__((unused));
static void softi2c_setbitrate(struct i2c_periph *periph, int bitrate) __attribute__((unused));


#if USE_SOFTI2C0 && ( \
  !defined(SOFTI2C0_SDA_GPIO) || !defined(SOFTI2C0_SDA_PIN) || \
  !defined(SOFTI2C0_SCL_GPIO) || !defined(SOFTI2C0_SCL_PIN))
#error "SDA and SCL pins must be configured for SOFTI2C0!"
#endif

#if USE_SOFTI2C1 && ( \
  !defined(SOFTI2C1_SDA_GPIO) || !defined(SOFTI2C1_SDA_PIN) || \
  !defined(SOFTI2C1_SCL_GPIO) || !defined(SOFTI2C1_SCL_PIN))
#error "SDA and SCL pins must be configured for SOFTI2C1!"
#endif


#if USE_SOFTI2C0
struct i2c_periph softi2c0;
void softi2c0_init(void) {
  i2c_init(&softi2c0);
  softi2c0_hw_init();
}
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
struct i2c_periph softi2c1;
void softi2c1_init(void) {
  i2c_init(&softi2c1);
  softi2c1_hw_init();
}
#endif /* USE_SOFTI2C1 */


static void softi2c_gpio_highz(uint32_t port, uint16_t pin) {
#if defined(CHIBIOS_MCU_ARCH_H) || defined(STM32_MCU_ARCH_H)
  /* Arch's with input_pullup */
  gpio_setup_input_pullup(port, pin);
#else
  /* Arch's without input_pullup */
  // Gpio_set might enable pullup on some architectures (e.g. arduino), not sure
  // if it works here. If not, an external pull-up resistor is required on the
  // I2C lines.
  gpio_setup_input(port, pin);
  gpio_set(port, pin);
#endif /* arch with/without input_pullup */
}

static bool softi2c_gpio_read(uint32_t port, uint16_t pin) {
  softi2c_gpio_highz(port, pin);
  return gpio_get(port, pin);
}

static void softi2c_gpio_drive_low(uint32_t port, uint16_t pin) {
  gpio_setup_output(port, pin);
  gpio_clear(port, pin);
}

static void softi2c_setup_gpio(
    uint32_t sda_port, uint16_t sda_pin,
    uint32_t scl_port, uint16_t scl_pin) {
#ifdef STM32_MCU_ARCH_H
  gpio_enable_clock(sda_port);
  gpio_enable_clock(scl_port);
#endif /* STM32F1/F4 */
  softi2c_gpio_highz(sda_port, sda_pin);
  softi2c_gpio_highz(scl_port, scl_pin);
}


#if USE_SOFTI2C0
#ifndef SOFTI2C0_CLOCK_SPEED
#define SOFTI2C0_CLOCK_SPEED 1000
#endif
PRINT_CONFIG_VAR(SOFTI2C0_CLOCK_SPEED);

void softi2c0_hw_init(void) {
  softi2c0.idle = softi2c_idle;
  softi2c0.submit = softi2c_submit;
  softi2c0.setbitrate = softi2c_setbitrate;

  /* Set up GPIO */
  softi2c_setup_gpio(
      SOFTI2C0_SDA_GPIO, SOFTI2C0_SDA_PIN,
      SOFTI2C0_SCL_GPIO, SOFTI2C0_SCL_PIN);

  /* Set default bitrate */
  i2c_setbitrate(&softi2c0, SOFTI2C0_CLOCK_SPEED);
}
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
#ifndef SOFTI2C1_CLOCK_SPEED
#define SOFTI2C1_CLOCK_SPEED 1000
#endif
PRINT_CONFIG_VAR(SOFTI2C1_CLOCK_SPEED);

void softi2c1_hw_init(void) {
  softi2c1.idle = softi2c_idle;
  softi2c1.submit = softi2c_submit;
  softi2c1.setbitrate = softi2c_setbitrate;

  /* Set up GPIO */
  softi2c_setup_gpio(
      SOFTI2C1_SDA_GPIO, SOFTI2C1_SDA_PIN,
      SOFTI2C1_SCL_GPIO, SOFTI2C1_SCL_PIN);

  /* Set default bitrate */
  i2c_setbitrate(&softi2c1, SOFTI2C1_CLOCK_SPEED);
}
#endif /* USE_SOFTI2C1 */
