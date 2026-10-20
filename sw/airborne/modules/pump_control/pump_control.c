/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/pump_control/pump_control.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Pump control for IMAV2026
 *
 * Default values for Tawaki v2, AUX 1 connector
 */

#include "modules/pump_control/pump_control.h"
#include "navigation.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/gpio.h"
#include "modules/datalink/downlink.h"

#ifndef PUMP_CONTROL_ADC
#error "PUMP_CONTROL_ADC must be defined!"
#define PUMP_CONTROL_ADC ADC_4
#endif

#ifndef PUMP_CONTROL_ADC_EMPTY
#define PUMP_CONTROL_ADC_EMPTY 20000
#endif

#ifndef PUMP_CONTROL_GPIO_ACTIVATE
#define PUMP_CONTROL_GPIO_ACTIVATE GPIOA,GPIO0
#endif

#ifndef PUMP_CONTROL_GPIO_PURGE
#define PUMP_CONTROL_GPIO_PURGE GPIOA,GPIO1
#endif


#define PUMP_CTRL_OFF 0
#define PUMP_CTRL_PUMP 1
#define PUMP_CTRL_VALVE 2

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

float pump_ctrl_state;

struct PumpControl {
  struct adc_buf adc_b;
  float time;
  float time_pumping;
};

static struct PumpControl pump_control;

void pump_control_init(void)
{
  pump_control.time = 0.f;
  pump_control.time_pumping = 0.f;
  adc_buf_channel(PUMP_CONTROL_ADC, &pump_control.adc_b, DEFAULT_AV_NB_SAMPLE);
  gpio_setup_output(PUMP_CONTROL_GPIO_ACTIVATE);
  gpio_setup_output(PUMP_CONTROL_GPIO_PURGE);
  gpio_clear(PUMP_CONTROL_GPIO_ACTIVATE);
  gpio_clear(PUMP_CONTROL_GPIO_PURGE);
}

void pump_activate(void)
{
  pump_control.time = 0.f;
  pump_control.time_pumping = 0.f;
  gpio_set(PUMP_CONTROL_GPIO_ACTIVATE);
  gpio_clear(PUMP_CONTROL_GPIO_PURGE);
  pump_ctrl_state = PUMP_CTRL_PUMP;
}

void pump_purge(void)
{
  pump_control.time = 0.f;
  pump_control.time_pumping = 0.f;
  gpio_set(PUMP_CONTROL_GPIO_PURGE);
  gpio_clear(PUMP_CONTROL_GPIO_ACTIVATE);
  pump_ctrl_state = PUMP_CTRL_VALVE;
}

void pump_stop(void)
{
  gpio_clear(PUMP_CONTROL_GPIO_ACTIVATE);
  gpio_clear(PUMP_CONTROL_GPIO_PURGE);
  pump_ctrl_state = PUMP_CTRL_OFF;
}

bool pump_pumping(void) {
  return adc_buf_get(&pump_control.adc_b) > PUMP_CONTROL_ADC_EMPTY;
}

bool pump_done(float duration, float timeout)
{
  pump_control.time += nav_dt;
  if (pump_pumping()) {
    pump_control.time_pumping += nav_dt;
  }
  if ((timeout > 0.f && pump_control.time > timeout)
      || pump_control.time_pumping > duration) {
    return true;
  }
  return false;
}

void pump_control_handler(float value) {
  int val = value;
  switch (val)
  {
  case PUMP_CTRL_OFF:
    pump_stop();
    break;
  case PUMP_CTRL_PUMP:
    pump_activate();
  break;
  case PUMP_CTRL_VALVE:
    pump_purge();
  break;
  
  default:
    break;
  }
}

void pump_control_report(void)
{
  uint16_t val1 = adc_buf_get(&pump_control.adc_b);
  uint16_t val2 = adc_buf_get(&pump_control.adc_b) > PUMP_CONTROL_ADC_EMPTY;

  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice,
      &val1,
      &val2);
}