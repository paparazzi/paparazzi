/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file arch/chibios/modules/sensors/hx711.c
 * Interface for the HX711 sensor
 *
 */

#include "hx711.h"
#include "mcu_periph/gpio.h"
#include "modules/datalink/downlink.h"
#include "modules/datalink/telemetry.h"
#include "filters/median_filter.h"
#include BOARD_CONFIG

#ifndef HX711_DEVICES_NB
#define HX711_DEVICES_NB 4
#endif

#ifndef HX711_GAIN
#define HX711_GAIN 1
#endif

#ifndef HX711_PWM_FREQUENCY
#define HX711_PWM_FREQUENCY 6000000
#endif

#ifndef HX711_DEVICES
#define HX711_DEVICES {}
#endif

#ifndef HX711_GROUND_THRESHOLD 
#define HX711_GROUND_THRESHOLD 100000 
#endif

#ifndef HX711_MEDIAN_FILT_SIZE 
#define HX711_MEDIAN_FILT_SIZE 3 
#endif

// Running at 50kHz for a full clock pulse (10us high, 10us low)
#define HX711_PERIOD (HX711_PWM_FREQUENCY / 50000)

struct MedianFilterFloat measurement_filt[HX711_DEVICES_NB];
float hx711_ground_threshold = HX711_GROUND_THRESHOLD;
int32_t hx711_offset = 0;
float hx711_meas_time = 0;
struct hx711_dev_t {
  ioportid_t data_port;
  uint16_t data_pin;
  int32_t measurement;
  int32_t offset;
};

struct hx711_t {
  volatile bool busy;
  volatile bool measurement_ready;
  struct hx711_dev_t devices[HX711_DEVICES_NB];

  volatile uint8_t read_bit_idx;
};


static void pwmpcb(PWMDriver *pwmp __attribute__((unused)));

static volatile struct hx711_t hx711 = {
  .busy = false,
  .measurement_ready = false,
  .devices = {
    HX711_DEVICES
  },
  .read_bit_idx = 0
};

static PWMConfig pwmcfg = {
  .frequency = HX711_PWM_FREQUENCY,
  .period = HX711_PERIOD,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};


void hx711_init(void)
{
  /*----------------
   * Configure GPIO
   *----------------*/
  gpio_setup_pin_af(HX711_PWM_PORT, HX711_PWM_PIN, HX711_PWM_AF, true);
  pwmcfg.channels[HX711_PWM_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
  pwmcfg.channels[HX711_PWM_CHANNEL].callback = pwmpcb;

  for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
    gpio_setup_input(hx711.devices[i].data_port, hx711.devices[i].data_pin);
  }

  /*---------------
   * Configure PWM
   *---------------*/
  pwmStart(&HX711_PWM_DRIVER, &pwmcfg);
  hx711.busy = false;
  hx711.measurement_ready = false;
  hx711.read_bit_idx = 0;
  for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
    init_median_filter_f(&measurement_filt[i], HX711_MEDIAN_FILT_SIZE);
  }
}

/**
 * Start a measurement and publish the results when possible
 */
void hx711_event(void)
{
  // If busy reading data, return
  if(hx711.busy)
    return;

  // Check if we have a measurement to read
  if(hx711.measurement_ready) {
    // Process the measurement ABI??

    // Send down for debug
    float filt_val[HX711_DEVICES_NB];
    for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
      update_median_filter_f(&measurement_filt[i], hx711.devices[i].measurement - hx711.devices[i].offset);
      filt_val[i] = get_median_filter_f(&measurement_filt[i]);
    }

    float current_time = get_sys_time_float();
    float freq = 1 / (current_time - hx711_meas_time);
    hx711_meas_time = current_time;

    RunOnceEvery(10, {
      DOWNLINK_SEND_HX711(DefaultChannel, DefaultDevice, &freq, HX711_DEVICES_NB, filt_val);
    });
    pprz_msg_send_HX711(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, &freq, HX711_DEVICES_NB, filt_val);

    hx711.measurement_ready = false;
  }
  
  // Check if all data pins are low, ready to read data
  for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
    if(gpio_get(hx711.devices[i].data_port, hx711.devices[i].data_pin) != 0)
      return;
  }

  // Start reading data
  chSysLock();
  hx711.busy = true;
  hx711.measurement_ready = false;
  hx711.read_bit_idx = 0;
  pwmEnableChannel(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL, HX711_PERIOD/2); // 50% duty cycle
  pwmEnableChannelNotification(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL);
  chSysUnlock();
}

/* Ground detected if strain gauges read above a certain threshold */
bool hx711_ground_detect(void) {
  for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
    if (fabsf(get_median_filter_f(&measurement_filt[i])) > hx711_ground_threshold) {
      return true;
    } 
  }
  return false;
}

/**
 * Callback on the falling edge of the clock signal
 */
static void pwmpcb(PWMDriver *pwmp __attribute__((unused))) {
  chSysLockFromISR();

  /* In case we where not reading just stop */
  if(!hx711.busy) {
    pwmDisableChannelNotificationI(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL);
    pwmDisableChannelI(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL);
    chSysUnlockFromISR();
    return;
  }
  
  /* Parse the measurements for all devices */
  if(hx711.read_bit_idx < 24) {
    for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
      volatile struct hx711_dev_t *dev = &hx711.devices[i];
      
      // Reset or bitshift
      if(hx711.read_bit_idx == 0)
        dev->measurement = 0;
      else
        dev->measurement <<= 1;
      
      // Read the bit
      if(gpio_get(dev->data_port, dev->data_pin) != 0)
        dev->measurement |= 1;

      // Fix 2s complement for 32 bit signed integer from 24 bit
      if(dev->measurement & (1 << 23)) {
        dev->measurement |= 0xFF000000;
      }
    }
  }
  hx711.read_bit_idx++;

  /* Send gain and finish reading data */
  if(hx711.read_bit_idx >= (24 + HX711_GAIN)) {
    pwmDisableChannelNotificationI(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL);
    pwmDisableChannelI(&HX711_PWM_DRIVER, HX711_PWM_CHANNEL);
    hx711.busy = false;
    hx711.measurement_ready = true;
    chSysUnlockFromISR();
    return;
  }

  chSysUnlockFromISR();
}

void hx711_autoset_offset(int32_t offset) {
  (void) offset;
  for(uint8_t i = 0; i < HX711_DEVICES_NB; i++) {
    hx711.devices[i].offset += get_median_filter_f(&measurement_filt[i]);
  }
}