/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
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
 * @file arch/chibios/mcu_periph/adc_arch.c
 * ADC driver
 *
 * ADC defines for Lia board (STM32F105)
 * 5 Channels:
 * 1 - ADC1, Channel 13
 * 2 - ADC2, Channel 10
 * 3 - ADC3, Channel 11
 * 4 - ADC4, Channel 14
 * 5 - Temperature sensor, Channel 16
 *
 * Default buffer depth is equal to MAX_AV_NB_SAMPLE
 * This allows us to always have more or equal samples than the user requires
 *
 * @note: With ChibiOS/RT 2.6.2 and STM32F105 chip the channels have to be interleaved,
 * otherwise the current measurement is affected by the previous channel measured.
 * Most likely bug on ChibiOS ADC driver for this particular chip.
 *
 * For 5 actual channels we have to define (DUMMY stants for 1 dummy channel):
 * CH1 + DUMMY + CH2 + DUMMY + CH3 + DUMMY + CH4 + DUMMY + CH_TEMP = 9 channels
 * Then only every second channel is really read. Might be fixed in next version
 * of ChibiOS
 *
 * V_ref is 3.3V, ADC has 12bit resolution.
 */
#include "mcu_periph/adc.h"
#include "hal.h"

// From active ADC channels
#define ADC_NUM_CHANNELS NB_ADC

// Macros to automatically enable the correct ADC

#define USE_AD1 1
//#if NB_ADC1_CHANNELS != 0
#if NB_ADC != 0
#ifndef USE_AD1
#define USE_AD1 1
#endif
#endif

//#if defined(AD2_1_CHANNEL) || defined(AD2_2_CHANNEL) || defined(AD2_3_CHANNEL) || defined(AD2_4_CHANNEL)
//#ifndef USE_AD2
//#define USE_AD2 1
//#endif
//#endif

// Create channel map
static const uint8_t adc_channel_map[ADC_NUM_CHANNELS] = {
#ifdef AD1_1_CHANNEL
  AD1_1_CHANNEL,
#endif
#ifdef AD1_2_CHANNEL
  AD1_2_CHANNEL,
#endif
#ifdef AD1_3_CHANNEL
  AD1_3_CHANNEL,
#endif
#ifdef AD1_4_CHANNEL
  AD1_4_CHANNEL,
#endif
#if USE_AD1_5
  AD1_5_CHANNEL,
#endif
#if USE_AD1_6
  AD1_6_CHANNEL,
#endif
#if USE_AD1_7
  AD1_7_CHANNEL,
#endif
#if USE_AD1_8
  AD1_8_CHANNEL,
#endif
#if USE_AD1_9
  AD1_9_CHANNEL,
#endif
#if USE_AD1_10
  AD1_10_CHANNEL,
#endif
#if USE_AD1_11
  AD1_11_CHANNEL,
#endif
#if USE_AD1_12
  AD1_12_CHANNEL,
#endif
#if USE_AD1_13
  AD1_13_CHANNEL,
#endif
#if USE_AD1_14
  AD1_14_CHANNEL,
#endif
#if USE_AD1_15
  AD1_15_CHANNEL,
#endif
#if USE_AD1_16
  AD1_16_CHANNEL,
#endif
};


uint8_t adc_error_flag = 0;
ADCDriver *adcp_err = NULL;

#ifndef ADC_BUF_DEPTH
#define ADC_BUF_DEPTH (MAX_AV_NB_SAMPLE/2)
#endif

#define ADC_V_REF_MV 3300
#define ADC_12_BIT_RESOLUTION 4096

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

#if USE_AD1
static struct adc_buf *adc1_buffers[ADC_NUM_CHANNELS];
static uint32_t adc1_sum_tmp[ADC_NUM_CHANNELS];
static uint8_t adc1_samples_tmp[ADC_NUM_CHANNELS];
#endif
#if USE_AD2
#error ADC2_not implemented in ChibiOS(STM32F105/7 board)
#endif

// From libopencm3
static void adc_regular_sequence(uint32_t *sqr1, uint32_t *sqr2, uint32_t *sqr3, uint8_t length, const uint8_t channel[])
{
  uint32_t first6 = 0;
  uint32_t second6 = 0;
  uint32_t third6 = ADC_SQR1_NUM_CH(length);
  uint8_t i = 0;

  for (i = 1; i <= length; i++) {
    if (i <= 6) {
      first6 |= (channel[i - 1] << ((i - 1) * 5));
    }
    if ((i > 6) & (i <= 12)) {
      second6 |= (channel[i - 1] << ((i - 6 - 1) * 5));
    }
    if ((i > 12) & (i <= 18)) {
      third6 |= (channel[i - 1] << ((i - 12 - 1) * 5));
    }
  }
  *sqr3 = first6;
  *sqr2 = second6;
  *sqr1 = third6;
}

// From libopencm3
static void adc_sample_time_on_all_channels(uint32_t *smpr1, uint32_t *smpr2, uint8_t time)
{
  uint8_t i;
  uint32_t reg32 = 0;

  for (i = 0; i <= 9; i++) {
    reg32 |= (time << (i * 3));
  }
  *smpr2 = reg32;

  reg32 = 0;
  for (i = 10; i <= 17; i++) {
    reg32 |= (time << ((i - 10) * 3));
  }
  *smpr1 = reg32;
}

/**
 * Adc1 callback
 *
 * Callback, fired after half of the buffer is filled (i.e. half of the samples
 * is collected). Since we are assuming continuous ADC conversion, the ADC state is
 * never equal to ADC_COMPLETE.
 *
 * @note    Averaging is done when the subsystems ask for ADC values
 * @param[in] adcp pointer to a @p ADCDriver object
 * @param[in] buffer pointer to a @p buffer with samples
 * @param[in] n number of samples
 */
#include "led.h"
void adc1callback(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  if (adcp->state != ADC_STOP) {
#if USE_AD1
    for (int channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
      if (adc1_buffers[channel] != NULL) {
        adc1_sum_tmp[channel] = 0;
        if (n > 0) {
          adc1_samples_tmp[channel] = n;
        }
        else {
          LED_TOGGLE(4);
          adc1_samples_tmp[channel] = 1;
        }
        for (unsigned int sample = 0; sample < n; sample++) {
          adc1_sum_tmp[channel] += buffer[channel + sample * ADC_NUM_CHANNELS];
        }
        adc1_sum_tmp[channel] = (adc1_sum_tmp[channel]) * ADC_V_REF_MV / ADC_12_BIT_RESOLUTION;
      }
    }
    chSysLockFromIsr();
    for (int channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
      if (adc1_buffers[channel] != NULL) {
        adc1_buffers[channel]->sum = adc1_sum_tmp[channel];
        adc1_buffers[channel]->av_nb_sample = adc1_samples_tmp[channel];
      }
    }
    chSysUnlockFromIsr();
#endif
  }
}


/**
 * Adc error callback
 *
 * Fired if DMA error happens or ADC overflows
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err)
{
  chSysLockFromIsr();
  adcp_err = adcp;
  adc_error_flag = err;
  chSysUnlockFromIsr();
}

/**
 * Link between ChibiOS ADC drivers and Paparazzi adc_buffers
 */
void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample)
{
  adc1_buffers[adc_channel] = s;
  if (av_nb_sample <= MAX_AV_NB_SAMPLE) {
    s->av_nb_sample = av_nb_sample;
  } else {
    s->av_nb_sample = MAX_AV_NB_SAMPLE;
  }
}

static  ADCConversionGroup adcgrpcfg;

/**
 * Adc init
 *
 * Initialize ADC drivers, buffers and start conversion in the background
 */
void adc_init(void)
{
  uint32_t sqr1, sqr2, sqr3;
  adc_regular_sequence(&sqr1, &sqr2, &sqr3, ADC_NUM_CHANNELS, adc_channel_map);

  /**
   * Conversion configuration for ADC
   *
   * Continuous conversion, includes CPU temp sensor
   * ADCCLK = 14 MHz
   * Temp sensor: Ts = 7 + 12.5 = 19.5us
   * Other channels: Ts = 41.5c + 12.5c = 54.0us
   *
   * @note: This should be probably moved into boar.h
   */
#ifdef __STM32F10x_H
  uint32_t smpr1, smpr2;
  adc_sample_time_on_all_channels(&smpr1, &smpr2, ADC_SAMPLE_41P5);

  adcgrpcfg = {
    TRUE, ADC_NUM_CHANNELS,
    adc1callback, adcerrorcallback,
    0, ADC_CR2_TSVREFE,
    smpr1, smpr2,
    sqr1, sqr2, sqr3
  };
#elif defined(__STM32F4xx_H)
  uint32_t smpr1, smpr2;
  adc_sample_time_on_all_channels(&smpr1, &smpr2, ADC_SAMPLE_56);

  adcgrpcfg.circular = TRUE;
  adcgrpcfg.num_channels = ADC_NUM_CHANNELS;
  adcgrpcfg.end_cb = adc1callback;
  adcgrpcfg.error_cb = adcerrorcallback;
  adcgrpcfg.cr1 = 0;
  adcgrpcfg.cr2 = 0;
  adcgrpcfg.smpr1 = smpr1;
  adcgrpcfg.smpr2 = smpr2;
  adcgrpcfg.sqr1 = sqr1;
  adcgrpcfg.sqr2 = sqr2;
  adcgrpcfg.sqr3 = sqr3;
#endif

  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);
}
