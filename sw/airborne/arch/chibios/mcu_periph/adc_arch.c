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
 * V_ref is 3.3V, ADC has 12bit or 16bit resolution.
 */
#include "mcu_periph/adc.h"
#include "mcu_periph/gpio.h"
#include "hal.h"
#include "std.h"
#include "mcu_periph/ram_arch.h"


// Enable a converter when the board configuration assigns at least one pin to it.
#ifndef USE_AD1
#if (defined(ADC_1) && ADC1_ADC == 1) || (defined(ADC_2) && ADC2_ADC == 1) || \
    (defined(ADC_3) && ADC3_ADC == 1) || (defined(ADC_4) && ADC4_ADC == 1) || \
    (defined(ADC_5) && ADC5_ADC == 1) || (defined(ADC_6) && ADC6_ADC == 1) || \
    (defined(ADC_7) && ADC7_ADC == 1) || (defined(ADC_8) && ADC8_ADC == 1) || \
    (defined(ADC_9) && ADC9_ADC == 1)
#define USE_AD1 1
#else
#define USE_AD1 0
#endif
#endif
#ifndef USE_AD2
#if (defined(ADC_1) && ADC1_ADC == 2) || (defined(ADC_2) && ADC2_ADC == 2) || \
    (defined(ADC_3) && ADC3_ADC == 2) || (defined(ADC_4) && ADC4_ADC == 2) || \
    (defined(ADC_5) && ADC5_ADC == 2) || (defined(ADC_6) && ADC6_ADC == 2) || \
    (defined(ADC_7) && ADC7_ADC == 2) || (defined(ADC_8) && ADC8_ADC == 2) || \
    (defined(ADC_9) && ADC9_ADC == 2)
#define USE_AD2 1
#else
#define USE_AD2 0
#endif
#endif
#ifndef USE_AD3
#if (defined(ADC_1) && ADC1_ADC == 3) || (defined(ADC_2) && ADC2_ADC == 3) || \
    (defined(ADC_3) && ADC3_ADC == 3) || (defined(ADC_4) && ADC4_ADC == 3) || \
    (defined(ADC_5) && ADC5_ADC == 3) || (defined(ADC_6) && ADC6_ADC == 3) || \
    (defined(ADC_7) && ADC7_ADC == 3) || (defined(ADC_8) && ADC8_ADC == 3) || \
    (defined(ADC_9) && ADC9_ADC == 3)
#define USE_AD3 1
#else
#define USE_AD3 0
#endif
#endif

/* Set the default sample rate */
#if !defined(ADC_SAMPLE_RATE)
#if defined(STM32H7XX)
#define ADC_SAMPLE_RATE ADC_SMPR_SMP_384P5
#elif defined(STM32F3XX)
#define ADC_SAMPLE_RATE ADC_SMPR_SMP_601P5
#else
#define ADC_SAMPLE_RATE ADC_SAMPLE_480
#endif
#endif

// Build converter-specific channel maps from the generated board pin data.
#if USE_AD1
static const uint8_t adc1_channel_map[] = {
#if defined(ADC_1) && ADC1_ADC == 1
  ADC_1_CHANNEL,
#endif
#if defined(ADC_2) && ADC2_ADC == 1
  ADC_2_CHANNEL,
#endif
#if defined(ADC_3) && ADC3_ADC == 1
  ADC_3_CHANNEL,
#endif
#if defined(ADC_4) && ADC4_ADC == 1
  ADC_4_CHANNEL,
#endif
#if defined(ADC_5) && ADC5_ADC == 1
  ADC_5_CHANNEL,
#endif
#if defined(ADC_6) && ADC6_ADC == 1
  ADC_6_CHANNEL,
#endif
#if defined(ADC_7) && ADC7_ADC == 1
  ADC_7_CHANNEL,
#endif
#if defined(ADC_8) && ADC8_ADC == 1
  ADC_8_CHANNEL,
#endif
#if defined(ADC_9) && ADC9_ADC == 1
  ADC_9_CHANNEL,
#endif
};
static const uint8_t adc1_channel_ids[] = {
#if defined(ADC_1) && ADC1_ADC == 1
  ADC_1,
#endif
#if defined(ADC_2) && ADC2_ADC == 1
  ADC_2,
#endif
#if defined(ADC_3) && ADC3_ADC == 1
  ADC_3,
#endif
#if defined(ADC_4) && ADC4_ADC == 1
  ADC_4,
#endif
#if defined(ADC_5) && ADC5_ADC == 1
  ADC_5,
#endif
#if defined(ADC_6) && ADC6_ADC == 1
  ADC_6,
#endif
#if defined(ADC_7) && ADC7_ADC == 1
  ADC_7,
#endif
#if defined(ADC_8) && ADC8_ADC == 1
  ADC_8,
#endif
#if defined(ADC_9) && ADC9_ADC == 1
  ADC_9,
#endif
};
#define ADC1_CONFIGURED_CHANNELS (sizeof(adc1_channel_map) / sizeof(adc1_channel_map[0]))
#endif

#if USE_AD2
static const uint8_t adc2_channel_map[] = {
#if defined(ADC_1) && ADC1_ADC == 2
  ADC_1_CHANNEL,
#endif
#if defined(ADC_2) && ADC2_ADC == 2
  ADC_2_CHANNEL,
#endif
#if defined(ADC_3) && ADC3_ADC == 2
  ADC_3_CHANNEL,
#endif
#if defined(ADC_4) && ADC4_ADC == 2
  ADC_4_CHANNEL,
#endif
#if defined(ADC_5) && ADC5_ADC == 2
  ADC_5_CHANNEL,
#endif
#if defined(ADC_6) && ADC6_ADC == 2
  ADC_6_CHANNEL,
#endif
#if defined(ADC_7) && ADC7_ADC == 2
  ADC_7_CHANNEL,
#endif
#if defined(ADC_8) && ADC8_ADC == 2
  ADC_8_CHANNEL,
#endif
#if defined(ADC_9) && ADC9_ADC == 2
  ADC_9_CHANNEL,
#endif
};
static const uint8_t adc2_channel_ids[] = {
#if defined(ADC_1) && ADC1_ADC == 2
  ADC_1,
#endif
#if defined(ADC_2) && ADC2_ADC == 2
  ADC_2,
#endif
#if defined(ADC_3) && ADC3_ADC == 2
  ADC_3,
#endif
#if defined(ADC_4) && ADC4_ADC == 2
  ADC_4,
#endif
#if defined(ADC_5) && ADC5_ADC == 2
  ADC_5,
#endif
#if defined(ADC_6) && ADC6_ADC == 2
  ADC_6,
#endif
#if defined(ADC_7) && ADC7_ADC == 2
  ADC_7,
#endif
#if defined(ADC_8) && ADC8_ADC == 2
  ADC_8,
#endif
#if defined(ADC_9) && ADC9_ADC == 2
  ADC_9,
#endif
};
#define ADC2_CONFIGURED_CHANNELS (sizeof(adc2_channel_map) / sizeof(adc2_channel_map[0]))
#endif

#if USE_AD3
static const uint8_t adc3_channel_map[] = {
#if defined(ADC_1) && ADC1_ADC == 3
  ADC_1_CHANNEL,
#endif
#if defined(ADC_2) && ADC2_ADC == 3
  ADC_2_CHANNEL,
#endif
#if defined(ADC_3) && ADC3_ADC == 3
  ADC_3_CHANNEL,
#endif
#if defined(ADC_4) && ADC4_ADC == 3
  ADC_4_CHANNEL,
#endif
#if defined(ADC_5) && ADC5_ADC == 3
  ADC_5_CHANNEL,
#endif
#if defined(ADC_6) && ADC6_ADC == 3
  ADC_6_CHANNEL,
#endif
#if defined(ADC_7) && ADC7_ADC == 3
  ADC_7_CHANNEL,
#endif
#if defined(ADC_8) && ADC8_ADC == 3
  ADC_8_CHANNEL,
#endif
#if defined(ADC_9) && ADC9_ADC == 3
  ADC_9_CHANNEL,
#endif
};
static const uint8_t adc3_channel_ids[] = {
#if defined(ADC_1) && ADC1_ADC == 3
  ADC_1,
#endif
#if defined(ADC_2) && ADC2_ADC == 3
  ADC_2,
#endif
#if defined(ADC_3) && ADC3_ADC == 3
  ADC_3,
#endif
#if defined(ADC_4) && ADC4_ADC == 3
  ADC_4,
#endif
#if defined(ADC_5) && ADC5_ADC == 3
  ADC_5,
#endif
#if defined(ADC_6) && ADC6_ADC == 3
  ADC_6,
#endif
#if defined(ADC_7) && ADC7_ADC == 3
  ADC_7,
#endif
#if defined(ADC_8) && ADC8_ADC == 3
  ADC_8,
#endif
#if defined(ADC_9) && ADC9_ADC == 3
  ADC_9,
#endif
};
#define ADC3_CONFIGURED_CHANNELS (sizeof(adc3_channel_map) / sizeof(adc3_channel_map[0]))
#endif

uint8_t adc_error_flag = 0;
ADCDriver *adcp_err = NULL;

// adc_samples buffers
// The DMA buffer is circular with MAX_AV_NB_SAMPLE samples per channel and the
// conversion callback fires once per half, so each update averages
// MAX_AV_NB_SAMPLE/2 samples (MAX_AV_NB_SAMPLE must be even).
#if USE_AD1
static IN_DMA_SECTION(adcsample_t adc1_samples[ADC1_CONFIGURED_CHANNELS * MAX_AV_NB_SAMPLE]);
#endif
#if USE_AD2
static IN_DMA_SECTION(adcsample_t adc2_samples[ADC2_CONFIGURED_CHANNELS * MAX_AV_NB_SAMPLE]);
#endif
#if USE_AD3
static IN_DMA_SECTION(adcsample_t adc3_samples[ADC3_CONFIGURED_CHANNELS * MAX_AV_NB_SAMPLE]);
#endif

#if USE_AD1
static ADCConversionGroup adc1_group;
static uint32_t adc1_sum_tmp[ADC1_CONFIGURED_CHANNELS];
static uint8_t adc1_samples_tmp[ADC1_CONFIGURED_CHANNELS];
#endif
#if USE_AD2
static ADCConversionGroup adc2_group;
static uint32_t adc2_sum_tmp[ADC2_CONFIGURED_CHANNELS];
static uint8_t adc2_samples_tmp[ADC2_CONFIGURED_CHANNELS];
#endif
#if USE_AD3
static ADCConversionGroup adc3_group;
static uint32_t adc3_sum_tmp[ADC3_CONFIGURED_CHANNELS];
static uint8_t adc3_samples_tmp[ADC3_CONFIGURED_CHANNELS];
#endif

static struct adc_buf *adc_buffers[ADC_NUM_CHANNELS];

#if USE_ADC_WATCHDOG
// watchdog structure with adc bank and callback
static struct {
  ADCDriver *adc;
  adc_channels_num_t channel;
  adcsample_t vmin;
  adc_watchdog_callback cb;
} adc_watchdog;
#endif

/**
 * @brief Configure the ADC conversion group depending on the architecture
 *
 * @param cfg The configuration to be set
 * @param num_channels The number of channels in the ADC
 * @param channels The channel mapping to real channels
 * @param sample_rate The sample rate for all channels
 * @param end_cb The callback function at the end of conversion
 * @param error_cb The callback function whenever an error occurs
 */
static void adc_configure(ADCConversionGroup *cfg, uint8_t num_channels, const uint8_t channels[], uint32_t sample_rate,
                          adccallback_t end_cb, adcerrorcallback_t error_cb)
{
  // Set the general configuration
  cfg->circular = true;
  cfg->num_channels = num_channels;
  cfg->end_cb = end_cb;
  cfg->error_cb = error_cb;

  // Set to 16bits by default else try 12bit
#if defined(ADC_CFGR_RES_16BITS)
  cfg->cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_16BITS;
#elif defined(ADC_CFGR_RES_12BITS)
  cfg->cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS;
#else
  cfg->sqr1 = ADC_SQR1_NUM_CH(num_channels);
  cfg->cr2 = ADC_CR2_SWSTART;

#if defined(ADC_CR2_TSVREFE)
  cfg->cr2 |= ADC_CR2_TSVREFE;
#endif
#endif

  // Go through all the channels
  for (uint8_t i = 0; i < num_channels; i++) {
    uint8_t chan = channels[i];

#if defined(STM32H7XX) || defined(STM32F3XX) || defined(STM32G4XX) || defined(STM32L4XX)
    cfg->pcsel |= (1 << chan);
    cfg->smpr[chan / 10] |= sample_rate << (3 * (chan % 10));

    if (i < 4) {
      cfg->sqr[0] |= chan << (6 * (i + 1));
    } else if (i < 9) {
      cfg->sqr[1] |= chan << (6 * (i - 4));
    } else {
      cfg->sqr[2] |= chan << (6 * (i - 9));
    }
#else
    if (chan < 10) {
      cfg->smpr2 |= sample_rate << (3 * chan);
    } else {
      cfg->smpr1 |= sample_rate << (3 * (chan - 10));
    }

    if (i < 6) {
      cfg->sqr3 |= chan << (5 * i);
    } else if (i < 12) {
      cfg->sqr2 |= chan << (5 * (i - 6));
    } else {
      cfg->sqr3 |= chan << (5 * (i - 12));
    }
#endif
  }
}

/**
 * ADC callback processing
 *
 * Callback, fired after half of the buffer is filled (i.e. half of the samples
 * is collected). Since we are assuming continuous ADC conversion, the ADC state is
 * never equal to ADC_COMPLETE.
 *
 * @note    Averaging is done when the modules ask for ADC values
 * @param[in] adcp pointer to a @p ADCDriver object
 * @param[in] buffer pointer to a @p buffer with samples
 * @param[in] n number of samples
 */
static void adc_process_samples(ADCDriver *adcp, const uint8_t channel_ids[], size_t num_channels,
                                adcsample_t samples[], uint32_t sums[], uint8_t sample_counts[])
{
  if (adcp->state != ADC_STOP) {
    const size_t n = MAX_AV_NB_SAMPLE / 2U;
    // depending on half buffer that has just been filled
    // if adcIsBufferComplete return true, the last filled
    // half buffer start in the middle of buffer, else, is start at
    // beginiing of buffer
    const adcsample_t *buffer = samples + (adcIsBufferComplete(adcp) ?
                                n * num_channels : 0U);
    cacheBufferInvalidate(samples, MAX_AV_NB_SAMPLE * num_channels * sizeof(adcsample_t));

    for (size_t channel = 0; channel < num_channels; channel++) {
      const uint8_t buffer_id = channel_ids[channel];
      if (adc_buffers[buffer_id] != NULL) {
        sums[channel] = 0;
        if (n > 0) {
          sample_counts[channel] = n;
        } else {
          sample_counts[channel] = 1;
        }
        for (unsigned int sample = 0; sample < n; sample++) {
          sums[channel] += buffer[channel + sample * num_channels];
        }
      }
    }
    chSysLockFromISR();
    for (size_t channel = 0; channel < num_channels; channel++) {
      const uint8_t buffer_id = channel_ids[channel];
      if (adc_buffers[buffer_id] != NULL) {
        adc_buffers[buffer_id]->sum = sums[channel];
        adc_buffers[buffer_id]->av_nb_sample = sample_counts[channel];
      }
    }
#if USE_ADC_WATCHDOG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits" // remove warning when ADC_NUM_CHANNELS is 0 and test always false
    if ((adc_watchdog.adc == adcp) && (adc_watchdog.channel < ADC_NUM_CHANNELS) &&
        (adc_watchdog.cb != NULL) && (adc_buffers[adc_watchdog.channel] != NULL)) {
      if (adc_buffers[adc_watchdog.channel]->sum <
          (adc_buffers[adc_watchdog.channel]->av_nb_sample * adc_watchdog.vmin)) {
        adc_watchdog.cb();
      }
    }
#pragma GCC diagnostic pop
#endif // USE_ADC_WATCHDOG

    chSysUnlockFromISR();
  }
}

void adc1callback(ADCDriver *adcp)
{
#if USE_AD1
  adc_process_samples(adcp, adc1_channel_ids, ADC1_CONFIGURED_CHANNELS, adc1_samples,
                      adc1_sum_tmp, adc1_samples_tmp);
#else
  (void)adcp;
#endif
}

void adc2callback(ADCDriver *adcp)
{
#if USE_AD2
  adc_process_samples(adcp, adc2_channel_ids, ADC2_CONFIGURED_CHANNELS, adc2_samples,
                      adc2_sum_tmp, adc2_samples_tmp);
#else
  (void)adcp;
#endif
}

void adc3callback(ADCDriver *adcp)
{
#if USE_AD3
  adc_process_samples(adcp, adc3_channel_ids, ADC3_CONFIGURED_CHANNELS, adc3_samples,
                      adc3_sum_tmp, adc3_samples_tmp);
#else
  (void)adcp;
#endif
}

/**
 * Adc error callback
 *
 * Fired if DMA error happens or ADC overflows
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err)
{
  chSysLockFromISR();
  adcp_err = adcp;
  adc_error_flag = err;
  chSysUnlockFromISR();
}

/**
 * Link between ChibiOS ADC drivers and Paparazzi adc_buffers
 */
void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample)
{
  // check for out-of-bounds access
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits" // remove warning when ADC_NUM_CHANNELS is 0 and test always true
  if (adc_channel >= ADC_NUM_CHANNELS) { return; }
#pragma GCC diagnostic pop
  adc_buffers[adc_channel] = s;
  if (av_nb_sample <= MAX_AV_NB_SAMPLE) {
    s->av_nb_sample = av_nb_sample;
  } else {
    s->av_nb_sample = MAX_AV_NB_SAMPLE;
  }
}

/**
 * Adc init
 *
 * Initialize ADC drivers, buffers and start conversion in the background
 */
void adc_init(void)
{
  /* Init GPIO ports for ADC operation
   */
#if USE_ADC_1
  PRINT_CONFIG_MSG("Info: Using ADC_1");
  gpio_setup_pin_analog(ADC_1_GPIO_PORT, ADC_1_GPIO_PIN);
#endif
#if USE_ADC_2
  PRINT_CONFIG_MSG("Info: Using ADC_2");
  gpio_setup_pin_analog(ADC_2_GPIO_PORT, ADC_2_GPIO_PIN);
#endif
#if USE_ADC_3
  PRINT_CONFIG_MSG("Info: Using ADC_3");
  gpio_setup_pin_analog(ADC_3_GPIO_PORT, ADC_3_GPIO_PIN);
#endif
#if USE_ADC_4
  PRINT_CONFIG_MSG("Info: Using ADC_4");
  gpio_setup_pin_analog(ADC_4_GPIO_PORT, ADC_4_GPIO_PIN);
#endif
#if USE_ADC_5
  PRINT_CONFIG_MSG("Info: Using ADC_5");
  gpio_setup_pin_analog(ADC_5_GPIO_PORT, ADC_5_GPIO_PIN);
#endif
#if USE_ADC_6
  PRINT_CONFIG_MSG("Info: Using ADC_6");
  gpio_setup_pin_analog(ADC_6_GPIO_PORT, ADC_6_GPIO_PIN);
#endif
#if USE_ADC_7
  PRINT_CONFIG_MSG("Info: Using ADC_7");
  gpio_setup_pin_analog(ADC_7_GPIO_PORT, ADC_7_GPIO_PIN);
#endif
#if USE_ADC_8
  PRINT_CONFIG_MSG("Info: Using ADC_8");
  gpio_setup_pin_analog(ADC_8_GPIO_PORT, ADC_8_GPIO_PIN);
#endif
#if USE_ADC_9
  PRINT_CONFIG_MSG("Info: Using ADC_9");
  gpio_setup_pin_analog(ADC_9_GPIO_PORT, ADC_9_GPIO_PIN);
#endif

#if USE_ADC_WATCHDOG
  adc_watchdog.adc = NULL;
  adc_watchdog.cb = NULL;
  adc_watchdog.channel = 0;
  adc_watchdog.vmin = (1 << 12) - 1; // max adc
#endif

#if USE_AD1
  // Configure the ADC structure
  adc_configure(&adc1_group, ADC1_CONFIGURED_CHANNELS, adc1_channel_map, ADC_SAMPLE_RATE, adc1callback, adcerrorcallback);

  // Start ADC in continious conversion mode
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adc1_group, adc1_samples, MAX_AV_NB_SAMPLE);
#endif

#if USE_AD2
  // Configure the ADC structure
  adc_configure(&adc2_group, ADC2_CONFIGURED_CHANNELS, adc2_channel_map, ADC_SAMPLE_RATE, adc2callback, adcerrorcallback);

  // Start ADC in continuous conversion mode
  adcStart(&ADCD2, NULL);
  adcStartConversion(&ADCD2, &adc2_group, adc2_samples, MAX_AV_NB_SAMPLE);
#endif

#if USE_AD3
  // Configure the ADC structure
  adc_configure(&adc3_group, ADC3_CONFIGURED_CHANNELS, adc3_channel_map, ADC_SAMPLE_RATE, adc3callback, adcerrorcallback);

  // Start ADC in continious conversion mode
  adcStart(&ADCD3, NULL);
  adcStartConversion(&ADCD3, &adc3_group, adc3_samples, MAX_AV_NB_SAMPLE);
#endif
}

#if USE_ADC_WATCHDOG
void register_adc_watchdog(ADCDriver *adc, adc_channels_num_t channel, adcsample_t vmin,
                           adc_watchdog_callback cb)
{
#if USE_AD1
  if (adc == &ADCD1) {
    for (size_t i = 0; i < ADC1_CONFIGURED_CHANNELS; i++) {
      if (adc1_channel_map[i] == channel) {
        adc_watchdog.adc = adc;
        adc_watchdog.channel = adc1_channel_ids[i];
        adc_watchdog.vmin = vmin;
        adc_watchdog.cb = cb;
        return;
      }
    }
  }
#endif
#if USE_AD2
  if (adc == &ADCD2) {
    for (size_t i = 0; i < ADC2_CONFIGURED_CHANNELS; i++) {
      if (adc2_channel_map[i] == channel) {
        adc_watchdog.adc = adc;
        adc_watchdog.channel = adc2_channel_ids[i];
        adc_watchdog.vmin = vmin;
        adc_watchdog.cb = cb;
        return;
      }
    }
  }
#endif
#if USE_AD3
  if (adc == &ADCD3) {
    for (size_t i = 0; i < ADC3_CONFIGURED_CHANNELS; i++) {
      if (adc3_channel_map[i] == channel) {
        adc_watchdog.adc = adc;
        adc_watchdog.channel = adc3_channel_ids[i];
        adc_watchdog.vmin = vmin;
        adc_watchdog.cb = cb;
        return;
      }
    }
  }
#endif
}
#endif
