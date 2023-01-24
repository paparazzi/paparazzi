/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 * @file arch/stm32/mcu_periph/adc_arch.c
 * @ingroup stm32_arch
 *
 * Driver for the analog to digital converters on STM32.
 *
 * Usage:
 * Define flags for ADCs to use (can be explicitly disabled by setting to 0):
 *
 *   -DUSE_ADC_1 -DUSE_ADC_3=1 -DUSE_ADC_4=0
 *
 * would explicitly enable the ADC_1 and ADC_3 and disable ADC_4.
 *
 * The mapping of these virtual "board" ADC_x numbers to a concrete AD converter
 * and channel is done in the sw/airborne/boards/<boardname>.h header files.
 * Some ADCs are normally already enabled in the board files per default
 * (e.g. for battery voltage measurement).
 *
 */

/*
  For better understanding of timer and GPIO settings:

  Table of GPIO pins available per ADC:

  ADC1/2:                   ADC3:
  C0  -> PA0                C0  -> PA0
  C1  -> PA1                C1  -> PA1
  C2  -> PA2                C2  -> PA2
  C3  -> PA3                C3  -> PA3
  C4  -> PA4                C4  -> PF6
  C5  -> PA5                C5  -> PF7
  C6  -> PA6                C6  -> PF8
  C7  -> PA7                C7  -> PF9
  C8  -> PB0                C8  -> PF10
  C9  -> PB1
  C10 -> PC0                C10 -> PC0
  C11 -> PC1                C11 -> PC1
  C12 -> PC2                C12 -> PC2
  C13 -> PC3                C13 -> PC3
  C14 -> PC4
  C15 -> PC5

  Table of timers available per ADC (from libstm/src/stm32_adc.c):

  T1_TRGO:    Timer1 TRGO event (ADC1, ADC2 and ADC3)
  T1_CC4:     Timer1 capture compare4 (ADC1, ADC2 and ADC3)
  T2_TRGO:    Timer2 TRGO event (ADC1 and ADC2)
  T2_CC1:     Timer2 capture compare1 (ADC1 and ADC2)
  T3_CC4:     Timer3 capture compare4 (ADC1 and ADC2)
  T4_TRGO:    Timer4 TRGO event (ADC1 and ADC2)
  TIM8_CC4: External interrupt line 15 or Timer8 capture compare4 event (ADC1 and ADC2)
  T4_CC3:     Timer4 capture compare3 (ADC3 only)
  T8_CC2:     Timer8 capture compare2 (ADC3 only)
  T8_CC4:     Timer8 capture compare4 (ADC3 only)
  T5_TRGO:    Timer5 TRGO event (ADC3 only)
  T5_CC4:     Timer5 capture compare4 (ADC3 only)

  By setting ADC_ExternalTrigInjecConv_None, injected conversion
  is started by software instead of external trigger for any ADC.

  Table of APB per Timer (from libstm/src/stm32_tim.c):

  RCC_APB1: TIM2, TIM3, TIM4, TIM5, TIM7 (non-advanced timers)
  RCC_APB2: TIM1, TIM8 (advanced timers)

*/

#include "mcu_periph/adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <string.h>
#include "mcu_periph/gpio.h"
#include "mcu_arch.h"
#include "std.h"
#include BOARD_CONFIG


#ifndef NVIC_ADC_IRQ_PRIO
#define NVIC_ADC_IRQ_PRIO 0
#endif

#if defined(STM32F1)
#define ADC_SAMPLE_TIME ADC_SMPR_SMP_41DOT5CYC
#elif defined(STM32F4)
#define ADC_SAMPLE_TIME ADC_SMPR_SMP_56CYC
#endif

// Macros to automatically enable the correct ADC

#if defined(AD1_1_CHANNEL) || defined(AD1_2_CHANNEL) || defined(AD1_3_CHANNEL) || defined(AD1_4_CHANNEL)
#ifndef USE_AD1
#define USE_AD1 1
#endif
#endif

#if defined(AD2_1_CHANNEL) || defined(AD2_2_CHANNEL) || defined(AD2_3_CHANNEL) || defined(AD2_4_CHANNEL)
#ifndef USE_AD2
#define USE_AD2 1
#endif
#endif

#if defined(STM32F4)

#if defined(AD3_1_CHANNEL) || defined(AD3_2_CHANNEL) || defined(AD3_3_CHANNEL) || defined(AD3_4_CHANNEL)
#ifndef USE_AD3
#define USE_AD3 1
#endif
#endif

#define RST_ADC1 RST_ADC
#define RST_ADC2 RST_ADC
#define RST_ADC3 RST_ADC

#else // !STM32F4
// ADC 3 not supported on STM32F1
#undef USE_AD3
#define USE_AD3 0
#endif

#if USE_AD1
PRINT_CONFIG_MSG("Analog to Digital Coverter 1 active")
#endif
#if USE_AD2
PRINT_CONFIG_MSG("Analog to Digital Coverter 2 active")
#endif
#if USE_AD3
PRINT_CONFIG_MSG("Analog to Digital Coverter 3 active")
#endif
#if !USE_AD1 && !USE_AD2 && !USE_AD3 && !defined FBW
#warning ALL ADC CONVERTERS INACTIVE
#endif

#ifndef ADC_TIMER_PERIOD
#define ADC_TIMER_PERIOD 10000
#endif

/** Timer frequency for ADC
 * Timer will trigger an update event after reaching the period reload value.
 * New conversion is triggered on update event.
 * ADC measuerement frequency is hence ADC_TIMER_FREQUENCY / ADC_TIMER_PERIOD.
 */
#ifndef ADC_TIMER_FREQUENCY
#define ADC_TIMER_FREQUENCY 2000000
#endif

/***************************************/
/***   STATIC FUNCTION PROTOTYPES    ***/
/***************************************/

static inline void adc_init_single(uint32_t adc, uint8_t nb_channels, uint8_t *channel_map);

static inline void adc_push_sample(struct adc_buf *buf,
                                   uint16_t sample);

static inline void adc_init_rcc(void);
static inline void adc_init_irq(void);


/********************************/
/***     GLOBAL VARIABLES     ***/
/********************************/

/* Only 4 ADC channels may be enabled at the same time
 * on each ADC, as there are only 4 injection registers.
 * Currently, the enums adc1_channels and adc2_channels only
 * serve to resolve the number of channels on each ADC.
 * There are 3 separate buffer lists, each holds the addresses of the actual adc buffers
 * for the particular adc converter.
 */

static uint8_t nb_adc1_channels = 0;
static uint8_t nb_adc2_channels = 0;
static uint8_t nb_adc3_channels = 0;

#if USE_AD1 || USE_AD2 || USE_AD3
#define ADC_NUM_CHANNELS 4
#endif

#if USE_AD1
/// List of buffers, one for each active channel.
static struct adc_buf *adc1_buffers[ADC_NUM_CHANNELS];
#endif
#if USE_AD2
/// List of buffers, one for each active channel.
static struct adc_buf *adc2_buffers[ADC_NUM_CHANNELS];
#endif
#if USE_AD3
/// List of buffers, one for each active channel.
static struct adc_buf *adc3_buffers[ADC_NUM_CHANNELS];
#endif

#if USE_ADC_WATCHDOG
#include "mcu_periph/sys_time.h"
// watchdog structure with adc bank and callback
static struct {
  uint32_t timeStamp;
  uint32_t adc;
  adc_watchdog_callback cb;
} adc_watchdog;
#endif

/***************************************/
/***   PUBLIC FUNCTION DEFINITIONS   ***/
/***************************************/

void adc_init(void)
{
#if USE_AD1 || USE_AD2 || USE_AD3
  uint8_t x = 0;

  // ADC channel mapping
  uint8_t adc_channel_map[4];
#endif

  /* Init GPIO ports for ADC operation
   */
#if USE_ADC_1
  PRINT_CONFIG_MSG("Info: Using ADC_1")
  gpio_setup_pin_analog(ADC_1_GPIO_PORT, ADC_1_GPIO_PIN);
#endif
#if USE_ADC_2
  PRINT_CONFIG_MSG("Info: Using ADC_2")
  gpio_setup_pin_analog(ADC_2_GPIO_PORT, ADC_2_GPIO_PIN);
#endif
#if USE_ADC_3
  PRINT_CONFIG_MSG("Info: Using ADC_3")
  gpio_setup_pin_analog(ADC_3_GPIO_PORT, ADC_3_GPIO_PIN);
#endif
#if USE_ADC_4
  PRINT_CONFIG_MSG("Info: Using ADC_4")
  gpio_setup_pin_analog(ADC_4_GPIO_PORT, ADC_4_GPIO_PIN);
#endif
#if USE_ADC_5
  PRINT_CONFIG_MSG("Info: Using ADC_5")
  gpio_setup_pin_analog(ADC_5_GPIO_PORT, ADC_5_GPIO_PIN);
#endif
#if USE_ADC_6
  PRINT_CONFIG_MSG("Info: Using ADC_6")
  gpio_setup_pin_analog(ADC_6_GPIO_PORT, ADC_6_GPIO_PIN);
#endif
#if USE_ADC_7
  PRINT_CONFIG_MSG("Info: Using ADC_7")
  gpio_setup_pin_analog(ADC_7_GPIO_PORT, ADC_7_GPIO_PIN);
#endif
#if USE_ADC_8
  PRINT_CONFIG_MSG("Info: Using ADC_8")
  gpio_setup_pin_analog(ADC_8_GPIO_PORT, ADC_8_GPIO_PIN);
#endif
#if USE_ADC_9
  PRINT_CONFIG_MSG("Info: Using ADC_9")
  gpio_setup_pin_analog(ADC_9_GPIO_PORT, ADC_9_GPIO_PIN);
#endif

  // Init clock and irq
  adc_init_rcc();
  adc_init_irq();

  /* If fewer than 4 channels are active, say 3, then they are placed in to
   * injection slots 2,3 and 4 because the stm32 architecture converts injected
   * slots 2,3 and 4 and skips slot 1 instead of logicaly converting slots 1,2
   * and 3 and leave slot 4.
   * EXAMPLE OF ADC EXECUTION ORDER WHEN WE HAVE SAY 2 ADC INPUTS USED on ADC1
   * The first board adc channel ADC1_1 is mapped to injected channel 3 and ADC1_2
   * to injected channel 4 and because the conversions start from the lowest
   * injection channel used, 3 in our case, injected channel 3 data will be
   * located at JDR1 and 4 to JDR2 so JDR1 = ADC1_1 and JDR2 = ADC1_2.
   * That's why "adc_channel_map" has this descending order.
   */

  nb_adc1_channels = 0;
#if USE_AD1
#ifdef AD1_1_CHANNEL
  adc_channel_map[AD1_1] = AD1_1_CHANNEL;
  nb_adc1_channels++;
#endif
#ifdef AD1_2_CHANNEL
  adc_channel_map[AD1_2] = AD1_2_CHANNEL;
  nb_adc1_channels++;
#endif
#ifdef AD1_3_CHANNEL
  adc_channel_map[AD1_3] = AD1_3_CHANNEL;
  nb_adc1_channels++;
#endif
#ifdef AD1_4_CHANNEL
  adc_channel_map[AD1_4] = AD1_4_CHANNEL;
  nb_adc1_channels++;
#endif
  // initialize buffer pointers with 0 (not set). Buffer null pointers will be ignored in interrupt
  // handler, which is important as there are no buffers registered at the time the ADC trigger
  // interrupt is enabled.
  for (x = 0; x < 4; x++) { adc1_buffers[x] = NULL; }
  adc_init_single(ADC1, nb_adc1_channels, adc_channel_map);
#endif // USE_AD1


  nb_adc2_channels = 0;
#if USE_AD2
#ifdef AD2_1_CHANNEL
  adc_channel_map[AD2_1 - nb_adc1_channels] = AD2_1_CHANNEL;
  nb_adc2_channels++;
#endif
#ifdef AD2_2_CHANNEL
  adc_channel_map[AD2_2 - nb_adc1_channels] = AD2_2_CHANNEL;
  nb_adc2_channels++;
#endif
#ifdef AD2_3_CHANNEL
  adc_channel_map[AD2_3 - nb_adc1_channels] = AD2_3_CHANNEL;
  nb_adc2_channels++;
#endif
#ifdef AD2_4_CHANNEL
  adc_channel_map[AD2_4 - nb_adc1_channels] = AD2_4_CHANNEL;
  nb_adc2_channels++;
#endif
  // initialize buffer pointers with 0 (not set). Buffer null pointers will be ignored in interrupt
  // handler, which is important as there are no buffers registered at the time the ADC trigger
  // interrupt is enabled.
  for (x = 0; x < 4; x++) { adc2_buffers[x] = NULL; }
  adc_init_single(ADC2, nb_adc2_channels, adc_channel_map);
#endif // USE_AD2


  nb_adc3_channels = 0;
#if USE_AD3
#ifdef AD3_1_CHANNEL
  adc_channel_map[AD3_1 - nb_adc1_channels - nb_adc2_channels] = AD3_1_CHANNEL;
  nb_adc3_channels++;
#endif
#ifdef AD3_2_CHANNEL
  adc_channel_map[AD3_2 - nb_adc1_channels - nb_adc2_channels] = AD3_2_CHANNEL;
  nb_adc3_channels++;
#endif
#ifdef AD3_3_CHANNEL
  adc_channel_map[AD3_3 - nb_adc1_channels - nb_adc2_channels] = AD3_3_CHANNEL;
  nb_adc3_channels++;
#endif
#ifdef AD3_4_CHANNEL
  adc_channel_map[AD3_4 - nb_adc1_channels - nb_adc2_channels] = AD3_4_CHANNEL;
  nb_adc3_channels++;
#endif
  // initialize buffer pointers with 0 (not set). Buffer null pointers will be ignored in interrupt
  // handler, which is important as there are no buffers registered at the time the ADC trigger
  // interrupt is enabled.
  for (x = 0; x < 4; x++) { adc3_buffers[x] = NULL; }
  adc_init_single(ADC3, nb_adc3_channels, adc_channel_map);
#endif // USE_AD3

#if USE_ADC_WATCHDOG
  adc_watchdog.cb = NULL;
  adc_watchdog.timeStamp = 0;
#endif

}

void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample)
{
  if (adc_channel < nb_adc1_channels) {
#if USE_AD1
    adc1_buffers[adc_channel] = s;
#endif
  } else if (adc_channel < (nb_adc1_channels + nb_adc2_channels)) {
#if USE_AD2
    adc2_buffers[adc_channel - nb_adc1_channels] = s;
#endif
  } else if (adc_channel < (nb_adc1_channels + nb_adc2_channels + nb_adc3_channels)) {
#if USE_AD3
    adc3_buffers[adc_channel - (nb_adc1_channels + nb_adc2_channels)] = s;
#endif
  }

  s->av_nb_sample = av_nb_sample;

}

#if USE_ADC_WATCHDOG
void register_adc_watchdog(uint32_t adc, uint8_t chan, uint16_t low, uint16_t high, adc_watchdog_callback cb)
{
  adc_watchdog.adc = adc;
  adc_watchdog.cb = cb;

  // activated adc watchdog of a single injected channel with interrupt
  adc_set_watchdog_low_threshold(adc, low);
  adc_set_watchdog_high_threshold(adc, high);
  adc_enable_analog_watchdog_injected(adc);
  adc_enable_analog_watchdog_on_selected_channel(adc, chan);
  adc_enable_awd_interrupt(adc);
}
#endif

/**************************************/
/***  PRIVATE FUNCTION DEFINITIONS  ***/
/**************************************/

#if defined(USE_AD_TIM4)
#define TIM_ADC      TIM4
#define RCC_TIM_ADC  RCC_TIM4
#elif defined(USE_AD_TIM1)
#define TIM_ADC      TIM1
#define RCC_TIM_ADC  RCC_TIM1
#else
#define TIM_ADC      TIM2
#define RCC_TIM_ADC  RCC_TIM2
#endif

/** Configure and enable RCC for peripherals (ADC1, ADC2, Timer) */
static inline void adc_init_rcc(void)
{
#if USE_AD1 || USE_AD2 || USE_AD3
  /* Timer peripheral clock enable. */
  rcc_periph_clock_enable(RCC_TIM_ADC);
#if defined(STM32F4)
  adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
#endif

  /* Enable ADC peripheral clocks. */
#if USE_AD1
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_reset_pulse(RST_ADC1);
#endif
#if USE_AD2
  rcc_periph_clock_enable(RCC_ADC2);
  rcc_periph_reset_pulse(RST_ADC2);
#endif
#if USE_AD3
  rcc_periph_clock_enable(RCC_ADC3);
  rcc_periph_reset_pulse(RST_ADC3);
#endif

  /* Time Base configuration */
  timer_set_mode(TIM_ADC, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  /* timer counts with ADC_TIMER_FREQUENCY */
  uint32_t timer_clk = timer_get_frequency(TIM_ADC);
  timer_set_prescaler(TIM_ADC, (timer_clk / ADC_TIMER_FREQUENCY) - 1);

  timer_set_period(TIM_ADC, ADC_TIMER_PERIOD);
  /* Generate TRGO on every update (when counter reaches period reload value) */
  timer_set_master_mode(TIM_ADC, TIM_CR2_MMS_UPDATE);
  timer_enable_counter(TIM_ADC);

#endif // USE_AD1 || USE_AD2 || USE_AD3
}

/** Configure and enable ADC interrupt */
static inline void adc_init_irq(void)
{
#if defined(STM32F1)
  nvic_set_priority(NVIC_ADC1_2_IRQ, NVIC_ADC_IRQ_PRIO);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);
#elif defined(STM32F4)
  nvic_set_priority(NVIC_ADC_IRQ, NVIC_ADC_IRQ_PRIO);
  nvic_enable_irq(NVIC_ADC_IRQ);
#endif
}


static inline void adc_init_single(uint32_t adc, uint8_t nb_channels, uint8_t *channel_map)
{
  // Paranoia, must be down for 2+ ADC clock cycles before calibration
  adc_power_off(adc);

  /* Configure ADC */
  /* Explicitly setting most registers, reset/default values are correct for most */
  /* Set CR1 register. */
  /* Clear AWDEN */
  adc_disable_analog_watchdog_regular(adc);
  /* Clear JAWDEN */
  adc_disable_analog_watchdog_injected(adc);
  /* Clear DISCEN */
  adc_disable_discontinuous_mode_regular(adc);
  /* Clear JDISCEN */
  adc_disable_discontinuous_mode_injected(adc);
  /* Clear JAUTO */
  adc_disable_automatic_injected_group_conversion(adc);
  /* Set SCAN */
  adc_enable_scan_mode(adc);
  /* Enable ADC<X> JEOC interrupt (Set JEOCIE) */
  adc_enable_eoc_interrupt_injected(adc);
  /* Clear AWDIE */
  adc_disable_awd_interrupt(adc);
  /* Clear EOCIE */
  adc_disable_eoc_interrupt(adc);

  /* Set CR2 register. */
  /* Clear TSVREFE */
  adc_disable_temperature_sensor();
  /* Clear EXTTRIG */
  adc_disable_external_trigger_regular(adc);
  /* Clear ALIGN */
  adc_set_right_aligned(adc);
  /* Clear DMA */
  adc_disable_dma(adc);
  /* Clear CONT */
  adc_set_single_conversion_mode(adc);

  //uint8_t x = 0;
  //for (x = 0; x < nb_channels; x++) {
  //  adc_set_sample_time(adc, channel_map[x], ADC_SAMPLE_TIME);
  //}
  adc_set_sample_time_on_all_channels(adc, ADC_SAMPLE_TIME);

  adc_set_injected_sequence(adc, nb_channels, channel_map);

#if USE_AD_TIM4
  PRINT_CONFIG_MSG("Info: Using TIM4 for ADC")
#if defined(STM32F1)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM4_TRGO);
#elif defined(STM32F4)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM4_TRGO, ADC_CR2_JEXTEN_BOTH_EDGES);
#endif
#elif USE_AD_TIM1
  PRINT_CONFIG_MSG("Info: Using TIM1 for ADC")
#if defined(STM32F1)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM1_TRGO);
#elif defined(STM32F4)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM1_TRGO, ADC_CR2_JEXTEN_BOTH_EDGES);
#endif
#else
  PRINT_CONFIG_MSG("Info: Using default TIM2 for ADC")
#if defined(STM32F1)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM2_TRGO);
#elif defined(STM32F4)
  adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM2_TRGO, ADC_CR2_JEXTEN_BOTH_EDGES);
#endif
#endif

  /* Enable ADC<X> */
  adc_power_on(adc);
#if defined(STM32F1)
  /* Rest ADC<X> calibaration register and wait until done */
  adc_reset_calibration(adc);
  /* Start ADC<X> calibaration and wait until done */
  adc_calibrate(adc);
#endif

  return;
} // adc_init_single


static inline void adc_push_sample(struct adc_buf *buf, uint16_t value)
{
  uint8_t new_head = buf->head + 1;

  if (new_head >= buf->av_nb_sample) {
    new_head = 0;
  }
  buf->sum -= buf->values[new_head];
  buf->values[new_head] = value;
  buf->sum += value;
  buf->head = new_head;
}

/*********************************/
/***   ADC INTERRUPT HANDLER   ***/
/*********************************/

#if defined(STM32F1)
void adc1_2_isr(void)
#elif defined(STM32F4)
void adc_isr(void)
#endif
{
#if USE_AD1 || USE_AD2 || USE_AD3
  uint8_t channel = 0;
  uint16_t value  = 0;
  struct adc_buf *buf;
#endif

#if USE_ADC_WATCHDOG
  /*
    We need adc sampling fast enough to detect battery plug out, but we did not
    need to get actual actual value so fast. So timer fire adc conversion fast,
    at least 500 hz, but we inject adc value in sampling buffer only at 50hz
   */
  const uint32_t timeStampDiff = get_sys_time_msec() - adc_watchdog.timeStamp;
  const bool shouldAccumulateValue = timeStampDiff > 20;
  if (shouldAccumulateValue) {
    adc_watchdog.timeStamp = get_sys_time_msec();
  }

  if (adc_watchdog.cb != NULL) {
    if (adc_awd(adc_watchdog.adc)) {
      ADC_SR(adc_watchdog.adc) &= ~ADC_SR_AWD; // clear int flag
      adc_watchdog.cb();
    }
  }
#endif

#if USE_AD1
  // Clear Injected End Of Conversion
  if (adc_eoc_injected(ADC1)) {
    ADC_SR(ADC1) &= ~ADC_SR_JEOC;
#if USE_ADC_WATCHDOG
    if (shouldAccumulateValue) {
#endif
      for (channel = 0; channel < nb_adc1_channels; channel++) {
        buf = adc1_buffers[channel];
        if (buf) {
          value = adc_read_injected(ADC1, channel + 1);
          adc_push_sample(buf, value);
        }
      }
#if USE_ADC_WATCHDOG
    }
#endif
  }
#endif // USE_AD1

#if USE_AD2
  if (adc_eoc_injected(ADC2)) {
    ADC_SR(ADC2) &= ~ADC_SR_JEOC;
#if USE_ADC_WATCHDOG
    if (shouldAccumulateValue) {
#endif
      for (channel = 0; channel < nb_adc2_channels; channel++) {
        buf = adc2_buffers[channel];
        if (buf) {
          value = adc_read_injected(ADC2, channel + 1);
          adc_push_sample(buf, value);
        }
      }
#if USE_ADC_WATCHDOG
    }
#endif
  }
#endif // USE_AD2

#if USE_AD3
  if (adc_eoc_injected(ADC3)) {
    ADC_SR(ADC3) &= ~ADC_SR_JEOC;
#if USE_ADC_WATCHDOG
    if (shouldAccumulateValue) {
#endif
      for (channel = 0; channel < nb_adc3_channels; channel++) {
        buf = adc3_buffers[channel];
        if (buf) {
          value = adc_read_injected(ADC3, channel + 1);
          adc_push_sample(buf, value);
        }
      }
#if USE_ADC_WATCHDOG
    }
#endif
  }
#endif // USE_AD3

  return;
}
