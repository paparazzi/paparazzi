/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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
 * @file arch/stm32/mcu_periph/adc_arch.c
 * @ingroup stm32_arch
 *
 * Driver for the analog to digital converters on STM32.
 *
 * Usage:
 * Define flags for ADCs to use and their channels:
 *
 *   -DUSE_AD1 -DUSE_AD1_1 -DUSE_AD1_3
 *
 * would enable ADC1 and it's channels 1 and 3.
 *
 */

/*
  For better understanding of timer and GPIO settings:

  Table of GPIO pins available per ADC:

  ADC1/2:                   ADC3:
  C0  -> PA0				C0  -> PA0
  C1  -> PA1				C1  -> PA1
  C2  -> PA2				C2  -> PA2
  C3  -> PA3				C3  -> PA3
  C4  -> PA4				C4  -> PF6
  C5  -> PA5				C5  -> PF7
  C6  -> PA6				C6  -> PF8
  C7  -> PA7				C7  -> PF9
  C8  -> PB0				C8  -> PF10
  C9  -> PB1
  C10 -> PC0				C10 -> PC0
  C11 -> PC1				C11 -> PC1
  C12 -> PC2				C12 -> PC2
  C13 -> PC3				C13 -> PC3
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
#if defined(STM32F1)
#include <libopencm3/stm32/f1/adc.h>
#define ADC_SAMPLE_TIME ADC_SMPR_SMP_41DOT5CYC
#elif defined(STM32F4)
#include <libopencm3/stm32/f4/adc.h>
#define ADC_SAMPLE_TIME ADC_SMPR_SMP_56CYC
#endif
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <string.h>
#include "std.h"
#include "led.h"
#include BOARD_CONFIG

/***************************************************************************************************/
/***************************   DO NOT EDIT - CONVINIENCE MACROS   **********************************/
/***************************************************************************************************/

// CONVINIENCE MACROS FOR MAKING EDITING SIMPLER - DO NOT EDIT - IT IS NOT NEEDED! 

#if defined(USE_AD1_1) || defined(USE_AD1_2) || defined(USE_AD1_3) || defined(USE_AD1_4)
#if !defined(USE_AD1)
#define USE_AD1   1
#endif
#pragma message "Analog to Digital Coverter 1 active"
#endif

#if defined(USE_AD2_1) || defined(USE_AD2_2) || defined(USE_AD2_3) || defined(USE_AD2_4)
#if !defined(USE_AD2)
#define USE_AD2   1
#endif
#pragma message "Analog to Digital Coverter 2 active"
#endif

#if defined(USE_AD3_1) || defined(USE_AD3_2) || defined(USE_AD3_3) || defined(USE_AD3_4)
#if !defined(USE_AD3)
#define USE_AD3   1
#endif
#pragma message "Analog to Digital Coverter 3 active"
#endif

#if !defined(AD1_OFFSET)
#define AD1_OFFSET	0
#endif	
#if !defined(AD2_OFFSET)
#define AD2_OFFSET	4
#endif	
#if !defined(AD3_OFFSET)
#define AD3_OFFSET	8
#endif	

//#if !defined(USE_ADC_1)
#if !defined(ADC_1_GPIO_CLOCK_PORT)
#define ADC_1_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_1_INIT)
#define ADC_1_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_2)
#if !defined(ADC_2_GPIO_CLOCK_PORT)
#define ADC_2_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_2_INIT)
#define ADC_2_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_3)
#if !defined(ADC_3_GPIO_CLOCK_PORT)
#define ADC_3_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_3_INIT)
#define ADC_3_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_4)
#if !defined(ADC_4_GPIO_CLOCK_PORT)
#define ADC_4_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_4_INIT)
#define ADC_4_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_5)
#if !defined(ADC_5_GPIO_CLOCK_PORT)
#define ADC_5_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_5_INIT)
#define ADC_5_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_6)
#if !defined(ADC_6_GPIO_CLOCK_PORT)
#define ADC_6_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_6_INIT)
#define ADC_6_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_7)
#if !defined(ADC_7_GPIO_CLOCK_PORT)
#define ADC_7_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_7_INIT)
#define ADC_7_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_8)
#if !defined(ADC_8_GPIO_CLOCK_PORT)
#define ADC_8_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_8_INIT)
#define ADC_8_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_9)
#if !defined(ADC_9_GPIO_CLOCK_PORT)
#define ADC_9_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_9_INIT)
#define ADC_9_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_10)
#if !defined(ADC_10_GPIO_CLOCK_PORT)
#define ADC_10_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_10_INIT)
#define ADC_10_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_11)
#if !defined(ADC_11_GPIO_CLOCK_PORT)
#define ADC_11_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_11_INIT)
#define ADC_11_INIT()		{}
#endif
//#endif

//#if !defined(USE_ADC_12)
#if !defined(ADC_12_GPIO_CLOCK_PORT)
#define ADC_12_GPIO_CLOCK_PORT	0
#endif
#if !defined(ADC_12_INIT)
#define ADC_12_INIT()		{}
#endif
//#endif

//#if defined(ADC_GPIO_CLOCK_PORT)
#undef ADC_GPIO_CLOCK_PORT
#define ADC_GPIO_CLOCK_PORT ( ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT |   \
                              ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT |   \
                              ADC_5_GPIO_CLOCK_PORT | ADC_6_GPIO_CLOCK_PORT |   \
                              ADC_7_GPIO_CLOCK_PORT | ADC_8_GPIO_CLOCK_PORT |   \
                              ADC_9_GPIO_CLOCK_PORT | ADC_10_GPIO_CLOCK_PORT |  \
                              ADC_11_GPIO_CLOCK_PORT | ADC_12_GPIO_CLOCK_PORT   \
                            )
//#endif
#define ADC_GPIO_INIT(gpio) {   ADC_1_INIT();  ADC_2_INIT();  ADC_3_INIT();  ADC_4_INIT();   \
				ADC_5_INIT();  ADC_6_INIT();  ADC_7_INIT();  ADC_8_INIT();   \
				ADC_9_INIT();  ADC_10_INIT(); ADC_11_INIT(); ADC_12_INIT();  \
			    }

#if !defined(USE_AD1) && !defined(USE_AD2) && !defined(USE_AD3)
#error ALL ADC CONVERTERS INACTIVE
#endif

volatile uint8_t adc_new_data_trigger;

/* Static functions */

static inline void adc_init_single(uint32_t adc);

static inline void adc_push_sample(struct adc_buf * buf,
                                   uint16_t sample);

static inline void adc_init_rcc( void );
static inline void adc_init_irq( void );

/*
  Only 4 ADC channels may be enabled at the same time
  on each ADC, as there are only 4 injection registers.
*/

/*
  Currently, the enums adc1_channels and adc2_channels only
  serve to resolve the number of channels on each ADC.
*/

/*
  Separate buffers for each ADC.
  Every ADC has a list of buffers, one for each active
  channel.
*/

#ifdef USE_AD1
/// List of buffers, one for each active channel.
static struct adc_buf * adc1_buffers[NB_ADC1_CHANNELS];
//Maps integer value x to ADC_InjectedChannel_x so they can be iterated safely
volatile uint32_t *adc1_injected_channels[4];
#endif
#if defined(USE_AD2) && defined(STM32F4)
/// List of buffers, one for each active channel.
static struct adc_buf * adc2_buffers[NB_ADC2_CHANNELS];
//Maps integer value x to ADC_InjectedChannel_x so they can be iterated safely
volatile uint32_t *adc2_injected_channels[4];
#endif
#if defined(USE_AD3) && defined(STM32F4)
/// List of buffers, one for each active channel.
static struct adc_buf * adc3_buffers[NB_ADC3_CHANNELS];
//Maps integer value x to ADC_InjectedChannel_x so they can be iterated safely
volatile uint32_t *adc3_injected_channels[4];
#endif


// Maps integer value x to ADC_Channel_y so they can be iterated incrementally.
// Example:
// 0 --> ADC_Channel_5
// 1 --> ADC_Channel_8
// 2 --> ADC_Channel_13
static uint8_t adc_channel_map[4];

void adc_buf_channel(uint8_t adc_channel, struct adc_buf * s, uint8_t av_nb_sample)
{

if (adc_channel < (4+AD1_OFFSET)){
   adc1_buffers[adc_channel] = s;

}
#if defined(USE_AD2) && defined(STM32F4)
else if (adc_channel < (4+AD2_OFFSET)){
         adc2_buffers[adc_channel-AD2_OFFSET] = s;

      }
#endif
#if defined(USE_AD3) && defined(STM32F4)
       else if (adc_channel < (4+AD3_OFFSET)){
               adc3_buffers[adc_channel-AD3_OFFSET] = s;
            }
#else
#endif

   s->av_nb_sample = av_nb_sample;



return;
}

// #define USE_AD_TIM4
/* Configure and enable RCC for peripherals (ADC1, ADC2, Timer) */
static inline void adc_init_rcc( void )
{
#if defined(USE_AD1) || (defined(USE_AD2)&&defined(STM32F4)) || (defined(USE_AD3)&&defined(STM32F4))
  uint32_t timer;
  volatile uint32_t *rcc_apbenr;
  uint32_t rcc_apb;
#if defined(USE_AD_TIM4)
  timer   = TIM4;
  rcc_apbenr = &RCC_APB1ENR;
  rcc_apb = RCC_APB1ENR_TIM4EN;
#elif defined(USE_AD_TIM1)
  timer   = TIM1;
  rcc_apbenr = &RCC_APB2ENR;
  rcc_apb = RCC_APB2ENR_TIM1EN;
#else
  timer   = TIM2;
  rcc_apbenr = &RCC_APB1ENR;
  rcc_apb = RCC_APB1ENR_TIM2EN;
#endif

  /* Timer peripheral clock enable. */
  rcc_peripheral_enable_clock(rcc_apbenr, rcc_apb);
  /* GPIO peripheral clock enable. */
#if defined(STM32F1)
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN |
                              RCC_APB2ENR_IOPCEN);
#elif defined(STM32F4)
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, ADC_GPIO_CLOCK_PORT);
  adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
#endif

  /* Enable ADC peripheral clocks. */
#ifdef USE_AD1
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
#endif
#if defined(USE_AD2) && defined(STM32F4)
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
#endif
#if defined(USE_AD3) && defined(STM32F4)
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC3EN);
#endif

  /* Time Base configuration */
  timer_reset(timer);
  timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
#if defined(STM32F1)
  timer_set_period(timer, 0xFF);
  timer_set_prescaler(timer, 0x8);
#elif defined(STM32F4)
  timer_set_period(timer, 0xFFFF);
  timer_set_prescaler(timer, 0x53);
#endif
  //timer_set_clock_division(timer, 0x0);
  /* Generate TRGO on every update. */
  timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);
  timer_enable_counter(timer);

#endif // defined (USE_AD1) || defined (USE_AD2) || (defined(USE_AD3) && defined(STM32F4))
}

/* Configure and enable ADC interrupt */
static inline void adc_init_irq( void )
{
#if defined(STM32F1)
  nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);
#elif defined(STM32F4)
  nvic_set_priority(NVIC_ADC_IRQ, 0);
  nvic_enable_irq(NVIC_ADC_IRQ);
#endif
}

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
static inline void adc_init_single(uint32_t adc)
{
  uint8_t num_channels = 0, rank = 3;
  uint8_t channels[4];

  // Paranoia, must be down for 2+ ADC clock cycles before calibration
  adc_off(adc);

  /* enable adc clock and configure pins as analog inputs */
  if (adc == ADC1) {
#ifdef USE_AD1
    num_channels = NB_ADC1_CHANNELS;
#endif
#if defined(USE_AD2) && defined(STM32F4)
  }
  else if (adc == ADC2) {
          num_channels = NB_ADC2_CHANNELS;
#endif
#if defined(USE_AD3) && defined(STM32F4)
  } 
  else if (adc == ADC3) {
          num_channels = NB_ADC3_CHANNELS;
#endif
  }

#if defined(ADC1_GPIO_INIT)
    ADC1_GPIO_INIT();   // For Lisa m v2
#else
    ADC_GPIO_INIT();
#endif

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
#if defined(STM32F1)
  adc_disable_temperature_sensor(adc);
#elif defined(STM32F4)
  adc_disable_temperature_sensor();
#endif
  /* Clear EXTTRIG */
  adc_disable_external_trigger_regular(adc);
  /* Clear ALIGN */
  adc_set_right_aligned(adc);
  /* Clear DMA */
  adc_disable_dma(adc);
  /* Clear CONT */
  adc_set_single_conversion_mode(adc);

// Reverse channel order for compensating the stm32's peculiar order for injected channels.
//all this because stm32 does not start from channel 0 when less then 4 channels are specified
//which means that if 3 channels are specified, conversion is done to ch 2,3,4 but not 1.
//EXAMPLE: the first BOARD_ADC_CHANNEL (adc_channel_map[0]) is mapped to JSQ4 and not JSQ1 (channels[0])
  rank = 3;
  if (adc_channel_map[0] < 16 ) {
    adc_set_sample_time(adc, adc_channel_map[0], ADC_SAMPLE_TIME);
    channels[rank] = adc_channel_map[0]; 
    rank--;
  }
  if (adc_channel_map[1] < 16) {
    adc_set_sample_time(adc, adc_channel_map[1], ADC_SAMPLE_TIME);
    channels[rank] = adc_channel_map[1];
    rank--;
  }
  if (adc_channel_map[2] < 16) {
    adc_set_sample_time(adc, adc_channel_map[2], ADC_SAMPLE_TIME);
    channels[rank] = adc_channel_map[2];
    rank--;
  }
  if (adc_channel_map[3] < 16) {
    adc_set_sample_time(adc, adc_channel_map[3], ADC_SAMPLE_TIME);
    channels[rank] = adc_channel_map[3];
  }

  adc_set_injected_sequence(adc, num_channels, channels);

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
  /* Enable ADC<X> reset calibaration register */
  adc_reset_calibration(adc);
  /* Check the end of ADC<X> reset calibration */
  while ((ADC_CR2(adc) & ADC_CR2_RSTCAL) != 0);
  /* Start ADC<X> calibaration */
  adc_calibration(adc);
  /* Check the end of ADC<X> calibration */
  while ((ADC_CR2(adc) & ADC_CR2_CAL) != 0);
#endif
} // adc_init_single


void adc_init( void ) {

  /* initialize buffer pointers with 0 (not set).
     buffer null pointers will be ignored in interrupt
     handler, which is important as there are no
     buffers registered at the time the ADC trigger
     interrupt is enabled.
  */
// EXAMPLE OF ADC EXECUTION ORDER WHEN WE HAVE SAY 2 ADC INPUTS USED on ADC1 
// Because the first board adc channel ADC1_1 is mapped to injected channel 4 and ADC1_2 
// to injected channel 3 but the conversion starts from the lowest injection channel used, 
// 3 in our case, injected channel 3 data wiil be located at JDR1 and 4 to JDR2 so 
// JDR1 = ADC1_2 and JDR2 = ADC1_1
  uint8_t channel;
#ifdef USE_AD1
  for(channel = 0; channel < NB_ADC1_CHANNELS; channel++){  adc1_buffers[channel] = NULL; }
  volatile uint32_t* tmp_channels_1[]={
					&ADC_JDR1(ADC1),
					&ADC_JDR2(ADC1),
					&ADC_JDR3(ADC1),
					&ADC_JDR4(ADC1)
				      };

#ifdef USE_AD1_1
// For example the below channel has now the address of ADC_JDR4 if all 4 channels are specified.
// For our 2 channel inputs example adc1_injected_channels[0] = &ADC_JDR2(ADC1) and
// adc1_injected_channels[1] = &ADC_JDR1(ADC1)
  adc1_injected_channels[ADC1_C1] = tmp_channels_1[NB_ADC1_CHANNELS-1-ADC1_C1];
#endif
#ifdef USE_AD1_2
  adc1_injected_channels[ADC1_C2] = tmp_channels_1[NB_ADC1_CHANNELS-1-ADC1_C2];
#endif
#ifdef USE_AD1_3
  adc1_injected_channels[ADC1_C3] = tmp_channels_1[NB_ADC1_CHANNELS-1-ADC1_C3];
#endif
#ifdef USE_AD1_4
  adc1_injected_channels[ADC1_C4] = tmp_channels_1[NB_ADC1_CHANNELS-1-ADC1_C4];
#endif

#endif // USE_AD1

#if defined(USE_AD2) && defined(STM32F4)
  for(channel = 0; channel < NB_ADC2_CHANNELS; channel++){  adc2_buffers[channel] = NULL; }
  volatile uint32_t* tmp_channels_2[]={
					&ADC_JDR1(ADC2),
					&ADC_JDR2(ADC2),
					&ADC_JDR3(ADC2),
					&ADC_JDR4(ADC2)
				      };
#ifdef USE_AD2_1
  adc2_injected_channels[ADC2_C1] = tmp_channels_2[NB_ADC2_CHANNELS-1-ADC2_C1];
#endif
#ifdef USE_AD2_2
  adc2_injected_channels[ADC2_C2] = tmp_channels_2[NB_ADC2_CHANNELS-1-ADC2_C2];
#endif
#ifdef USE_AD2_3
  adc2_injected_channels[ADC2_C3] = tmp_channels_2[NB_ADC2_CHANNELS-1-ADC2_C3];
#endif
#ifdef USE_AD2_4
  adc2_injected_channels[ADC2_C4] = tmp_channels_2[NB_ADC2_CHANNELS-1-ADC2_C4];
#endif

#endif // USE_AD2

#if defined(USE_AD3) && defined(STM32F4)
  for(channel = 0; channel < NB_ADC3_CHANNELS; channel++){  adc3_buffers[channel] = NULL; }
  volatile uint32_t* tmp_channels_3[]={
					&ADC_JDR1(ADC3),
					&ADC_JDR2(ADC3),
					&ADC_JDR3(ADC3),
					&ADC_JDR4(ADC3)
				      };
#ifdef USE_AD3_1
  adc3_injected_channels[ADC3_C1] = tmp_channels_3[NB_ADC3_CHANNELS-1-ADC3_C1];
#endif
#ifdef USE_AD3_2
  adc3_injected_channels[ADC3_C2] = tmp_channels_3[NB_ADC3_CHANNELS-1-ADC3_C2];
#endif
#ifdef USE_AD3_3
  adc3_injected_channels[ADC3_C3] = tmp_channels_3[NB_ADC3_CHANNELS-1-ADC3_C3];
#endif
#ifdef USE_AD3_4
  adc3_injected_channels[ADC3_C4] = tmp_channels_3[NB_ADC3_CHANNELS-1-ADC3_C4];
#endif

#endif // USE_AD3

  adc_init_rcc();
  adc_init_irq();

#ifdef USE_AD1
  adc_new_data_trigger = FALSE;
  // adc_init_single(ADCx, c1, c2, c3, c4)
#if defined(BOARD_ADC1_CHANNEL_1)
  adc_channel_map[0] = BOARD_ADC1_CHANNEL_1;
#elif defined(USE_AD1_1)
  adc_channel_map[0] = USE_AD1_1;
#else
  adc_channel_map[0] = 0xFF;
#endif
#if defined(BOARD_ADC1_CHANNEL_2)
  adc_channel_map[1] = BOARD_ADC1_CHANNEL_2;
#elif defined(USE_AD1_2)
  adc_channel_map[1] = USE_AD1_2;
#else
  adc_channel_map[1] = 0xFF;
#endif
#if defined(BOARD_ADC1_CHANNEL_3)
  adc_channel_map[2] = BOARD_ADC1_CHANNEL_3;
#elif defined(USE_AD1_3)
  adc_channel_map[2] = USE_AD1_3;
#else
  adc_channel_map[2] = 0xFF;
#endif
#if defined(BOARD_ADC1_CHANNEL_4)
  adc_channel_map[3] = BOARD_ADC1_CHANNEL_4;
#elif defined(USE_AD1_4)
  adc_channel_map[3] = USE_AD1_4;
#else
  adc_channel_map[3] = 0xFF;
#endif

  adc_init_single(ADC1);
#endif // USE_AD1

#if defined(USE_AD2) && defined(STM32F4)
  adc_new_data_trigger = FALSE;
#if defined(BOARD_ADC2_CHANNEL_1)
  adc_channel_map[0] = BOARD_ADC2_CHANNEL_1;
#elif defined(USE_AD2_1)
  adc_channel_map[0] = USE_AD2_1;
#else
  adc_channel_map[0] = 0xFF;
#endif
#if defined(BOARD_ADC2_CHANNEL_2)
  adc_channel_map[1] = BOARD_ADC2_CHANNEL_2;
#elif defined(USE_AD2_2)
  adc_channel_map[1] = USE_AD2_2;
#else
  adc_channel_map[1] = 0xFF;
#endif
#if defined(BOARD_ADC2_CHANNEL_3)
  adc_channel_map[2] = BOARD_ADC2_CHANNEL_3;
#elif defined(USE_AD2_3)
  adc_channel_map[2] = USE_AD2_3;
#else
  adc_channel_map[2] = 0xFF;
#endif
#if defined(BOARD_ADC2_CHANNEL_4)
  adc_channel_map[3] = BOARD_ADC2_CHANNEL_4;
#elif defined(USE_AD2_4)
  adc_channel_map[3] = USE_AD2_4;
#else
  adc_channel_map[3] = 0xFF;
#endif

  adc_init_single(ADC2);
#endif // USE_AD2

#if defined(USE_AD3) && defined(STM32F4)
  adc_new_data_trigger = FALSE;
#if defined(BOARD_ADC3_CHANNEL_1)
  adc_channel_map[0] = BOARD_ADC3_CHANNEL_1;
#elif defined(USE_AD3_1)
  adc_channel_map[0] = USE_AD3_1;
#else
  adc_channel_map[0] = 0xFF;
#endif
#if defined(BOARD_ADC3_CHANNEL_2)
  adc_channel_map[1] = BOARD_ADC3_CHANNEL_2;
#elif defined(USE_AD3_2)
  adc_channel_map[1] = USE_AD3_2;
#else
  adc_channel_map[1] = 0xFF;
#endif
#if defined(BOARD_ADC3_CHANNEL_3)
  adc_channel_map[2] = BOARD_ADC3_CHANNEL_3;
#elif defined(USE_AD3_3)
  adc_channel_map[2] = USE_AD3_3;
#else
  adc_channel_map[2] = 0xFF;
#endif
#if defined(BOARD_ADC3_CHANNEL_4)
  adc_channel_map[3] = BOARD_ADC3_CHANNEL_4;
#elif defined(USE_AD3_4)
  adc_channel_map[3] = USE_AD3_4;
#else
  adc_channel_map[3] = 0xFF;
#endif

  adc_init_single(ADC3);
#endif // USE_AD3

}

static inline void adc_push_sample(struct adc_buf * buf, uint16_t value) {
  uint8_t new_head = buf->head + 1;

  if (new_head >= buf->av_nb_sample) {
    new_head = 0;
  }
  buf->sum -= buf->values[new_head];
  buf->values[new_head] = value;
  buf->sum += value;
  buf->head = new_head;
}

/******************************  ADC1+2+3 interrupt hander   *******************************/
 
#if defined(STM32F1)
void adc1_2_isr(void)
#elif defined(STM32F4)
void adc_isr(void)
#endif
{
  uint8_t channel = 0;
  uint16_t value  = 0;
  struct adc_buf * buf;

#ifdef USE_AD1
#pragma message "ADC1 active"
  // Clear Injected End Of Conversion
if (ADC_SR(ADC1) & ADC_SR_JEOC){ 
   ADC_SR(ADC1) &= ~ADC_SR_JEOC; 
   for (channel = 0; channel < NB_ADC1_CHANNELS; channel++) {
       buf = adc1_buffers[channel];
       if (buf) {
          value = *adc1_injected_channels[channel];
          adc_push_sample(buf, value);
       }
  }
#if !defined(USE_AD2) && !defined(USE_AD3)
  adc_new_data_trigger = 1;
#endif
}
#endif
#if defined(USE_AD2) && defined(STM32F4)
#pragma message "ADC2 active"
if (ADC_SR(ADC2) & ADC_SR_JEOC){ 
   ADC_SR(ADC2) &= ~ADC_SR_JEOC; 
   for (channel = 0; channel < NB_ADC2_CHANNELS; channel++) {
       buf = adc2_buffers[channel];
       if (buf) {
          value = *adc2_injected_channels[channel];
          adc_push_sample(buf, value);
       }
  }
#if !defined(USE_AD3) 
  adc_new_data_trigger = 1;
#endif
}
#endif
#if defined(USE_AD3) && defined(STM32F4)
#pragma message "ADC3 active"
if (ADC_SR(ADC3) & ADC_SR_JEOC){ 
   ADC_SR(ADC3) &= ~ADC_SR_JEOC; 
   for (channel = 0; channel < NB_ADC3_CHANNELS; channel++) {
       buf = adc3_buffers[channel];
       if (buf) {
          value = *adc3_injected_channels[channel];
          adc_push_sample(buf, value);
       }
  }
  adc_new_data_trigger = 1;
}
#endif
}
