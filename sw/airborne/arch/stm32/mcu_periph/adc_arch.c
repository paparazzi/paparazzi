/*
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 *
 * This is the driver for the analog to digital converters
 * on STM32
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
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>
#include <string.h>
#include "std.h"
#include "led.h"
#include BOARD_CONFIG

void adc1_2_isr(void);

uint8_t adc_new_data_trigger;

/* Static functions */

static inline void adc_init_single(uint32_t adc,
                   uint8_t chan1, uint8_t chan2,
                   uint8_t chan3, uint8_t chan4);

static inline void adc_push_sample(struct adc_buf * buf,
                   uint16_t sample);

static inline void adc_init_rcc( void );
static inline void adc_init_irq( void );

#ifdef USE_AD2
#error NOT_IMPLEMENTED__currently_only_ADC1_is_supported
#endif

/*
  Only 4 ADC channels may be enabled at the same time
  on each ADC, as there are only 4 injection registers.
*/

// ADCx_GPIO_INIT
// {{{

/*
  GPIO mapping for ADC1 pins (PB.1, PB.0, PC.5, PC.3).
    Can be changed by predefining ADC1_GPIO_INIT.
*/
#ifdef USE_AD1
#ifndef ADC1_GPIO_INIT
#define ADC1_GPIO_INIT() {			\
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,	\
		GPIO_CNF_INPUT_ANALOG,		\
		GPIO1 |	GPIO0);			\
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,	\
		GPIO_CNF_INPUT_ANALOG,		\
		GPIO5 |	GPIO3);			\
}
#endif // ADC1_GPIO_INIT
#endif // USE_AD1

/*
  GPIO mapping for ADC2 pins.
    Can be changed by predefining ADC2_GPIO_INIT.
    Uses the same GPIOs as ADC1 (lisa specific).
*/
#ifdef USE_AD2
#ifndef ADC2_GPIO_INIT
#define ADC2_GPIO_INIT() {			\
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,	\
	    GPIO_CNF_INPUT_ANALOG,		\
	    GPIO1 | GPIO0);			\
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT,	\
	    GPIO_CNF_INPUT_ANALOG,		\
	    GPIO5 | GPIO3);			\
  }
#endif // ADC2_GPIO_INIT
#endif // USE_AD2

// }}}

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
static struct adc_buf * adc1_buffers[NB_ADC1_CHANNELS];
#endif
#ifdef USE_AD2
static struct adc_buf * adc2_buffers[NB_ADC2_CHANNELS];
#endif

/*
    Static mapping from channel index to channel injection
    index:
*/

/*
 Maps integer value x to ADC_InjectedChannel_x,
 so they can be iterated safely
*/
volatile uint32_t *adc_injected_channels[4];
/*
 Maps integer value x to ADC_Channel_y, like

 0 --> ADC_Channel_5
 1 --> ADC_Channel_8
 2 --> ADC_Channel_13

 so they can be iterated incrementally.
*/
static uint8_t adc_channel_map[4];

/*
  TODO: Extend interface to allow adressing a
  specific ADC (at least ADC1 and ADC2)?
*/
void adc_buf_channel(uint8_t adc_channel,
             struct adc_buf * s,
             uint8_t av_nb_sample)
{
  adc1_buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}

// #define USE_AD_TIM4
/* Configure and enable RCC for peripherals (ADC1, ADC2, Timer) */
static inline void adc_init_rcc( void )
{ // {{{
#if defined (USE_AD1) || defined (USE_AD2)
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

    /*
     * Historic Note:
     * Previously in libstm32 we were setting the ADC clock here.
     * It was being set to PCLK2 DIV2 resulting in 36MHz clock on the ADC. I am
     * pretty sure that this is wrong as based on the datasheet the ADC clock
     * must not exceed 14MHz! Now the clock is being set by the clock init
     * routine in libopencm3 so we don't have to set up this clock ourselves any
     * more. This comment is here just as a reminder and may be removed in the
     * future when we know that everything is working properly.
     * (by Esden the historian :D)
     */

    /* Timer peripheral clock enable. */
    rcc_peripheral_enable_clock(rcc_apbenr, rcc_apb);
    /* GPIO peripheral clock enable. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN |
			                      RCC_APB2ENR_IOPCEN);

    /* Enable ADC peripheral clocks. */
#ifdef USE_AD1
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
#endif
#ifdef USE_AD2
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
#endif

    /* Time Base configuration */
    timer_reset(timer);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
	    TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(timer, 0xFF);
    timer_set_prescaler(timer, 0x8);
    timer_set_clock_division(timer, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(timer);

#endif // defined (USE_AD1) || defined (USE_AD2)
} // }}}

/* Configure and enable ADC interrupt */
static inline void adc_init_irq( void )
{ // {{{
    nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
    nvic_enable_irq(NVIC_ADC1_2_IRQ);
} // }}}

/*
    Usage:

        adc_init_single(ADC1, 1, 1, 0, 0);

    ... would enable ADC1, enabling channels 1 and 2,
    but not 3 and 4.
*/
static inline void adc_init_single(uint32_t adc,
                   uint8_t chan1, uint8_t chan2,
                   uint8_t chan3, uint8_t chan4)
{
    uint8_t num_channels, rank;
    uint8_t channels[4];

    // Paranoia, must be down for 2+ ADC clock cycles before calibration
    adc_off(adc);

    /* enable adc clock */
    if (adc == ADC1) {
#ifdef USE_AD1
        num_channels = NB_ADC1_CHANNELS;
        ADC1_GPIO_INIT();
#endif
    }
    else if (adc == ADC2) {
#ifdef USE_AD2
        num_channels = NB_ADC2_CHANNELS;
        ADC2_GPIO_INIT();
#endif
    }

    /* Configure ADC */
    //adc_reset(adc);

    /* XXX: This register fiddeling code should be moved to libopencm3! */
    /* Set CR1 register. */
    {
	    uint32_t tmpreg = ADC_CR1(adc);

	    /* Clear DUALMOD and SCAN. */
	    tmpreg &= ~(ADC_CR1_DUALMOD_MASK | ADC_CR1_SCAN);

	    /* Set correct DUALMOD and SCAN. */
	    tmpreg |= ADC_CR1_DUALMOD_IND | ADC_CR1_SCAN;

	    ADC_CR1(adc) = tmpreg;
    }

    /* Set CR2 register. */
    {
	    uint32_t tmpreg = ADC_CR2(adc);

	    /* Clear CONT, ALIGN and EXTSEL. */
	    tmpreg &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_EXTSEL_MASK);

	    /* Set correct CONT, ALIGN and EXTSEL. */
	    tmpreg |= ADC_CR2_EXTSEL_SWSTART;

	    ADC_CR2(adc) = tmpreg;
    }

    /* Set SQR1 register. */
    {
	    uint32_t tmpreg = ADC_SQR1(adc);

	    /* Clear regular channel sequence length. */
	    tmpreg &= ~(0xF); //~(ADC_SQR1_L_MSK);

	    /* Set regular channel sequence length. */
	    tmpreg |= 0;

	    ADC_SQR1(adc) = tmpreg;
    }

    /* Set JSQR1 injected sequence length. */
    {
	    uint32_t tmpreg = ADC_JSQR(adc);

	    /* Clear injected channel sequence length. */
	    tmpreg &= ~(0x2 << 20); //~(ADC_JSQR_JL_MSK);

	    /* Set injected channel sequence length. */
	    tmpreg |= (num_channels << ADC_JSQR_JL_LSB);

	    ADC_JSQR(adc) = tmpreg;
    }

    rank = 0;
    if (chan1) {
	adc_set_conversion_time(adc, adc_channel_map[0], ADC_SMPR1_SMP_41DOT5CYC);
	channels[rank] = adc_channel_map[0];
        rank++;
    }
    if (chan2) {
	adc_set_conversion_time(adc, adc_channel_map[1], ADC_SMPR1_SMP_41DOT5CYC);
	channels[rank] = adc_channel_map[1];
        rank++;
    }
    if (chan3) {
	adc_set_conversion_time(adc, adc_channel_map[2], ADC_SMPR1_SMP_41DOT5CYC);
	channels[rank] = adc_channel_map[2];
        rank++;
    }
    if (chan4) {
	adc_set_conversion_time(adc, adc_channel_map[3], ADC_SMPR1_SMP_41DOT5CYC);
	channels[rank] = adc_channel_map[3];
    }

    adc_set_injected_sequence(adc, num_channels, channels);

#if defined(USE_AD_TIM4)
    adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM4_TRGO);
#elif defined(USE_AD_TIM1)
    adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM1_TRGO);
#else
    adc_enable_external_trigger_injected(adc, ADC_CR2_JEXTSEL_TIM2_TRGO);
#endif

    /* Enable ADC<X> JEOC interrupt */
    adc_enable_jeoc_interrupt(adc);

    /* Enable ADC<X> */
    adc_on(adc);

    /* Enable ADC<X> reset calibaration register */
    adc_reset_calibration(adc);

    /* Check the end of ADC<X> reset calibration */
    while ((ADC_CR2(adc) & ADC_CR2_RSTCAL) != 0);
    /* Start ADC<X> calibaration */
    adc_calibration(adc);
    /* Check the end of ADC<X> calibration */
    while ((ADC_CR2(adc) & ADC_CR2_CAL) != 0);

} // adc_init_single

void adc_init( void ) {

    /* initialize buffer pointers with 0 (not set).
       buffer null pointers will be ignored in interrupt
       handler, which is important as there are no
       buffers registered at the time the ADC trigger
       interrupt is enabled.
    */
    uint8_t channel;
#ifdef USE_AD1
    for(channel = 0; channel < NB_ADC1_CHANNELS; channel++)
        adc1_buffers[channel] = NULL;
    adc_injected_channels[0] = &ADC_JDR1(ADC1);
    adc_injected_channels[1] = &ADC_JDR2(ADC1);
    adc_injected_channels[2] = &ADC_JDR3(ADC1);
    adc_injected_channels[3] = &ADC_JDR4(ADC1);
#endif
#ifdef USE_AD2
    for(channel = 0; channel < NB_ADC2_CHANNELS; channel++)
        adc2_buffers[channel] = NULL;
    adc_injected_channels[0] = &ADC_JDR1(ADC2);
    adc_injected_channels[1] = &ADC_JDR2(ADC2);
    adc_injected_channels[2] = &ADC_JDR3(ADC2);
    adc_injected_channels[3] = &ADC_JDR4(ADC2);
#endif

    adc_new_data_trigger = FALSE;
    adc_channel_map[0] = BOARD_ADC_CHANNEL_1;
    adc_channel_map[1] = BOARD_ADC_CHANNEL_2;
    // FIXME for now we get battery voltage this way
    //    adc_channel_map[2] = BOARD_ADC_CHANNEL_3;
    adc_channel_map[2] = BOARD_ADC_CHANNEL_3;
    adc_channel_map[3] = BOARD_ADC_CHANNEL_4;

    adc_init_rcc();
    adc_init_irq();

// adc_init_single(ADCx, c1, c2, c3, c4)
// {{{
#ifdef USE_AD1
    adc_init_single(ADC1,
#ifdef USE_AD1_1
            1,
#else
            0,
#endif
#ifdef USE_AD1_2
            1,
#else
            0,
#endif
#ifdef USE_AD1_3
            1,
#else
            0,
#endif
#ifdef USE_AD1_4
            1
#else
            0
#endif
    );
#endif // USE_AD1

#ifdef USE_AD2
    adc_init_single(ADC2,
#ifdef USE_AD2_1
            1,
#else
            0,
#endif
#ifdef USE_AD2_2
            1,
#else
            0,
#endif
#ifdef USE_AD2_3
            1,
#else
            0,
#endif
#ifdef USE_AD2_4
            1
#else
            0
#endif
    );
#endif // USE_AD2

// }}}
}

static inline void adc_push_sample(struct adc_buf * buf, uint16_t value) {
    uint8_t new_head = buf->head + 1;

    if (new_head >= buf->av_nb_sample) { new_head = 0; }
    buf->sum -= buf->values[new_head];
    buf->values[new_head] = value;
    buf->sum += value;
    buf->head = new_head;
}

/**
 * ADC1+2 interrupt hander
 */
void adc1_2_isr(void)
{
    uint8_t channel = 0;
    uint16_t value  = 0;
    struct adc_buf * buf;

#ifdef USE_AD1
    // Clear Injected End Of Conversion
    ADC_SR(ADC1) &= ~ADC_SR_JEOC;
    for(channel = 0; channel < NB_ADC1_CHANNELS; channel++) {
        buf = adc1_buffers[channel];
        if(buf) {
            value = *adc_injected_channels[channel];
            adc_push_sample(buf, value);
        }
    }
    adc_new_data_trigger = 1;
#endif
#ifdef USE_AD2
    // Clear Injected End Of Conversion
    ADC_SR(ADC2) &= ~ADC_SR_JEOC;
    for(channel = 0; channel < NB_ADC2_CHANNELS; channel++) {
        buf = adc2_buffers[channel];
        if(buf) {
            value = *adc_injected_channels[channel];
            adc_push_sample(buf, value);
        }
    }
    adc_new_data_trigger = 1;
#endif
}
