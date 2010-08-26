/*
 * $Id$
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

#include "adc.h"
#include <stm32/rcc.h>
#include <stm32/misc.h>
#include <stm32/adc.h>
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/tim.h>
#include "led.h"
#include BOARD_CONFIG

void adc1_2_irq_handler(void);

uint8_t adc_new_data_trigger;

/* Static functions */

static inline void adc_init_single(ADC_TypeDef * adc_t, 
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
  GPIO mapping for ADC1 pins (PA.B, PB.1, PC.3, PC.5). 
	Can be changed by predefining ADC1_GPIO_INIT. 
*/
#ifdef USE_AD1
#ifndef ADC1_GPIO_INIT
#define ADC1_GPIO_INIT(gpio) { \
	(gpio).GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1; \
	(gpio).GPIO_Mode = GPIO_Mode_AIN; \
	GPIO_Init(GPIOB, (&gpio)); \
	(gpio).GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_5; \
	GPIO_Init(GPIOC, (&gpio)); \
}
#endif // ADC1_GPIO_INIT
#endif // USE_AD1

/* 
  GPIO mapping for ADC2 pins. 
	Can be changed by predefining ADC2_GPIO_INIT. 
*/
#ifdef USE_AD2
#ifndef ADC2_GPIO_INIT
#define ADC2_GPIO_INIT(gpio) { }
#endif // ADC2_GPIO_INIT
#endif // USE_AD2

// }}}

/* 
  Currently, the enums adc1_channels and adc2_channels only 
  serve to resolve the number of channels on each ADC. 
*/

// NB_ADCx_CHANNELS
// {{{
enum adc1_channels { 
#ifdef USE_AD1_1
	ADC1_C1,
#endif
#ifdef USE_AD1_2
	ADC1_C2,
#endif
#ifdef USE_AD1_3
	ADC1_C3,
#endif
#ifdef USE_AD1_4
	ADC1_C4,
#endif
	NB_ADC1_CHANNELS
};

enum adc2_channels { 
#ifdef USE_AD2_1
	ADC2_C1,
#endif
#ifdef USE_AD2_2
	ADC2_C2,
#endif
#ifdef USE_AD2_3
	ADC2_C3,
#endif
#ifdef USE_AD2_4
	ADC2_C4,
#endif
	NB_ADC2_CHANNELS
};

// }}}

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
static uint8_t adc_injected_channels[4]; 
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
	TIM_TypeDef * timer; 
	uint32_t rcc_apb; 
#ifdef USE_AD_TIM4
	timer   = TIM4;
	rcc_apb = RCC_APB1Periph_TIM4;
#else 
	timer = TIM2;
	rcc_apb = RCC_APB1Periph_TIM2;
#endif

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB1PeriphClockCmd(rcc_apb, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | 
			       RCC_APB2Periph_GPIOC, ENABLE);
#ifdef USE_AD1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
#endif
#ifdef USE_AD2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
#endif
	
	/* Time Base configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period        = 0xFF;          
	TIM_TimeBaseStructure.TIM_Prescaler     = 0x8; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(timer, TIM_TRGOSource_Update);
	TIM_Cmd(timer, ENABLE);

#endif // defined (USE_AD1) || defined (USE_AD2)
} // }}}

/* Configure and enable ADC interrupt */
static inline void adc_init_irq( void ) 
{ // {{{
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel                   = ADC1_2_IRQn; 
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority        = 0;
	nvic.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&nvic);
} // }}}

/* 
	Usage: 
	
		adc_init_single(ADC1, 1, 1, 0, 0);
	
	... would enable ADC1, enabling channels 1 and 2, 
	but not 3 and 4. 
*/
static inline void adc_init_single(ADC_TypeDef * adc_t, 
				   uint8_t chan1, uint8_t chan2, 
				   uint8_t chan3, uint8_t chan4) 
{ 
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;
	uint8_t num_channels, rank; 

	// Paranoia, must be down for 2+ ADC clock cycles before calibration
	ADC_Cmd(adc_t, DISABLE); 

	/* enable adc_t clock */
	if (adc_t == ADC1) { 
#ifdef USE_AD1
		num_channels = NB_ADC1_CHANNELS;
		ADC1_GPIO_INIT(gpio);
#endif
	}
	else if (adc_t == ADC2) { 
#ifdef USE_AD2
		num_channels = NB_ADC2_CHANNELS;
		ADC2_GPIO_INIT(gpio); 
#endif
	}

	/* Configure ADC */
	
	adc.ADC_Mode               = ADC_Mode_Independent; 
	adc.ADC_ScanConvMode       = ENABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign          = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel       = 0; // No. of channels in regular mode
	ADC_Init(adc_t, &adc);

	ADC_InjectedSequencerLengthConfig(adc_t, num_channels);

	rank = 1; 
	if (chan1) { 
		ADC_InjectedChannelConfig(adc_t, adc_channel_map[0], rank,
					  ADC_SampleTime_41Cycles5);
		rank++;
	}
	if (chan2) { 
		ADC_InjectedChannelConfig(adc_t, adc_channel_map[1], rank,
					  ADC_SampleTime_41Cycles5);
		rank++;
	}
	if (chan3) { 
		ADC_InjectedChannelConfig(adc_t, adc_channel_map[2], rank,
					  ADC_SampleTime_41Cycles5);
		rank++;
	}
	if (chan4) { 
		ADC_InjectedChannelConfig(adc_t, adc_channel_map[3], rank,
					  ADC_SampleTime_41Cycles5);
	}


	ADC_ExternalTrigInjectedConvCmd(adc_t, ENABLE);
#ifdef USE_AD_TIM4
	ADC_ExternalTrigInjectedConvConfig(adc_t, ADC_ExternalTrigInjecConv_T4_TRGO);
#else
	ADC_ExternalTrigInjectedConvConfig(adc_t, ADC_ExternalTrigInjecConv_T2_TRGO);
#endif

	/* Enable ADC<X> JEOC interrupt */
	ADC_ITConfig(adc_t, ADC_IT_JEOC, ENABLE);

	/* Enable ADC<X> */
	ADC_Cmd(adc_t, ENABLE);

	/* Enable ADC<X> reset calibaration register */
	ADC_ResetCalibration(adc_t);

	/* Check the end of ADC<X> reset calibration */
	while (ADC_GetResetCalibrationStatus(adc_t)) ;
	/* Start ADC<X> calibaration */
	ADC_StartCalibration(adc_t);
	/* Check the end of ADC<X> calibration */
	while (ADC_GetCalibrationStatus(adc_t)) ;

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
		adc1_buffers[channel] = 0; 
#endif
#ifdef USE_AD2
	for(channel = 0; channel < NB_ADC2_CHANNELS; channel++)
		adc2_buffers[channel] = 0; 
#endif
	
	adc_new_data_trigger = 0;
	adc_injected_channels[0] = ADC_InjectedChannel_1;
	adc_injected_channels[1] = ADC_InjectedChannel_2;
	adc_injected_channels[2] = ADC_InjectedChannel_3;
	adc_injected_channels[3] = ADC_InjectedChannel_4;
	// TODO: Channel selection should be configured 
	// using defines. 
	adc_channel_map[0] = ADC_Channel_8;
	adc_channel_map[1] = ADC_Channel_9;
	adc_channel_map[2] = ADC_Channel_13;
	adc_channel_map[3] = ADC_Channel_15;

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
void adc1_2_irq_handler(void)
{
	uint8_t channel = 0;
	uint16_t value = 0; 
	struct adc_buf * buf; 

	if(NB_ADC1_CHANNELS == 4) { LED_TOGGLE(3); }
	else { LED_OFF(3); }

#ifdef USE_AD1
	// Clear Injected End Of Conversion
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC); 
	for(channel = 0; channel < NB_ADC1_CHANNELS; channel++) { 
		buf = adc1_buffers[channel];
		if(buf) { 
			value = ADC_GetInjectedConversionValue(ADC1, adc_injected_channels[channel]);
			if(value == 0) { LED_ON(2); }
			adc_push_sample(buf, value);
		}
	}
	adc_new_data_trigger = 1;
#endif
#ifdef USE_AD2
#endif
}

