/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

#ifndef CONFIG_OPA_FTD_1_0_H
#define CONFIG_OPA_FTD_1_0_H

#define BOARD_OPA_FTD

/* OPA/FTD has a 12MHz external clock and 168MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 168000000

/*
 * MCU Power control
 */
#define MCU_PWR GPIOB
#define MCU_PWR_PIN GPIO5
#define MCU_PWR_ON gpio_set
#define MCU_PWR_OFF gpio_clear

/*
 * Buttons
 */

/* E-Stop button */
#define BTN_ESTOP GPIOB
#define BTN_ESTOP_PIN GPIO12

/*
 * Onboard LEDs
 */

/* Status (red), on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear
#define LED_1_AFIO_REMAP ((void)0)

/* RC (green), on PC5 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO5
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear
#define LED_2_AFIO_REMAP ((void)0)

/* Status LED ext, PB13 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOB
#define LED_3_GPIO_PIN GPIO13
#define LED_3_GPIO_ON gpio_set
#define LED_3_GPIO_OFF gpio_clear
#define LED_3_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOB
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOB
#define UART3_GPIO_TX GPIO10

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOC
#define UART4_GPIO_RX GPIO11
#define UART4_GPIO_PORT_TX GPIOC
#define UART4_GPIO_TX GPIO10

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2

/*
 * Spektrum
 */
/* Define power pin */
#define RADIO_CONTROL_POWER GPIOC
#define RADIO_CONTROL_POWER_PIN GPIO13
#define RADIO_CONTROL_POWER_ON gpio_set
#define RADIO_CONTROL_POWER_OFF gpio_clear

/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO2
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#define SPEKTRUM_UART5_RCC RCC_UART5
#define SPEKTRUM_UART5_BANK GPIOD
#define SPEKTRUM_UART5_PIN GPIO2
#define SPEKTRUM_UART5_AF GPIO_AF8
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO4


/*
 * ADC
 */

/* BATT PC4/ADC14 */
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 14
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif


// Default: 10k / 2k2 ->
// #define DefaultVoltageOfAdc(adc) (0.00446777*adc)

// Opa-FBW:  10k / 4k7
#define DefaultVoltageOfAdc(adc) (00.00252*adc)


/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM4 1
#define PWM_USE_TIM5 1

#define USE_PWM1  1
#define USE_PWM2  1
#define USE_PWM3  1
#define USE_PWM4  1
#define USE_PWM5  1
#define USE_PWM6  1
#define USE_PWM7  1
#define USE_PWM8  1
#define USE_PWM9  1
#define USE_PWM10 1
#define ACTUATORS_PWM_NB 10


// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO0
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM4
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO6
#define PWM_SERVO_7_AF GPIO_AF2
#define PWM_SERVO_7_OC TIM_OC1
#define PWM_SERVO_7_OC_BIT (1<<0)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM4
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO7
#define PWM_SERVO_8_AF GPIO_AF2
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 8
#define PWM_SERVO_9_TIMER TIM4
#define PWM_SERVO_9_GPIO GPIOB
#define PWM_SERVO_9_PIN GPIO8
#define PWM_SERVO_9_AF GPIO_AF2
#define PWM_SERVO_9_OC TIM_OC3
#define PWM_SERVO_9_OC_BIT (1<<2)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

#if USE_PWM10
#define PWM_SERVO_10 9
#define PWM_SERVO_10_TIMER TIM4
#define PWM_SERVO_10_GPIO GPIOB
#define PWM_SERVO_10_PIN GPIO9
#define PWM_SERVO_10_AF GPIO_AF2
#define PWM_SERVO_10_OC TIM_OC4
#define PWM_SERVO_10_OC_BIT (1<<3)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

/* servos 1-4  on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6  on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)
/* servos 7-10 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT|PWM_SERVO_9_OC_BIT|PWM_SERVO_10_OC_BIT)

#endif /* CONFIG_OPA_FTD_1_0_H */
