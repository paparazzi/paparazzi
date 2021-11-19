/*
 * Copyright (C) 2016 Felix Ruess <felix.ruess@gmail.com
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

#ifndef CONFIG_OPENPILOT_REVO_NANO_H
#define CONFIG_OPENPILOT_REVO_NANO_H

/* Some info about this board can be found at:
 * https://librepilot.atlassian.net/wiki/display/LPDOC/OpenPilot+Revolution+Nano
 */


/* OpenPilot Revo Nano has a STM32F411 with 8MHz external clock and up to 100MHz internal.
 * Libopencm3 doesn't explicitly support the STM32F41x yet,
 * but using the 84MHz clock setup from the STM32F401 should work...
 */
#define EXT_CLK 8000000
#define AHB_CLK 84000000

/*
 * Onboard LEDs
 */

/* blue, on PC14 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* orange, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* UART */
/* Flexi port, shared with I2C1 */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6

/* Main port */
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO0
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2

/* S.Bus inverter control on PC15 */
#define RC_POLARITY_GPIO_PORT GPIOC
#define RC_POLARITY_GPIO_PIN GPIO15


/* PPM
 *
 * PPM sum in is on FlexIO pin 4 (blue)
 */

/* input on PB1 */
#define USE_PPM_TIM3 1
#define PPM_CHANNEL         TIM_IC4
#define PPM_TIMER_INPUT     TIM_IC_IN_TI4
#define PPM_IRQ             NVIC_TIM3_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC4IE
#define PPM_CC_IF           TIM_SR_CC4IF
#define PPM_GPIO_PORT       GPIOB
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         GPIO_AF2


/* SPI */
/* MPU9250 */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12

/* MPU9250 select */
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO12



/* I2C mapping */
/* I2C1 on flexi port shared with UART1 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

/* I2C3 for baro and flash */
#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOB
#define I2C3_GPIO_SDA GPIO4
#define I2C3_GPIO_SDA_AF GPIO_AF9

/*
 * ADC
 */
/* not tested, channels and ADx_1 not correct yet */

#ifndef USE_ADC_1
#define USE_ADC_1 0
#endif

/* Pin 4 on FlexIO, blue */
#if USE_ADC_0
#define AD1_0_CHANNEL 12
#define ADC_0 AD1_1
#define ADC_0_GPIO_PORT GPIOB
#define ADC_0_GPIO_PIN GPIO1
#endif

/* Pin 5 on FlexIO, yellow */
#if USE_ADC_1
#define AD1_1_CHANNEL 12
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO0
#endif

/* pin 6 on FlexIO, green */
#if USE_ADC_2
#define AD1_2_CHANNEL 11
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO7
#endif

/* pin 7 on FlexIO, orange */
#if USE_ADC_3
#define AD1_3_CHANNEL 11
#define ADC_3 AD1_2
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO6
#endif

/* pin 8 on FlexIO, violet */
#if USE_ADC_4
#define AD1_4_CHANNEL 11
#define ADC_4 AD1_2
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO5
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
//#ifndef ADC_CHANNEL_VSUPPLY
//#define ADC_CHANNEL_VSUPPLY ADC_1
//#endif

/* no voltage divider on board, adjust VoltageOfAdc in airframe file */
#define DefaultVoltageOfAdc(adc) (0.0045*adc)


/*
 * PWM
 *
 */
#define PWM_USE_TIM1 1
#define PWM_USE_TIM2 1
#define PWM_USE_TIM4 1
#define PWM_USE_TIM5 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1


#define ACTUATORS_PWM_NB 6


// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO10
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM2
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO3
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM4
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
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


/* servo 1 on TIM1 */
#define PWM_TIM1_CHAN_MASK (PWM_SERVO_1_OC_BIT)
/* servo 2 on TIM2 */
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_2_OC_BIT)
/* servos 3-4 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_OPENPILOT_REVO_NANO_H */
