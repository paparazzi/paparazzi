/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

#ifndef CONFIG_NAVSTIK_1_0_H
#define CONFIG_NAVSTIK_1_0_H

/* Navstik has a 12MHz external clock and 168MHz internal. */
#define EXT_CLK 25000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO4
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO5
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear
#define LED_2_AFIO_REMAP ((void)0)


/*
 * not actual LEDS, used as GPIOs
 */
#define GPS_POWER_GPIO GPIOA,GPIO4
#define IMU_POWER_GPIO GPIOC,GPIO15

/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO6
#define SPEKTRUM_BIND_PIN_PORT GPIOC
#define SPEKTRUM_BIND_PIN_HIGH 1

#define SPEKTRUM_UART6_RCC RCC_USART6
#define SPEKTRUM_UART6_BANK GPIOC
#define SPEKTRUM_UART6_PIN GPIO7
#define SPEKTRUM_UART6_AF GPIO_AF8
#define SPEKTRUM_UART6_IRQ NVIC_USART6_IRQ
#define SPEKTRUM_UART6_ISR usart6_isr
#define SPEKTRUM_UART6_DEV USART6

/*
 * PPM
 */

/* input on PC6 (Spektrum Tx) */
#define USE_PPM_TIM8 8
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM3_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOC
#define PPM_GPIO_PIN        GPIO7
#define PPM_GPIO_AF         GPIO_AF8

/* SPI */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOC
#define SPI2_GPIO_MISO GPIO2
#define SPI2_GPIO_PORT_MOSI GPIOC
#define SPI2_GPIO_MOSI GPIO3
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO10

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO0

#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO5


/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9


/*
 * ADC
 */

/* Onboard ADCs */
/*
   BATT_volt     PC1/ADC123  (ADC123_IN11)
   BATT_current  PA1/ADC123  (ADC123_IN1)
*/

// Internal ADC for battery enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 11
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO1
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL 1
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO1
#endif

/* allow to define ADC_CHANNEL_VSUPPLY and ADC_CHANNEL_CURRENT in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif
#ifndef ADC_CHANNEL_CURRENT
#define ADC_CHANNEL_CURRENT ADC_2
#endif

#define DefaultVoltageOfAdc(adc) (0.00382*adc)
#define DefaultMilliAmpereOfAdc(adc) (0.42497*adc)


/*
 * PWM
 *
 */
#define PWM_USE_TIM1 1
#define PWM_USE_TIM2 2
#define PWM_USE_TIM3 3
#define PWM_USE_TIM8 8

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1


// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO5
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO10
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM8
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF GPIO_AF3
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM2
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO11
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM3
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO1
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC4
#define PWM_SERVO_5_OC_BIT (1<<3)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM3
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO0
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC3
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servo 2 on TIM1 */
#define PWM_TIM1_CHAN_MASK (PWM_SERVO_2_OC_BIT)
/* servo 4 on TIM2 */
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_4_OC_BIT)
/* servos 1,5,6 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)
/* servo 3 on TIM8 */
#define PWM_TIM8_CHAN_MASK (PWM_SERVO_3_OC_BIT)

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_NAVSTIK_1_0_H */
