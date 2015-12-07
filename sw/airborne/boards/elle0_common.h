/*
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
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

#ifndef CONFIG_ELLE0_COMMON_H
#define CONFIG_ELLE0_COMMON_H

#define BOARD_ELLE0

/* ELLE0 has a 25MHz external clock and 168MHz internal. */
#define EXT_CLK 25000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* orange, on PC2 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * not actual LEDS, used as GPIOs
 */

/* PB1, DRDY on EXT SPI connector*/
#define LED_BODY_GPIO GPIOB
#define LED_BODY_GPIO_PIN GPIO1
#define LED_BODY_GPIO_ON gpio_set
#define LED_BODY_GPIO_OFF gpio_clear
#define LED_BODY_AFIO_REMAP ((void)0)

/* PC12, on GPIO connector*/
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_PIN GPIO12
#define LED_12_GPIO_ON gpio_clear
#define LED_12_GPIO_OFF gpio_set
#define LED_12_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO0
#define SPEKTRUM_BIND_PIN_PORT GPIOB

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF GPIO_AF7
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

#define SPEKTRUM_UART2_RCC RCC_USART2
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2

#define SPEKTRUM_UART5_RCC RCC_UART5
#define SPEKTRUM_UART5_BANK GPIOD
#define SPEKTRUM_UART5_PIN GPIO2
#define SPEKTRUM_UART5_AF GPIO_AF8
#define SPEKTRUM_UART5_IRQ NVIC_UART5_IRQ
#define SPEKTRUM_UART5_ISR uart5_isr
#define SPEKTRUM_UART5_DEV UART5

/* PPM
 *
 * Default is PPM config 2, input on GPIOA1 (Servo pin 6)
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 2
#endif

#if PPM_CONFIG == 1
/* input on PA10 (UART1_RX) */
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC3IE
#define PPM_CC_IF           TIM_SR_CC3IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         GPIO_AF1

#elif PPM_CONFIG == 2
/* input on PA01 (Servo 6 pin) */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         GPIO_AF1

// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif
#define USE_AD_TIM1 1

#else
#error "Unknown PPM config"

#endif // PPM_CONFIG

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13

#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12

#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO4

#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12


/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * ADC
 */

/* Onboard ADCs */
/*
   ADC1 PC3/ADC13
   ADC2 PC0/ADC10
   ADC3 PC1/ADC11
   ADC4 PC5/ADC15
   ADC6 PC2/ADC12
   BATT PC4/ADC14
*/

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#if USE_ADC_1
#define AD1_1_CHANNEL 13
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO3
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL 10
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO0
#endif

#if USE_ADC_3
#define AD1_3_CHANNEL 11
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO1
#endif

#if USE_ADC_4
#define AD2_1_CHANNEL 15
#define ADC_4 AD2_1
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO5
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_5
#define USE_ADC_5 1
#endif
#if USE_ADC_5
#define AD1_4_CHANNEL 14
#define ADC_5 AD1_4
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO4
#endif

#if USE_ADC_6
#define AD2_2_CHANNEL 12
#define ADC_6 AD2_2
#define ADC_6_GPIO_PORT GPIOC
#define ADC_6_GPIO_PIN GPIO2
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_5
#endif

#define DefaultVoltageOfAdc(adc) (0.0045*adc)


/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM5 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

#if USE_SERVOS_7AND8
#if USE_I2C1
#error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
#else
#define ACTUATORS_PWM_NB 8
#define USE_PWM7 1
#define USE_PWM8 1
#define PWM_USE_TIM4 1
#define ACTUATORS_PWM_NB 8
#endif
#else
#define ACTUATORS_PWM_NB 6
#endif

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

/* servos 1-4 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)
/* servos 7-8 on TIM4 if USE_SERVOS_7AND8 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT)


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_ELLE0_1_0_H */
