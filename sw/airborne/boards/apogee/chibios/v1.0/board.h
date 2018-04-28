/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_APOGEE
#define BOARD_NAME  "AB/GRZ STM32F4 Apogee 1.0"

/*
 * Board oscillators-related settings.
 * NOTE: LSE fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                16000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F407xx
//#define STM32F40_41xxx
//#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX              0 // GPS TX
#define GPIOA_UART4_RX              1 // GPS RX
#define GPIOA_PWM2_CH3              2 // SERVO 1
#define GPIOA_UART2_RX              3 // SBUS RX
#define GPIOA_ADC1_IN4              4 // BAT ADC
#define GPIOA_SPI1_SCK              5 // SPI SCK
#define GPIOA_SPI1_MISO             6 // SPI MISO
#define GPIOA_SPI1_MOSI             7 // SPI MOSI
#define GPIOA_ICU1_CH1              8 // PPM_IN
#define GPIOA_OTG_FS_VBUS           9 // VBUS
#define GPIOA_USART1_RX             10 // MODEM RX
#define GPIOA_OTG_FS_DM             11 // USB
#define GPIOA_OTG_FS_DP             12 // USB
#define GPIOA_SWDIO                 13 // SERIAL WIRE DEBUG
#define GPIOA_SWCLK                 14 // SERIAL WIRE DEBUG
#define GPIOA_PWM2_CH1              15 // SERVO 5

#define GPIOB_PWM3_CH3              0 // SERVO 0
#define GPIOB_AUX1                  1 // AUX1 / SERVO6 (TIM3_CHANNEL_4 when pwm)
#define GPIOB_BOOT1                 2 //
#define GPIOB_PWM2_CH2              3 // SERVO 4
#define GPIOB_PWM3_CH1              4 // SERVO 3
#define GPIOB_PWM3_CH2              5 // SERVO 2
#define GPIOB_USART1_TX             6 // MODEM TX
#define GPIOB_I2C1_SDA              7 // I2C1 SDA
#define GPIOB_I2C1_SCL              8 // I2C1 SCL
#define GPIOB_SPI1_CS               9 // SPI CS
#define GPIOB_I2C2_SCL              10 // I2C2 SCL
#define GPIOB_I2C2_SDA              11 // I2C2 SDA
#define GPIOB_POWER_SWITCH          12 // POWER SWITCH
#define GPIOB_RX2_POL               13 // UART2 POLARITY
#define GPIOB_SDIO_DETECT           14 // SDIO DETECT
#define GPIOB_AUX4                  15 // AUX4, only GPIO capable

#define GPIOC_LED1                  0 // LED 1
#define GPIOC_LED3                  1 // LED 3
#define GPIOC_PIN2                  2 // NOT YET USED, ADC Capable
#define GPIOC_LED4                  3 // LED 4
#define GPIOC_AUX3                  4 // AUX3 ADC Capable
#define GPIOC_AUX2                  5 // AUX2 ADC Capable
#define GPIOC_USART6_TX             6 // UART6 TX
#define GPIOC_USART6_RX             7 // UART6 RX
#define GPIOC_SDIO_D0               8 // SDIO
#define GPIOC_SDIO_D1               9 // SDIO
#define GPIOC_SDIO_D2               10 // SDIO
#define GPIOC_SDIO_D3               11 // SDIO
#define GPIOC_SDIO_CK               12 // SDIO
#define GPIOC_LED2                  13 // LED 2
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_SDIO_CMD              2 // SDIO

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_UART4_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_UART4_RX) |     \
                                     PIN_MODE_INPUT(GPIOA_PWM2_CH3) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_RX) |     \
                                     PIN_MODE_ANALOG(GPIOA_ADC1_IN4) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_ICU1_CH1) |     \
                                     PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |        \
                                     PIN_MODE_INPUT(GPIOA_PWM2_CH1))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM2_CH3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_IN4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ICU1_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM2_CH1))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_UART4_TX) |     \
                                     PIN_OSPEED_100M(GPIOA_UART4_RX) |     \
                                     PIN_OSPEED_100M(GPIOA_PWM2_CH3) |     \
                                     PIN_OSPEED_100M(GPIOA_UART2_RX) |     \
                                     PIN_OSPEED_100M(GPIOA_ADC1_IN4) |     \
                                     PIN_OSPEED_100M(GPIOA_SPI1_SCK) |     \
                                     PIN_OSPEED_100M(GPIOA_SPI1_MISO) |    \
                                     PIN_OSPEED_100M(GPIOA_SPI1_MOSI) |    \
                                     PIN_OSPEED_100M(GPIOA_ICU1_CH1) |     \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OSPEED_100M(GPIOA_USART1_RX) |    \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |    \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |    \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) |        \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) |        \
                                     PIN_OSPEED_100M(GPIOA_PWM2_CH1))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_UART4_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_UART4_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_PWM2_CH3) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_UART2_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_IN4) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_ICU1_CH1) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_RX) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |          \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_PWM2_CH1))

#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_UART4_TX) |     \
                                     PIN_ODR_HIGH(GPIOA_UART4_RX) |     \
                                     PIN_ODR_HIGH(GPIOA_PWM2_CH3) |     \
                                     PIN_ODR_HIGH(GPIOA_UART2_RX) |     \
                                     PIN_ODR_HIGH(GPIOA_ADC1_IN4) |     \
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK) |     \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) |    \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |    \
                                     PIN_ODR_HIGH(GPIOA_ICU1_CH1) |     \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_ODR_HIGH(GPIOA_USART1_RX) |    \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |    \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |    \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |        \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |        \
                                     PIN_ODR_HIGH(GPIOA_PWM2_CH1))

#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART4_TX, 8) |   \
                                     PIN_AFIO_AF(GPIOA_UART4_RX, 8) |   \
                                     PIN_AFIO_AF(GPIOA_PWM2_CH3, 0) |   \
                                     PIN_AFIO_AF(GPIOA_UART2_RX, 7) |   \
                                     PIN_AFIO_AF(GPIOA_ADC1_IN4, 0) |   \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5) |   \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5) |  \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5))

#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_ICU1_CH1, 1) |     \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_VBUS, 0) |  \
                                     PIN_AFIO_AF(GPIOA_USART1_RX, 7) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |   \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |   \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |        \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |        \
                                     PIN_AFIO_AF(GPIOA_PWM2_CH1, 0))

/*
 * GPIOB setup:
 *
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PWM3_CH3)        | \
                                     PIN_MODE_INPUT(GPIOB_AUX1)            | \
                                     PIN_MODE_INPUT(GPIOB_BOOT1)           | \
                                     PIN_MODE_INPUT(GPIOB_PWM2_CH2)        | \
                                     PIN_MODE_INPUT(GPIOB_PWM3_CH1)        | \
                                     PIN_MODE_INPUT(GPIOB_PWM3_CH2)        | \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_TX)   | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL)    | \
                                     PIN_MODE_OUTPUT(GPIOB_SPI1_CS)        | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SCL)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA)    | \
                                     PIN_MODE_OUTPUT(GPIOB_POWER_SWITCH)   | \
                                     PIN_MODE_OUTPUT(GPIOB_RX2_POL)        | \
                                     PIN_MODE_INPUT(GPIOB_SDIO_DETECT)     | \
                                     PIN_MODE_INPUT(GPIOB_AUX4))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH3)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AUX1)        | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_BOOT1)      | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM2_CH2)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH1)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH2)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART1_TX)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_CS)     | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_POWER_SWITCH)| \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RX2_POL)     | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO_DETECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AUX4))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PWM3_CH3)      | \
                                     PIN_OSPEED_100M(GPIOB_AUX1)          | \
                                     PIN_OSPEED_100M(GPIOB_BOOT1)         | \
                                     PIN_OSPEED_100M(GPIOB_PWM2_CH2)      | \
                                     PIN_OSPEED_100M(GPIOB_PWM3_CH1)      | \
                                     PIN_OSPEED_100M(GPIOB_PWM3_CH2)      | \
                                     PIN_OSPEED_100M(GPIOB_USART1_TX)     | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SDA)      | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SCL)      | \
                                     PIN_OSPEED_100M(GPIOB_SPI1_CS)       | \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SCL)      | \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SDA)      | \
                                     PIN_OSPEED_100M(GPIOB_POWER_SWITCH)  | \
                                     PIN_OSPEED_100M(GPIOB_RX2_POL)       | \
                                     PIN_OSPEED_100M(GPIOB_SDIO_DETECT)   | \
                                     PIN_OSPEED_100M(GPIOB_AUX4))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PWM3_CH3)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_AUX1)       | \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1)      | \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM2_CH2)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM3_CH1)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM3_CH2)   | \
                                     PIN_PUPDR_FLOATING(GPIOB_USART1_TX)  | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SDA)     | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SCL)     | \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI1_CS)    | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SCL)     | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SDA)     | \
                                     PIN_PUPDR_PULLUP(GPIOB_POWER_SWITCH) | \
                                     PIN_PUPDR_PULLUP(GPIOB_RX2_POL)      | \
                                     PIN_PUPDR_PULLUP(GPIOB_SDIO_DETECT)  | \
                                     PIN_PUPDR_FLOATING(GPIOB_AUX4))

#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PWM3_CH3)         | \
                                     PIN_ODR_LOW(GPIOB_AUX1)              | \
                                     PIN_ODR_LOW(GPIOB_BOOT1)             | \
                                     PIN_ODR_HIGH(GPIOB_PWM2_CH2)         | \
                                     PIN_ODR_HIGH(GPIOB_PWM3_CH1)         | \
                                     PIN_ODR_HIGH(GPIOB_PWM3_CH2)         | \
                                     PIN_ODR_HIGH(GPIOB_USART1_TX)        | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA)         | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL)         | \
                                     PIN_ODR_HIGH(GPIOB_SPI1_CS)          | \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SCL)         | \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA)         | \
                                     PIN_ODR_HIGH(GPIOB_POWER_SWITCH)     | \
                                     PIN_ODR_HIGH(GPIOB_RX2_POL)          | \
                                     PIN_ODR_HIGH(GPIOB_SDIO_DETECT)      | \
                                     PIN_ODR_HIGH(GPIOB_AUX4))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PWM3_CH3, 0)       | \
                                     PIN_AFIO_AF(GPIOB_AUX1, 0)           | \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0)          | \
                                     PIN_AFIO_AF(GPIOB_PWM2_CH2, 0)       | \
                                     PIN_AFIO_AF(GPIOB_PWM3_CH1, 0)       | \
                                     PIN_AFIO_AF(GPIOB_PWM3_CH2, 0)       | \
                                     PIN_AFIO_AF(GPIOB_USART1_TX, 7)      | \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 4)       | \
                                     PIN_AFIO_AF(GPIOB_SPI1_CS, 0)        | \
                                     PIN_AFIO_AF(GPIOB_I2C2_SCL, 4)       | \
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA, 4)       | \
                                     PIN_AFIO_AF(GPIOB_POWER_SWITCH, 0)   | \
                                     PIN_AFIO_AF(GPIOB_RX2_POL, 0)        | \
                                     PIN_AFIO_AF(GPIOB_SDIO_DETECT, 0)    | \
                                     PIN_AFIO_AF(GPIOB_AUX4, 0))

/*
 * GPIOC setup:
 *
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED1)          | \
                                     PIN_MODE_OUTPUT(GPIOC_LED3)          | \
                                     PIN_MODE_OUTPUT(GPIOC_PIN2)          | \
                                     PIN_MODE_OUTPUT(GPIOC_LED4)          | \
                                     PIN_MODE_INPUT(GPIOC_AUX3)           | \
                                     PIN_MODE_INPUT(GPIOC_AUX2)           | \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_TX)  | \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_RX)  | \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0)    | \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1)    | \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D2)    | \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3)    | \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CK)    | \
                                     PIN_MODE_OUTPUT(GPIOC_LED2)          | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN)       | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_LED1)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED3)       | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN2)      | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED4)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX3)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX2)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_TX)  | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_RX)  | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CK)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED2)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_LED1)          | \
                                     PIN_OSPEED_100M(GPIOC_LED3)          | \
                                     PIN_OSPEED_100M(GPIOC_PIN2)          | \
                                     PIN_OSPEED_100M(GPIOC_LED4)          | \
                                     PIN_OSPEED_100M(GPIOC_AUX3)          | \
                                     PIN_OSPEED_100M(GPIOC_AUX2)          | \
                                     PIN_OSPEED_100M(GPIOC_USART6_TX)     | \
                                     PIN_OSPEED_100M(GPIOC_USART6_RX)     | \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D0)       | \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D1)       | \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D2)       | \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D3)       | \
                                     PIN_OSPEED_100M(GPIOC_SDIO_CK)       | \
                                     PIN_OSPEED_100M(GPIOC_LED2)          | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN)      | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_LED1)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_LED3)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_LED4)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_AUX3)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_AUX2)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_TX)  | \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_RX)  | \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0)      | \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1)      | \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2)      | \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3)      | \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO_CK)    | \
                                     PIN_PUPDR_FLOATING(GPIOC_LED2)       | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN)   | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))

#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_LED1)             | \
                                     PIN_ODR_HIGH(GPIOC_LED3)             | \
                                     PIN_ODR_HIGH(GPIOC_PIN2)             | \
                                     PIN_ODR_HIGH(GPIOC_LED4)             | \
                                     PIN_ODR_LOW(GPIOC_AUX3)              | \
                                     PIN_ODR_LOW(GPIOC_AUX2)              | \
                                     PIN_ODR_HIGH(GPIOC_USART6_TX)        | \
                                     PIN_ODR_HIGH(GPIOC_USART6_RX)        | \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0)          | \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1)          | \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2)          | \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3)          | \
                                     PIN_ODR_HIGH(GPIOC_SDIO_CK)          | \
                                     PIN_ODR_HIGH(GPIOC_LED2)             | \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN)         | \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))

#define VAL_GPIOC_AFRL             (PIN_AFIO_AF(GPIOC_LED1, 0)            | \
                                    PIN_AFIO_AF(GPIOC_LED3, 0)            | \
                                    PIN_AFIO_AF(GPIOC_PIN2, 0)            | \
                                    PIN_AFIO_AF(GPIOC_LED4, 0)            | \
                                    PIN_AFIO_AF(GPIOC_AUX3, 0)            | \
                                    PIN_AFIO_AF(GPIOC_AUX2, 0)            | \
                                    PIN_AFIO_AF(GPIOC_USART6_TX, 8)       | \
                                    PIN_AFIO_AF(GPIOC_USART6_RX, 8))

#define VAL_GPIOC_AFRH             (PIN_AFIO_AF(GPIOC_SDIO_D0, 12)        | \
                                    PIN_AFIO_AF(GPIOC_SDIO_D1, 12)        | \
                                    PIN_AFIO_AF(GPIOC_SDIO_D2, 12)        | \
                                    PIN_AFIO_AF(GPIOC_SDIO_D3, 12)        | \
                                    PIN_AFIO_AF(GPIOC_SDIO_CK, 12)        | \
                                    PIN_AFIO_AF(GPIOC_LED2, 0)            | \
                                    PIN_AFIO_AF(GPIOC_OSC32_IN, 0)        | \
                                    PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
 * PD2  - SDIO CMD                  (alternate 12).
 */
#define VAL_GPIOD_MODER             PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD)
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOD_PUPDR             0x55555555 // all pullup, GPIOD_SDIO_CMD should be !
#define VAL_GPIOD_ODR               0xFFFFFFFF
#define VAL_GPIOD_AFRL              PIN_AFIO_AF(GPIOD_SDIO_CMD, 12)
#define VAL_GPIOD_AFRH              0x00000000

/*
 * GPIOE setup:
 *
*/

#define VAL_GPIOE_MODER             0x00000000
#define VAL_GPIOE_OTYPER            0x00000000
#define VAL_GPIOE_OSPEEDR           0x00000000
#define VAL_GPIOE_PUPDR             0x55555555 // all pullup
#define VAL_GPIOE_ODR               0xFFFFFFFF
#define VAL_GPIOE_AFRL              0x00000000
#define VAL_GPIOE_AFRH              0x00000000

/*
 * GPIOF setup:
 *
*/
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0x00000000
#define VAL_GPIOF_PUPDR             0x55555555 // all pullup
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * GPIOG setup:
 *
*/
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0x00000000
#define VAL_GPIOG_PUPDR             0x55555555 // all pullup
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_100M(GPIOH_PIN2) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN3) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN4) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN5) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN6) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN7) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN8) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN9) |          \
                                     PIN_OSPEED_100M(GPIOH_PIN10) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN11) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN12) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN13) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN14) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) |        \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))

/*
 * GPIOI setup:
 *
*/
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0x00000000
#define VAL_GPIOI_PUPDR             0x55555555 // all pullup
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
/* red, on PC0 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* orange, on PC13 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO13
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/* green, on PC1 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO1
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* yellow, on PC3 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

/* AUX1, on PB1, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOB
#define LED_5_GPIO_PIN GPIO1
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* AUX2, on PC5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOC
#define LED_6_GPIO_PIN GPIO5
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* AUX3, on PC4, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOC
#define LED_7_GPIO_PIN GPIO4
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* AUX4, on PB15, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOB
#define LED_8_GPIO_PIN GPIO15
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear


/* Pint to set Uart2 RX polarity, on PB13, output high inverts, low doesn't */
#define RC_POLARITY_GPIO_PORT GPIOB
#define RC_POLARITY_GPIO_PIN GPIO13

/*
 * ADCs
 */
// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN9
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO1
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN15
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO5
#endif

// AUX 3
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN14
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO4
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN4
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define DefaultVoltageOfAdc(adc) (0.006185*adc)

/*
 * PWM defines
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOB
#define PWM_SERVO_0_PIN GPIO0
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_DRIVER PWMD3
#define PWM_SERVO_0_CHANNEL 2
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO2
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_DRIVER PWMD2
#define PWM_SERVO_1_CHANNEL 2
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO5
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_DRIVER PWMD3
#define PWM_SERVO_2_CHANNEL 1
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO4
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD3
#define PWM_SERVO_3_CHANNEL 0
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO3
#define PWM_SERVO_4_AF GPIO_AF1
#define PWM_SERVO_4_DRIVER PWMD2
#define PWM_SERVO_4_CHANNEL 1
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO15
#define PWM_SERVO_5_AF GPIO_AF1
#define PWM_SERVO_5_DRIVER PWMD2
#define PWM_SERVO_5_CHANNEL 0
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif

#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_DRIVER PWMD3
#define PWM_SERVO_6_CHANNEL 3
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_DISABLED
#endif


#ifdef STM32_PWM_USE_TIM2
#define PWM_CONF_TIM2 STM32_PWM_USE_TIM2
#else
#define PWM_CONF_TIM2 1
#endif
#define PWM_CONF2_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM2_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_5_ACTIVE, NULL }, \
    { PWM_SERVO_4_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM3
#define PWM_CONF_TIM3 STM32_PWM_USE_TIM3
#else
#define PWM_CONF_TIM3 1
#endif
#define PWM_CONF3_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM3_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_0_ACTIVE, NULL }, \
    { PWM_SERVO_6_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 2
#define PPM_TIMER_FREQUENCY 2000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD1

/*
 * Spektrum
 */

// shorter wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/* The line that is pulled low at power up to initiate the bind process
 * PB15: AUX4
 */
#define SPEKTRUM_BIND_PIN GPIO15
#define SPEKTRUM_BIND_PIN_PORT GPIOB

/* The line used to send the pulse train for the bind process
 * When using UART2 on Apogee, this as to be a different pin than the uart2 rx
 * Default pin for this is PA8: PPM_IN
 */
#ifndef SPEKTRUM_PRIMARY_BIND_CONF_PORT
#define SPEKTRUM_PRIMARY_BIND_CONF_PORT GPIOA
#define SPEKTRUM_PRIMARY_BIND_CONF_PIN GPIO8
#endif

/*
 * PWM input
 */
// PWM_INPUT 1 on PA8 (also PPM IN)
#define PWM_INPUT1_ICU            ICUD1
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
// PPM in (aka PA8) is used: not compatible with PPM RC receiver
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO8
#define PWM_INPUT1_GPIO_AF        GPIO_AF1

// PWM_INPUT 2 on PA3 (also SERVO 1)
#if (USE_PWM1 && USE_PWM_INPUT2)
#error "PW1 and PWM_INPUT2 are not compatible"
#endif
#define PWM_INPUT2_ICU            ICUD9
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT2_GPIO_PORT      GPIOA
#define PWM_INPUT2_GPIO_PIN       GPIO2
#define PWM_INPUT2_GPIO_AF        GPIO_AF3

/**
 * I2C defines
 */
#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif
#if I2C1_CLOCK_SPEED == 400000
#define I2C1_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C1 clock speed
#endif
#define I2C1_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           I2C1_DUTY_CYCLE,   \
           }

#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif
#if I2C2_CLOCK_SPEED == 400000
#define I2C2_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error Invalid I2C2 clock speed
#endif
#define I2C2_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           I2C2_DUTY_CYCLE,   \
           }

/**
 * SPI Config
 */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

// SLAVE0 on SPI connector
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO9
// SLAVE1 on AUX1
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO1
// SLAVE2 on AUX2
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO5
// SLAVE3 on AUX3
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO4
// SLAVE4 on AUX4
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN GPIO15

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/**
 * SDIO
 */
#define SDIO_D0_PORT GPIOC
#define SDIO_D0_PIN GPIOC_SDIO_D0
#define SDIO_D1_PORT GPIOC
#define SDIO_D1_PIN GPIOC_SDIO_D1
#define SDIO_D2_PORT GPIOC
#define SDIO_D2_PIN GPIOC_SDIO_D2
#define SDIO_D3_PORT GPIOC
#define SDIO_D3_PIN GPIOC_SDIO_D3
#define SDIO_CK_PORT GPIOC
#define SDIO_CK_PIN GPIOC_SDIO_CK
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN GPIOD_SDIO_CMD
#define SDIO_AF 12
// bat monitoring for file closing
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_4_CHANNEL
// usb led status
#define SDLOG_USB_LED 4
#define SDLOG_USB_VBUS_PORT GPIOA
#define SDLOG_USB_VBUS_PIN GPIO9

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
