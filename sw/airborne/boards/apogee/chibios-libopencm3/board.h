/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#define BOARD_NAME  "AB/GRZ STM32F4 Apogee 0.99"


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
#define STM32F40_41xxx
//#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX              0 // FDTI
#define GPIOA_UART4_RX              1 // FDTI
#define GPIOA_PWM2_CH3              2 // SERVO 1
#define GPIOA_PWM2_CH4              3 // SERVO 0
#define GPIOA_ADC1_IN4              4 // BAT ADC
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MISO             6
#define GPIOA_SPI1_MOSI             7
#define GPIOA_ICU1_CH1              8 // PPM_IN
#define GPIOA_OTG_FS_VBUS           9 //
#define GPIOA_USART1_RX             10 // OLED
#define GPIOA_OTG_FS_DM             11 //
#define GPIOA_OTG_FS_DP             12 //
#define GPIOA_SWDIO                 13 // SERIAL WIRE DEBUG
#define GPIOA_SWCLK                 14 // SERIAL WIRE DEBUG
#define GPIOA_PWM2_CH1              15 // SERVO 5

#define GPIOB_PWM3_CH3              0 // SERVO0
#define GPIOB_AUX_PIN1              1 // AUX1_SRV_7 (TIM3_CHANNEL_4 when pwm)
#define GPIOB_BOOT1                 2 //
#define GPIOB_PWM2_CH2              3 // SERVO 4
#define GPIOB_PWM3_CH1              4 // SERVO 3
#define GPIOB_PWM3_CH2              5 // SERVO 2
#define GPIOB_USART1_TX             6 // OLED
#define GPIOB_I2C1_SDA              7
#define GPIOB_I2C1_SCL              8
#define GPIOB_SPI1_CS               9 //
#define GPIOB_I2C2_SCL              10
#define GPIOB_I2C2_SDA              11
#define GPIOB_PIN12                 12 //
#define GPIOB_PIN13                 13 //
#define GPIOB_SDIO_DETECT           14 //
#define GPIOB_AUX_PIN15             15 // AUX4, only GPIO capable, should be modified for apogee V2 (C5 instead)

#define GPIOC_LED1                  0 //
#define GPIOC_LED3                  1 // NOT YET USED, ADC Capable
#define GPIOC_PIN2                  2 // NOT YET USED, ADC Capable
#define GPIOC_LED4                  3 // NOT YET USED, ADC Capable
#define GPIOC_AUX_PIN4              4 // AUX3 ADC Capable
#define GPIOC_AUX_PIN5              5 // AUX2 ADC Capable
#define GPIOC_USART6_TX             6 // jaune
#define GPIOC_USART6_RX             7 // vert
#define GPIOC_SDIO_D0               8
#define GPIOC_SDIO_D1               9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CK               12
#define GPIOC_LED2                  13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_SDIO_CMD              2

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
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
 * GPIOA_UART4_TX              0 // FDTI
 * GPIOA_UART4_RX              1 // FDTI
 * GPIOA_PWM2_CH3              2 // SERVO 2
 * GPIOA_PWM2_CH4              3 // SERVO 3
 * GPIOA_ADC1_IN4              4 // BAT ADC
 * GPIOA_SPI1_SCK              5
 * GPIOA_SPI1_MISO             6
 * GPIOA_SPI1_MOSI             7
 * GPIOA_ICU1_CH1              8 // PPM_IN
 * GPIOA_OTG_FS_VBUS           9 //
 * GPIOA_USART1_RX             10 // OLED
 * GPIOA_OTG_FS_DM                 11 //
 * GPIOA_OTG_FS_DP                 12 //
 * GPIOA_SWDIO                 13 // SERIAL WIRE DEBUG
 * GPIOA_SWCLK                 14 // SERIAL WIRE DEBUG
 * GPIOA_PWM2_CH1              15 // SERVO 0
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_UART4_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_UART4_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM2_CH3) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM2_CH4) |     \
                                     PIN_MODE_ANALOG   (GPIOA_ADC1_IN4) |     \
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
                                     PIN_MODE_ALTERNATE(GPIOA_PWM2_CH1))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM2_CH3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM2_CH4) |     \
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
                                     PIN_OSPEED_100M(GPIOA_PWM2_CH4) |     \
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
                                     PIN_PUPDR_FLOATING(GPIOA_PWM2_CH4) |     \
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
                                     PIN_ODR_HIGH(GPIOA_PWM2_CH4) |     \
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
                                     PIN_AFIO_AF(GPIOA_PWM2_CH3, 1) |   \
                                     PIN_AFIO_AF(GPIOA_PWM2_CH4, 1) |   \
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
                                     PIN_AFIO_AF(GPIOA_PWM2_CH1, 1))

/*
 * GPIOB setup:
 *
 * GPIOB_PWM3_CH3              0 // SERVO0
 * GPIOB_AUX_PIN1              1 // AUX_SRV_7 (TIM3_CHANNEL_4 when pwm)
 * GPIOB_BOOT1               2 //
 * GPIOB_PWM2_CH2              3 // SERVO 1
 * GPIOB_PWM3_CH1              4 // SERVO 4
 * GPIOB_PWM3_CH2              5 // SERVO 5
 * GPIOB_USART1_TX             6 // OLED
 * GPIOB_I2C1_SDA              7
 * GPIOB_I2C1_SCL              8
 * GPIOB_SPI1_CS                  9 //
 * GPIOB_I2C2_SCL              10
 * GPIOB_I2C2_SDA              11
 * GPIOB_PIN12                 12 //
 * GPIOB_PIN13                 13 //
 * GPIOB_SDIO_DETECT           14 //
 * GPIOB_AUX_PIN15                 15 //
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PWM3_CH3)        | \
                                     PIN_MODE_INPUT(GPIOB_AUX_PIN1)        | \
                                     PIN_MODE_INPUT(GPIOB_BOOT1)           | \
                                     PIN_MODE_ALTERNATE(GPIOB_PWM2_CH2)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_PWM3_CH1)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_PWM3_CH2)    | \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_TX)   | \
                                     PIN_MODE_INPUT(GPIOB_I2C1_SDA)    | \
                                     PIN_MODE_INPUT(GPIOB_I2C1_SCL)    | \
                                     PIN_MODE_OUTPUT(GPIOB_SPI1_CS)            | \
                                     PIN_MODE_INPUT(GPIOB_I2C2_SCL)    | \
                                     PIN_MODE_INPUT(GPIOB_I2C2_SDA)    | \
                                     PIN_MODE_INPUT(GPIOB_PIN12)    | \
                                     PIN_MODE_INPUT(GPIOB_PIN13)   | \
                                     PIN_MODE_INPUT(GPIOB_SDIO_DETECT) | \
                                     PIN_MODE_INPUT(GPIOB_AUX_PIN15))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH3)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AUX_PIN1)    | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_BOOT1)        | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM2_CH2)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH1)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM3_CH2)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART1_TX)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI1_CS)        | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL)   | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12)     | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO_DETECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AUX_PIN15))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PWM3_CH3) |          \
                                     PIN_OSPEED_100M(GPIOB_AUX_PIN1) |          \
                                     PIN_OSPEED_100M(GPIOB_BOOT1) |          \
                                     PIN_OSPEED_100M(GPIOB_PWM2_CH2) |           \
                                     PIN_OSPEED_100M(GPIOB_PWM3_CH1) |          \
                                     PIN_OSPEED_100M(GPIOB_PWM3_CH2) |      \
                                     PIN_OSPEED_100M(GPIOB_USART1_TX) |     \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SDA) |     \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SCL) |          \
                                     PIN_OSPEED_100M(GPIOB_SPI1_CS) |           \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SCL) |     \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SDA) |     \
                                     PIN_OSPEED_100M(GPIOB_PIN12) |         \
                                     PIN_OSPEED_100M(GPIOB_PIN13) |         \
                                     PIN_OSPEED_100M(GPIOB_SDIO_DETECT) |         \
                                     PIN_OSPEED_100M(GPIOB_AUX_PIN15))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PWM3_CH3) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_AUX_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PWM2_CH2) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PWM3_CH1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PWM3_CH2) | \
                                     PIN_PUPDR_FLOATING(GPIOB_USART1_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI1_CS) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SCL) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SDA) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_SDIO_DETECT) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_AUX_PIN15))

#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_PWM3_CH3) |             \
                                     PIN_ODR_LOW(GPIOB_AUX_PIN1) |             \
                                     PIN_ODR_LOW(GPIOB_BOOT1) |             \
                                     PIN_ODR_HIGH(GPIOB_PWM2_CH2) |              \
                                     PIN_ODR_HIGH(GPIOB_PWM3_CH1) |             \
                                     PIN_ODR_HIGH(GPIOB_PWM3_CH2) |         \
                                     PIN_ODR_HIGH(GPIOB_USART1_TX) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |             \
                                     PIN_ODR_HIGH(GPIOB_SPI1_CS) |              \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SCL) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA) |        \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOB_SDIO_DETECT) |            \
                                     PIN_ODR_HIGH(GPIOB_AUX_PIN15))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PWM3_CH3, 0) |           \
                                     PIN_AFIO_AF(GPIOB_AUX_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PWM2_CH2, 1) |            \
                                     PIN_AFIO_AF(GPIOB_PWM3_CH1, 2) |           \
                                     PIN_AFIO_AF(GPIOB_PWM3_CH2, 2) |           \
                                     PIN_AFIO_AF(GPIOB_USART1_TX, 7) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 0) |       \
                                     PIN_AFIO_AF(GPIOB_SPI1_CS, 0) |       \
                                     PIN_AFIO_AF(GPIOB_I2C2_SCL, 0) |       \
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA, 0) |       \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOB_SDIO_DETECT, 0) |          \
                                     PIN_AFIO_AF(GPIOB_AUX_PIN15, 0))

/*
 * GPIOC setup:
 *
 * GPIOC_PIN0                  0
 * GPIOC_LED3                  1
 * GPIOC_PIN2                  2
 * GPIOC_LED4                  3
 * GPIOC_AUX_PIN4              4
 * GPIOC_AUX_PIN5              5
 * GPIOC_USART6_TX             6
 * GPIOC_USART6_RX             7
 * GPIOC_SDIO_D0               8
 * GPIOC_SDIO_D1               9
 * GPIOC_SDIO_D2               10
 * GPIOC_SDIO_D3               11
 * GPIOC_SDIO_CK               12
 * GPIOC_LED2                  13
 * GPIOC_OSC32_IN              14
 * GPIOC_OSC32_OUT             15
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED1) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED3) |           \
                                     PIN_MODE_OUTPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_OUTPUT(GPIOC_LED4) |           \
                                     PIN_MODE_INPUT(GPIOC_AUX_PIN4) |       \
                                     PIN_MODE_INPUT(GPIOC_AUX_PIN5) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CK) |   \
                                     PIN_MODE_OUTPUT(GPIOC_LED2) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |      \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOC_LED1) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_TX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_RX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CK) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_LED1) |\
                                     PIN_OSPEED_100M(GPIOC_LED3) |          \
                                     PIN_OSPEED_100M(GPIOC_PIN2) |          \
                                     PIN_OSPEED_100M(GPIOC_LED4) |       \
                                     PIN_OSPEED_100M(GPIOC_AUX_PIN4) |          \
                                     PIN_OSPEED_100M(GPIOC_AUX_PIN5) |          \
                                     PIN_OSPEED_100M(GPIOC_USART6_TX) |          \
                                     PIN_OSPEED_100M(GPIOC_USART6_RX) |          \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D0) |     \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D1) |               \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D2) |               \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D3) |               \
                                     PIN_OSPEED_100M(GPIOC_SDIO_CK) |   \
                                     PIN_OSPEED_100M(GPIOC_LED2) |         \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) |         \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_LED1) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_LED3) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_LED4) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_AUX_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_AUX_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_TX) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_RX) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO_CK) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_LED2) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))

#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_LED1) |  \
                                     PIN_ODR_HIGH(GPIOC_LED3) |             \
                                     PIN_ODR_LOW(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_LED4) |          \
                                     PIN_ODR_LOW(GPIOC_AUX_PIN4) |             \
                                     PIN_ODR_LOW(GPIOC_AUX_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOC_USART6_TX) |             \
                                     PIN_ODR_HIGH(GPIOC_USART6_RX) |             \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0) |             \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1) |             \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2) |             \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_CK) |             \
                                     PIN_ODR_HIGH(GPIOC_LED2) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))

#define VAL_GPIOC_AFRL             (PIN_AFIO_AF(GPIOC_LED1, 0) |           \
                                    PIN_AFIO_AF(GPIOC_LED3, 0) |           \
                                    PIN_AFIO_AF(GPIOC_PIN2, 0) |           \
                                    PIN_AFIO_AF(GPIOC_LED4, 0) |           \
                                    PIN_AFIO_AF(GPIOC_AUX_PIN4, 0) |       \
                                    PIN_AFIO_AF(GPIOC_AUX_PIN5, 0) |       \
                                    PIN_AFIO_AF(GPIOC_USART6_TX, 8) |      \
                                    PIN_AFIO_AF(GPIOC_USART6_RX, 8))

#define VAL_GPIOC_AFRH             (PIN_AFIO_AF(GPIOC_SDIO_D0, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D1, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D2, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_D3, 12) |       \
                                    PIN_AFIO_AF(GPIOC_SDIO_CK, 12) |       \
                                    PIN_AFIO_AF(GPIOC_LED2, 0) |           \
                                    PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |       \
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
