/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for STMicroelectronics STM32373C-vortex board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32373C_XVERT
#define BOARD_NAME                  "STMicroelectronics STM32373C-xvert"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                16000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32F373xC

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_LED                   1U
#define GPIOA_UART2TX               2U
#define GPIOA_UART2RX               3U
#define GPIOA_PIN4                  4U
#define GPIOA_ADC1                  5U
#define GPIOA_ADC2                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_UART1TX               9U
#define GPIOA_UART1RX               10U
#define GPIOA_PWM_PIN11             11U
#define GPIOA_PWM_PIN12             12U
#define GPIOA_SWD                   13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_MIC_IN                0U
#define GPIOB_ADC_POT_IN            1U
#define GPIOB_PIN2                  2U
#define GPIOB_SWO                   3U
#define GPIOB_JTRST                 4U
#define GPIOB_PIN5                  5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_PIN11                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_PIN13                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOC_LED1                  0U
#define GPIOC_LED2                  1U
#define GPIOC_LED3                  2U
#define GPIOC_LED4                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_USB_DISCONNECT        5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_CAN_RX                0U
#define GPIOD_CAN_TX                1U
#define GPIOD_LCD_CS                2U
#define GPIOD_USART2_CTS            3U
#define GPIOD_USART2_RST            4U
#define GPIOD_USART2_TX             5U
#define GPIOD_USART2_RX             6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_AUDIO_RST             11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_SD_CS                 2U
#define GPIOE_SD_DETECT             3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_JOY_SEL               6U
#define GPIOE_RTD_IN                7U
#define GPIOE_PRESSUREP             8U
#define GPIOE_PRESSUREN             9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PRESSURE_TEPM         14U
#define GPIOE_PIN15                 15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U
#define GPIOF_JOY_DOWN              2U
#define GPIOF_PIN3                  3U
#define GPIOF_JOY_LEFT              4U
#define GPIOF_PIN5                  5U
#define GPIOF_I2C2_SCL              6U
#define GPIOF_I2C2_SDA              7U
#define GPIOF_PIN8                  8U
#define GPIOF_JOY_RIGHT             9U
#define GPIOF_JOY_UP                10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_PIN0                  0U
#define GPIOH_PIN1                  1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

/*
 * IO lines assignments.
 */
//#define LINE_WKUP_BUTTON            PAL_LINE(GPIOA, 0U)
//#define LINE_LDR_OUT                PAL_LINE(GPIOA, 1U)
//#define LINE_KEY_BUTTON             PAL_LINE(GPIOA, 2U)
//#define LINE_COMP2_OUT              PAL_LINE(GPIOA, 7U)
//#define LINE_I2C2_SMB               PAL_LINE(GPIOA, 8U)
//#define LINE_I2C2_SCL               PAL_LINE(GPIOA, 9U)
//#define LINE_I2C2_SDA               PAL_LINE(GPIOA, 10U)
//#define LINE_USB_DM                 PAL_LINE(GPIOA, 11U)
//#define LINE_USB_DP                 PAL_LINE(GPIOA, 12U)
//#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
//#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
//#define LINE_JTDI                   PAL_LINE(GPIOA, 15U)

//#define LINE_MIC_IN                 PAL_LINE(GPIOB, 0U)
//#define LINE_ADC_POT_IN             PAL_LINE(GPIOB, 1U)
//#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
//#define LINE_JTRST                  PAL_LINE(GPIOB, 4U)
//#define LINE_I2C1_SCL               PAL_LINE(GPIOB, 6U)
//#define LINE_I2C1_SDA               PAL_LINE(GPIOB, 7U)

//#define LINE_LED1                   PAL_LINE(GPIOC, 0U)
//#define LINE_LED2                   PAL_LINE(GPIOC, 1U)
//#define LINE_LED3                   PAL_LINE(GPIOC, 2U)
//#define LINE_LED4                   PAL_LINE(GPIOC, 3U)
//#define LINE_USB_DISCONNECT         PAL_LINE(GPIOC, 5U)
//#define LINE_SPI3_SCK               PAL_LINE(GPIOC, 10U)
//#define LINE_SPI3_MISO              PAL_LINE(GPIOC, 11U)
//#define LINE_SPI3_MOSI              PAL_LINE(GPIOC, 12U)
//#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
//#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)

//#define LINE_CAN_RX                 PAL_LINE(GPIOD, 0U)
//#define LINE_CAN_TX                 PAL_LINE(GPIOD, 1U)
//#define LINE_LCD_CS                 PAL_LINE(GPIOD, 2U)
//#define LINE_USART2_CTS             PAL_LINE(GPIOD, 3U)
//#define LINE_USART2_RST             PAL_LINE(GPIOD, 4U)
//#define LINE_USART2_TX              PAL_LINE(GPIOD, 5U)
//#define LINE_USART2_RX              PAL_LINE(GPIOD, 6U)
//#define LINE_AUDIO_RST              PAL_LINE(GPIOD, 11U)

//#define LINE_SD_CS                  PAL_LINE(GPIOE, 2U)
//#define LINE_SD_DETECT              PAL_LINE(GPIOE, 3U)
//#define LINE_JOY_SEL                PAL_LINE(GPIOE, 6U)
//#define LINE_RTD_IN                 PAL_LINE(GPIOE, 7U)
//#define LINE_PRESSUREP              PAL_LINE(GPIOE, 8U)
//#define LINE_PRESSUREN              PAL_LINE(GPIOE, 9U)
//#define LINE_PRESSURE_TEPM          PAL_LINE(GPIOE, 14U)

//#define LINE_OSC_IN                 PAL_LINE(GPIOF, 0U)
//#define LINE_OSC_OUT                PAL_LINE(GPIOF, 1U)
//#define LINE_JOY_DOWN               PAL_LINE(GPIOF, 2U)
//#define LINE_JOY_LEFT               PAL_LINE(GPIOF, 4U)
//#define LINE_JOY_RIGHT              PAL_LINE(GPIOF, 9U)
//#define LINE_JOY_UP                 PAL_LINE(GPIOF, 10U)

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
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))


#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |    \
                                     PIN_MODE_OUTPUT(GPIOA_LED) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2RX) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_ANALOG(GPIOA_ADC1) |           \
                                     PIN_MODE_ANALOG(GPIOA_ADC2) |           \
                                     PIN_MODE_OUTPUT(GPIOA_PIN7) |     \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1RX) |   \
                                     PIN_MODE_INPUT(GPIOA_PWM_PIN11) |     \
                                     PIN_MODE_INPUT(GPIOA_PWM_PIN12) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWD) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN15))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2RX) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1RX) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PWM_PIN11) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PWM_PIN12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWD) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_PIN0) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_LED) |    \
                                     PIN_OSPEED_HIGH(GPIOA_UART2TX) | \
                                     PIN_OSPEED_HIGH(GPIOA_UART2RX) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC1) |          \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN8) |   \
                                     PIN_OSPEED_HIGH(GPIOA_UART1TX) |      \
                                     PIN_OSPEED_HIGH(GPIOA_UART1RX) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_PWM_PIN11) |        \
                                     PIN_OSPEED_VERYLOW(GPIOA_PWM_PIN12) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWD) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN15))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) |\
                                     PIN_PUPDR_FLOATING(GPIOA_LED) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_UART2TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART2RX) |         \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC2) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1RX) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PWM_PIN11) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PWM_PIN12) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_SWD) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |      \
                                     PIN_ODR_HIGH(GPIOA_LED) |          \
                                     PIN_ODR_HIGH(GPIOA_UART2TX) |       \
                                     PIN_ODR_HIGH(GPIOA_UART2RX) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC1) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC2) |             \
                                     PIN_ODR_LOW(GPIOA_PIN7) |         \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |         \
                                     PIN_ODR_HIGH(GPIOA_UART1TX) |         \
                                     PIN_ODR_HIGH(GPIOA_UART1RX) |         \
                                     PIN_ODR_HIGH(GPIOA_PWM_PIN11) |           \
                                     PIN_ODR_HIGH(GPIOA_PWM_PIN12) |           \
                                     PIN_ODR_HIGH(GPIOA_SWD) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_LED, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_UART2TX, 7U) |    \
                                     PIN_AFIO_AF(GPIOA_UART2RX, 7U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC1, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC2, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_UART1TX, 7U) |      \
                                     PIN_AFIO_AF(GPIOA_UART1RX, 7U) |      \
                                     PIN_AFIO_AF(GPIOA_PWM_PIN11, 2U) |       \
                                     PIN_AFIO_AF(GPIOA_PWM_PIN12, 2U) |       \
                                     PIN_AFIO_AF(GPIOA_SWD, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - MIC_IN                    (analog).
 * PB1  - ADC_POT_IN                (analog).
 * PB2  - PIN2                      (input pullup).
 * PB3  - SWO                       (alternate 0).
 * PB4  - JTRST                     (input floating).
 * PB5  - PIN5                      (input pullup).
 * PB6  - I2C1_SCL                  (alternate 4).
 * PB7  - I2C1_SDA                  (alternate 4).
 * PB8  - PIN8                      (input pullup).
 * PB9  - PIN9                      (input pullup).
 * PB10 - PIN10                     (input pullup).
 * PB11 - PIN11                     (input pullup).
 * PB12 - PIN12                     (input pullup).
 * PB13 - PIN13                     (input pullup).
 * PB14 - PIN14                     (input pullup).
 * PB15 - PIN15                     (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_MIC_IN) |        \
                                     PIN_MODE_ANALOG(GPIOB_ADC_POT_IN) |    \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) |        \
                                     PIN_MODE_INPUT(GPIOB_JTRST) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |   \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_MIC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ADC_POT_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTRST) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_MIC_IN) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_ADC_POT_IN) | \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN2) |       \
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |           \
                                     PIN_OSPEED_HIGH(GPIOB_JTRST) |         \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_MIC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_ADC_POT_IN) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_JTRST) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_MIC_IN) |           \
                                     PIN_ODR_HIGH(GPIOB_ADC_POT_IN) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_JTRST) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |         \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |         \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_MIC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_ADC_POT_IN, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_JTRST, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - LED1                      (output opendrain maximum).
 * PC1  - LED2                      (output opendrain maximum).
 * PC2  - LED3                      (output opendrain maximum).
 * PC3  - LED4                      (output opendrain maximum).
 * PC4  - PIN4                      (input pullup).
 * PC5  - USB_DISCONNECT            (output pushpull maximum).
 * PC6  - PIN6                      (input pullup).
 * PC7  - PIN7                      (input pullup).
 * PC8  - PIN8                      (input pullup).
 * PC9  - PIN9                      (input pullup).
 * PC10 - SPI3_SCK                  (alternate 6).
 * PC11 - SPI3_MISO                 (alternate 6).
 * PC12 - SPI3_MOSI                 (alternate 6).
 * PC13 - PIN13                     (input pullup).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED1) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED2) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED3) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED4) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |           \
                                     PIN_MODE_OUTPUT(GPIOC_USB_DISCONNECT) |\
                                     PIN_MODE_INPUT(GPIOC_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10) |   \
                                     PIN_MODE_INPUT(GPIOC_PIN11) |  \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |  \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOC_LED1) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED2) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED3) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED4) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USB_DISCONNECT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_LED1) |          \
                                     PIN_OSPEED_HIGH(GPIOC_LED2) |          \
                                     PIN_OSPEED_HIGH(GPIOC_LED3) |          \
                                     PIN_OSPEED_HIGH(GPIOC_LED4) |          \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN4) |       \
                                     PIN_OSPEED_HIGH(GPIOC_USB_DISCONNECT) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN11) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN12) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN13) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_LED1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_LED2) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_LED3) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_LED4) |       \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_USB_DISCONNECT) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_LED1) |             \
                                     PIN_ODR_HIGH(GPIOC_LED2) |             \
                                     PIN_ODR_HIGH(GPIOC_LED3) |             \
                                     PIN_ODR_HIGH(GPIOC_LED4) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOC_USB_DISCONNECT) |   \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10) |         \
                                     PIN_ODR_HIGH(GPIOC_PIN11) |        \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |        \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_LED1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_LED2, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_LED3, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_LED4, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_USB_DISCONNECT, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - CAN_RX                    (alternate 7).
 * PD1  - CAN_TX                    (alternate 7).
 * PD2  - LCD_CS                    (output pushpull maximum).
 * PD3  - USART2_CTS                (alternate 7).
 * PD4  - USART2_RST                (alternate 7).
 * PD5  - USART2_TX                 (alternate 7).
 * PD6  - USART2_RX                 (alternate 7).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - AUDIO_RST                 (output pushpull maximum).
 * PD12 - PIN12                     (input pullup).
 * PD13 - PIN13                     (input pullup).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN_TX) |     \
                                     PIN_MODE_OUTPUT(GPIOD_LCD_CS) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_CTS) | \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_RST) | \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_RX) |  \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_OUTPUT(GPIOD_AUDIO_RST) |     \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_CTS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_RST) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUDIO_RST) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_CAN_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CAN_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_CS) |        \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_CTS) |    \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_RST) |    \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_RX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN10) |      \
                                     PIN_OSPEED_HIGH(GPIOD_AUDIO_RST) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_CAN_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_CAN_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_CS) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_CTS) | \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_RST) | \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_RX) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_AUDIO_RST) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_CAN_RX) |           \
                                     PIN_ODR_HIGH(GPIOD_CAN_TX) |           \
                                     PIN_ODR_HIGH(GPIOD_LCD_CS) |           \
                                     PIN_ODR_HIGH(GPIOD_USART2_CTS) |       \
                                     PIN_ODR_HIGH(GPIOD_USART2_RST) |       \
                                     PIN_ODR_HIGH(GPIOD_USART2_TX) |        \
                                     PIN_ODR_HIGH(GPIOD_USART2_RX) |        \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_LOW(GPIOD_AUDIO_RST) |         \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN_RX, 7U) |        \
                                     PIN_AFIO_AF(GPIOD_CAN_TX, 7U) |        \
                                     PIN_AFIO_AF(GPIOD_LCD_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_USART2_CTS, 7U) |    \
                                     PIN_AFIO_AF(GPIOD_USART2_RST, 7U) |    \
                                     PIN_AFIO_AF(GPIOD_USART2_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_USART2_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_AUDIO_RST, 0U) |     \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - SD_CS                     (output opendrain maximum).
 * PE3  - SD_DETECT                 (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - JOY_SEL                   (input pulldown).
 * PE7  - RTD_IN                    (analog).
 * PE8  - PRESSUREP                 (analog).
 * PE9  - PRESSUREN                 (analog).
 * PE10 - PIN10                     (input pullup).
 * PE11 - PIN11                     (input pullup).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PRESSURE_TEPM             (input floating).
 * PE15 - PIN15                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_OUTPUT(GPIOE_SD_CS) |         \
                                     PIN_MODE_INPUT(GPIOE_SD_DETECT) |      \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_JOY_SEL) |        \
                                     PIN_MODE_ANALOG(GPIOE_RTD_IN) |        \
                                     PIN_MODE_ANALOG(GPIOE_PRESSUREP) |     \
                                     PIN_MODE_ANALOG(GPIOE_PRESSUREN) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOE_PRESSURE_TEPM) |  \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_SD_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SD_DETECT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_JOY_SEL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_RTD_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PRESSUREP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PRESSUREN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PRESSURE_TEPM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SD_CS) |         \
                                     PIN_OSPEED_HIGH(GPIOE_SD_DETECT) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOE_JOY_SEL) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_RTD_IN) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PRESSUREP) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PRESSUREN) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PRESSURE_TEPM) |\
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_SD_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_SD_DETECT) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_JOY_SEL) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_RTD_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PRESSUREP) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PRESSUREN) |  \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PRESSURE_TEPM) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_SD_CS) |            \
                                     PIN_ODR_HIGH(GPIOE_SD_DETECT) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_JOY_SEL) |          \
                                     PIN_ODR_HIGH(GPIOE_RTD_IN) |           \
                                     PIN_ODR_HIGH(GPIOE_PRESSUREP) |        \
                                     PIN_ODR_HIGH(GPIOE_PRESSUREN) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |            \
                                     PIN_ODR_LOW(GPIOE_PIN13) |             \
                                     PIN_ODR_LOW(GPIOE_PRESSURE_TEPM) |     \
                                     PIN_ODR_LOW(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SD_CS, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_SD_DETECT, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_JOY_SEL, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_RTD_IN, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PRESSUREP, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PRESSUREN, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PRESSURE_TEPM, 0U) | \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (input floating).
 * PF1  - OSC_OUT                   (input floating).
 * PF2  - JOY_DOWN                  (input pulldown).
 * PF3  - PIN3                      (input pullup).
 * PF4  - JOY_LEFT                  (input pulldown).
 * PF5  - PIN5                      (input pullup).
 * PF6  - PIN6                      (input pullup).
 * PF7  - PIN7                      (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - JOY_RIGHT                 (input pulldown).
 * PF10 - JOY_UP                    (input pulldown).
 * PF11 - PIN11                     (input pullup).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOF_JOY_DOWN) |       \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_JOY_LEFT) |       \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_SCL) |           \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_SDA) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_JOY_RIGHT) |      \
                                     PIN_MODE_INPUT(GPIOF_JOY_UP) |         \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_JOY_DOWN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_JOY_LEFT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SCL) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_JOY_RIGHT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_JOY_UP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOF_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOF_JOY_DOWN) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN3) |       \
                                     PIN_OSPEED_HIGH(GPIOF_JOY_LEFT) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOF_I2C2_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOF_I2C2_SDA) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOF_JOY_RIGHT) |     \
                                     PIN_OSPEED_HIGH(GPIOF_JOY_UP) |        \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOF_JOY_DOWN) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_JOY_LEFT) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_I2C2_SCL) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_I2C2_SDA) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_JOY_RIGHT) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOF_JOY_UP) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOF_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOF_JOY_DOWN) |         \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_JOY_LEFT) |         \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SCL) |             \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SDA) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_JOY_RIGHT) |        \
                                     PIN_ODR_HIGH(GPIOF_JOY_UP) |           \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_JOY_DOWN, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_JOY_LEFT, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_I2C2_SCL, 4U) |          \
                                     PIN_AFIO_AF(GPIOF_I2C2_SDA, 4U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_JOY_RIGHT, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_JOY_UP, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input pullup).
 * PG1  - PIN1                      (input pullup).
 * PG2  - PIN2                      (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - PIN6                      (input pullup).
 * PG7  - PIN7                      (input pullup).
 * PG8  - PIN8                      (input pullup).
 * PG9  - PIN9                      (input pullup).
 * PG10 - PIN10                     (input pullup).
 * PG11 - PIN11                     (input pullup).
 * PG12 - PIN12                     (input pullup).
 * PG13 - PIN13                     (input pullup).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - PIN0                      (input pullup).
 * PH1  - PIN1                      (input pullup).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |           \
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
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |       \
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
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOH_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLUP(GPIOH_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |             \
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
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

//*****************************************************************************

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * LEDs
 */
/* red */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* green */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOD
#define LED_2_GPIO_PIN GPIO8
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/*orange  */
#ifndef USE_LED_3
#define USE_LED_3 0
#endif
#define LED_3_GPIO GPIOE
#define LED_3_GPIO_PIN GPIO10
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* green */
#ifndef USE_LED_4
#define USE_LED_4 0
#endif
#define LED_4_GPIO GPIOE
#define LED_4_GPIO_PIN GPIO11
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

/* blue*/
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOE
#define LED_5_GPIO_PIN GPIO12
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* red*/
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOE
#define LED_6_GPIO_PIN GPIO13
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* orange*/
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOE
#define LED_7_GPIO_PIN GPIO13
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* green*/
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOE
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

#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN4
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOA
#define ADC_4_GPIO_PIN GPIO4
#endif

///* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
//#ifndef ADC_CHANNEL_VSUPPLY
//#define ADC_CHANNEL_VSUPPLY ADC_4
//#endif

//#define DefaultVoltageOfAdc(adc) (0.006185*adc)

#define ACTUATORS_PWM_NB 4
//the first two motors are through the xvert escs
//the second two are two standard pwm servos

#define XVERT_ESC_0 0
#define XVERT_ESC_1 1

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO11
#define PWM_SERVO_2_AF GPIO_AF2 //alternate function of the pin, alsu used to select which timer (table 12 in datasheet)
#define PWM_SERVO_2_DRIVER PWMD5 //timer ID. Which timer is being used by this pwm pin
#define PWM_SERVO_2_CHANNEL 1   //channel *in* the timer Find it in table 12 (subtract by 1!)
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOA
#define PWM_SERVO_3_PIN GPIO12
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD5
#define PWM_SERVO_3_CHANNEL 2
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#if !STM32_PWM_USE_TIM5
#define PWM_CONF_TIM5 STM32_PWM_USE_TIM5
#else
#define PWM_CONF_TIM5 1
#endif
#define PWM_CONF5_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM5_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_3_ACTIVE, NULL }, \
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

///*
// * PWM input
// */
//// PWM_INPUT 1 on PA8 (also PPM IN)
//#define PWM_INPUT1_ICU            ICUD1
//#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
//// PPM in (aka PA8) is used: not compatible with PPM RC receiver
//#define PWM_INPUT1_GPIO_PORT      GPIOA
//#define PWM_INPUT1_GPIO_PIN       GPIO8
//#define PWM_INPUT1_GPIO_AF        GPIO_AF1

//// PWM_INPUT 2 on PA3 (also SERVO 1)
//#if (USE_PWM1 && USE_PWM_INPUT2)
//#error "PW1 and PWM_INPUT2 are not compatible"
//#endif
//#define PWM_INPUT2_ICU            ICUD9
//#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_1
//#define PWM_INPUT2_GPIO_PORT      GPIOA
//#define PWM_INPUT2_GPIO_PIN       GPIO2
//#define PWM_INPUT2_GPIO_AF        GPIO_AF3

/**
 * I2C defines
 */
#define I2C1_CFG_DEF {      \
STM32_TIMINGR_PRESC(15U) |  \
STM32_TIMINGR_SCLDEL(4U) |  \
STM32_TIMINGR_SDADEL(2U) |  \
STM32_TIMINGR_SCLH(15U)  |  \
STM32_TIMINGR_SCLL(21U),    \
0,0 }

#define I2C2_CFG_DEF {      \
STM32_TIMINGR_PRESC(15U) |  \
STM32_TIMINGR_SCLDEL(4U) |  \
STM32_TIMINGR_SDADEL(2U) |  \
STM32_TIMINGR_SCLH(15U)  |  \
STM32_TIMINGR_SCLL(21U),    \
0,0 }


///**
// * SPI Config
// */
//#define SPI1_GPIO_AF GPIO_AF5
//#define SPI1_GPIO_PORT_MISO GPIOA
//#define SPI1_GPIO_MISO GPIO6
//#define SPI1_GPIO_PORT_MOSI GPIOA
//#define SPI1_GPIO_MOSI GPIO7
//#define SPI1_GPIO_PORT_SCK GPIOA
//#define SPI1_GPIO_SCK GPIO5

//// SLAVE0 on SPI connector
//#define SPI_SELECT_SLAVE0_PORT GPIOB
//#define SPI_SELECT_SLAVE0_PIN GPIO9
//// SLAVE1 on AUX1
//#define SPI_SELECT_SLAVE1_PORT GPIOB
//#define SPI_SELECT_SLAVE1_PIN GPIO1
//// SLAVE2 on AUX2
//#define SPI_SELECT_SLAVE2_PORT GPIOC
//#define SPI_SELECT_SLAVE2_PIN GPIO5
//// SLAVE3 on AUX3
//#define SPI_SELECT_SLAVE3_PORT GPIOC
//#define SPI_SELECT_SLAVE3_PIN GPIO4
//// SLAVE4 on AUX4
//#define SPI_SELECT_SLAVE4_PORT GPIOB
//#define SPI_SELECT_SLAVE4_PIN GPIO15

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


///**
// * SDIO
// */
//#define SDIO_D0_PORT GPIOC
//#define SDIO_D0_PIN GPIOC_SDIO_D0
//#define SDIO_D1_PORT GPIOC
//#define SDIO_D1_PIN GPIOC_SDIO_D1
//#define SDIO_D2_PORT GPIOC
//#define SDIO_D2_PIN GPIOC_SDIO_D2
//#define SDIO_D3_PORT GPIOC
//#define SDIO_D3_PIN GPIOC_SDIO_D3
//#define SDIO_CK_PORT GPIOC
//#define SDIO_CK_PIN GPIOC_SDIO_CK
//#define SDIO_CMD_PORT GPIOD
//#define SDIO_CMD_PIN GPIOD_SDIO_CMD
//#define SDIO_AF 12
//// bat monitoring for file closing
//#define SDLOG_BAT_ADC ADCD1
//#define SDLOG_BAT_CHAN AD1_4_CHANNEL
//// usb led status
//#define SDLOG_USB_LED 4
//#define SDLOG_USB_VBUS_PORT GPIOA
//#define SDLOG_USB_VBUS_PIN GPIO9


//*****************************************************************************

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
