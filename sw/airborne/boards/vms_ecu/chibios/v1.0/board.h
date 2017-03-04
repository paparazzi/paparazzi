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
 * Setup for STMicroelectronics STM32F4-Lisa MX.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F4_ECU
#define BOARD_NAME "STMicroelectronics STM32F4-Ecu"


/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000
#endif


/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F407xx


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
 * Port A setup.
 *
 * PA0  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA1  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA2  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA3  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA4  - Analog output (DAC1) - set up as INPUT ANALOG (per reference manual)
 * PA5  - Analog output (DAC2) - set up as INPUT ANALOG (per reference manual)
 * PA6  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA7  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA8  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA9  - Alternate output                   (UART1_Tx)
 * PA10 - Alternate input                   (UART1_Rx)
 * PA11 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA12 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PA13 - Digital input                    (JTAG_TMS/SWDIO)
 * PA14 - Digital input                    (JTAG_TCK/SWCLCK)
 * PA15 - Digital input                    (JTAG_TDI)
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(0)| \
                                     PIN_MODE_INPUT(1) | \
                                     PIN_MODE_INPUT(2) | \
                                     PIN_MODE_INPUT(3)     | \
                                     PIN_MODE_INPUT(4) | \
                                     PIN_MODE_INPUT(5) | \
                                     PIN_MODE_INPUT(6)     | \
                                     PIN_MODE_INPUT(7) | \
                                     PIN_MODE_INPUT(8)    | \
                                     PIN_MODE_ALTERNATE(9)     | \
                                     PIN_MODE_ALTERNATE(10)    | \
                                     PIN_MODE_INPUT(11)    | \
                                     PIN_MODE_INPUT(12)    | \
                                     PIN_MODE_ALTERNATE(13)    | \
                                     PIN_MODE_ALTERNATE(14)    | \
                                     PIN_MODE_ALTERNATE(15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(0) | \
                                     PIN_OTYPE_PUSHPULL(1)  | \
                                     PIN_OTYPE_PUSHPULL(2)  | \
                                     PIN_OTYPE_PUSHPULL(3)  | \
                                     PIN_OTYPE_PUSHPULL(4)  | \
                                     PIN_OTYPE_PUSHPULL(5)  | \
                                     PIN_OTYPE_PUSHPULL(6)  | \
                                     PIN_OTYPE_PUSHPULL(7)  | \
                                     PIN_OTYPE_PUSHPULL(8) | \
                                     PIN_OTYPE_PUSHPULL(9)  | \
                                     PIN_OTYPE_PUSHPULL(10) | \
                                     PIN_OTYPE_PUSHPULL(11) | \
                                     PIN_OTYPE_PUSHPULL(12) | \
                                     PIN_OTYPE_PUSHPULL(13) | \
                                     PIN_OTYPE_PUSHPULL(14) | \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(0) | \
                                     PIN_OSPEED_100M(1)  | \
                                     PIN_OSPEED_100M(2)  | \
                                     PIN_OSPEED_100M(3)  | \
                                     PIN_OSPEED_100M(4)  | \
                                     PIN_OSPEED_100M(5)  | \
                                     PIN_OSPEED_100M(6)  | \
                                     PIN_OSPEED_100M(7)  | \
                                     PIN_OSPEED_100M(8)  | \
                                     PIN_OSPEED_100M(9)  | \
                                     PIN_OSPEED_100M(10) | \
                                     PIN_OSPEED_100M(11) | \
                                     PIN_OSPEED_100M(12) | \
                                     PIN_OSPEED_100M(13) | \
                                     PIN_OSPEED_100M(14) | \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(0) | \
                                     PIN_PUPDR_PULLDOWN(1)    | \
                                     PIN_PUPDR_PULLDOWN(2)    | \
                                     PIN_PUPDR_PULLDOWN(3)    | \
                                     PIN_PUPDR_FLOATING(4)  | \
                                     PIN_PUPDR_FLOATING(5)  | \
                                     PIN_PUPDR_PULLDOWN(6)  | \
                                     PIN_PUPDR_PULLDOWN(7)  | \
                                     PIN_PUPDR_PULLDOWN(8)  | \
                                     PIN_PUPDR_FLOATING(9)  | \
                                     PIN_PUPDR_FLOATING(10) | \
                                     PIN_PUPDR_PULLDOWN(11) | \
                                     PIN_PUPDR_PULLDOWN(12) | \
                                     PIN_PUPDR_FLOATING(13) | \
                                     PIN_PUPDR_FLOATING(14) | \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(0) | \
                                     PIN_ODR_LOW(1)  | \
                                     PIN_ODR_LOW(2)  | \
                                     PIN_ODR_LOW(3)  | \
                                     PIN_ODR_HIGH(4)  | \
                                     PIN_ODR_HIGH(5)  | \
                                     PIN_ODR_LOW(6)  | \
                                     PIN_ODR_LOW(7)  | \
                                     PIN_ODR_LOW(8)  | \
                                     PIN_ODR_HIGH(9)  | \
                                     PIN_ODR_LOW(10) | \
                                     PIN_ODR_LOW(11) | \
                                     PIN_ODR_LOW(12) | \
                                     PIN_ODR_HIGH(13) | \
                                     PIN_ODR_HIGH(14) | \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0, 0) | \
                                     PIN_AFIO_AF(1, 0)  | \
                                     PIN_AFIO_AF(2, 0)  | \
                                     PIN_AFIO_AF(3, 0)  | \
                                     PIN_AFIO_AF(4, 0)  | \
                                     PIN_AFIO_AF(5, 0)  | \
                                     PIN_AFIO_AF(6, 0)  | \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8, 0)  | \
                                     PIN_AFIO_AF(9, 7)   | \
                                     PIN_AFIO_AF(10, 7) | \
                                     PIN_AFIO_AF(11, 0) | \
                                     PIN_AFIO_AF(12, 0) | \
                                     PIN_AFIO_AF(13, 0)  | \
                                     PIN_AFIO_AF(14, 0)  | \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port B setup:
 * PB0  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB1  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB2  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB3  - Digital input                    (JTAG_TDO/SWD)
 * PB4  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB5  - Alternate Digital input.                   (CAN2_RX)
 * PB6  - Alternate Open Drain output 50MHz.         (CAN2_TX)
 * PB7  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB8  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB9  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PB10 - Alternate Open Drain output 2MHz.(I2C2_SCL)
 * PB11 - Alternate Open Drain output 2MHz.(I2C2_SDA)
 * PB12 - Push Pull output 50MHz.          (IMU_ACC_SPI2_CS)
 * PB13 - Alternate Push Pull output 50MHz (IMU_SPI2_SCK)
 * PB14 - Digital input                    (IMU_SPI2_MISO)
 * PB15 - Alternate Push Pull output 50MHz (IMU_SPI2_MOSI)
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_ALTERNATE(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_ALTERNATE(5) |           \
                                     PIN_MODE_ALTERNATE(6) |        \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |        \
                                     PIN_MODE_ALTERNATE(10) |         \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_OUTPUT(12) |          \
                                     PIN_MODE_ALTERNATE(13) |          \
                                     PIN_MODE_ALTERNATE(14) |          \
                                     PIN_MODE_ALTERNATE(15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |        \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_OPENDRAIN(10) |     \
                                     PIN_OTYPE_OPENDRAIN(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |           \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_50M(6) |           \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |           \
                                     PIN_OSPEED_2M(10) |        \
                                     PIN_OSPEED_2M(11) |         \
                                     PIN_OSPEED_50M(12) |         \
                                     PIN_OSPEED_50M(13) |         \
                                     PIN_OSPEED_50M(14) |         \
                                     PIN_OSPEED_50M(15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(0) |         \
                                     PIN_PUPDR_PULLDOWN(1) |         \
                                     PIN_PUPDR_PULLDOWN(2) |         \
                                     PIN_PUPDR_FLOATING(3) |        \
                                     PIN_PUPDR_PULLDOWN(4) |         \
                                     PIN_PUPDR_FLOATING(5) |         \
                                     PIN_PUPDR_FLOATING(6) |        \
                                     PIN_PUPDR_PULLDOWN(7) |         \
                                     PIN_PUPDR_PULLDOWN(8) |         \
                                     PIN_PUPDR_PULLDOWN(9) |        \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |        \
                                     PIN_PUPDR_FLOATING(13) |        \
                                     PIN_PUPDR_FLOATING(14) |        \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(0) |             \
                                     PIN_ODR_LOW(1) |             \
                                     PIN_ODR_LOW(2) |             \
                                     PIN_ODR_HIGH(3) |              \
                                     PIN_ODR_LOW(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |              \
                                     PIN_ODR_LOW(7) |             \
                                     PIN_ODR_LOW(8) |             \
                                     PIN_ODR_LOW(9) |              \
                                     PIN_ODR_HIGH(10) |           \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |            \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 9) |           \
                                     PIN_AFIO_AF(6, 9) |            \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |            \
                                     PIN_AFIO_AF(10, 4) |         \
                                     PIN_AFIO_AF(11, 4) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 5) |          \
                                     PIN_AFIO_AF(14, 5) |          \
                                     PIN_AFIO_AF(15, 5))

/*
 * Port C setup:
 * PC0  - Analog input                     (ADC1)
 * PC1  - Analog input                     (ADC2)
 * PC2  - Analog input                     (ADC3)
 * PC3  - Analog input                     (ADC4)
 * PC4  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PC5  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PC6  - Alternate Push Pull output 50MHz (UART6_TX)
 * PC7  - Alternate input                  (UART7_RX)
 * PC8  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PC9  - Push Pull output 50MHz           (SPI3_NSS)
 * PC10 - Alternate Push Pull output 50MHz (SPI3_SCK)
 * PC11 - Alternate Digital input          (SPI3_MISO)
 * PC12 - Alternate Push Pull output 50MHz (SPI3_MOSI)
 * PC13 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PC14 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 * PC15 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected)
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_ALTERNATE(7) |       \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_OUTPUT(9) |           \
                                     PIN_MODE_ALTERNATE(10) |       \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_ALTERNATE(12) |       \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |    \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |       \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |       \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(0) |\
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |       \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_50M(6) |          \
                                     PIN_OSPEED_50M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_50M(9) |          \
                                     PIN_OSPEED_50M(10) |          \
                                     PIN_OSPEED_50M(11) |         \
                                     PIN_OSPEED_50M(12) |          \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0) |\
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |      \
                                     PIN_PUPDR_PULLDOWN(4) |         \
                                     PIN_PUPDR_PULLDOWN(5) |         \
                                     PIN_PUPDR_FLOATING(6) |         \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_PULLDOWN(8) |         \
                                     PIN_PUPDR_FLOATING(9) |         \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |       \
                                     PIN_PUPDR_PULLDOWN(13) |        \
                                     PIN_PUPDR_PULLDOWN(14) |        \
                                     PIN_PUPDR_PULLDOWN(15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(0) |  \
                                     PIN_ODR_LOW(1) |             \
                                     PIN_ODR_LOW(2) |             \
                                     PIN_ODR_LOW(3) |          \
                                     PIN_ODR_LOW(4) |             \
                                     PIN_ODR_LOW(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_LOW(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |             \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |             \
                                     PIN_ODR_LOW(13) |            \
                                     PIN_ODR_LOW(14) |            \
                                     PIN_ODR_LOW(15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0, 0) |\
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |        \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 8) |           \
                                     PIN_AFIO_AF(7, 8))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 6) |           \
                                     PIN_AFIO_AF(11, 6) |          \
                                     PIN_AFIO_AF(12, 6) |           \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port D setup:
 * PD0  - Alternate Digital input. (CAN1_RX).
 * PD1  - Alternate Open Drain output 50MHz (CAN1_TX).
 * PD2  - Digital input (CARD_DETECT).
 * PD3  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD4  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD5  - Alternate Push Pull output 50MHz. (UART2_TX).
 * PD6  - Alternate Digital input. (UART2_RX).
 * PD7  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD8  - Alternate Push Pull output 50MHz. (UART3_TX).
 * PD9  - Alternate Digital input. (UART3_RX).
 * PD10 - Digital output. (DOUT_LS1).
 * PD11 - Digital output. (DOUT_LS2).
 * PD12 - Digital output. (DOUT_LS3).
 * PD13 - Digital output. (DOUT_LS4).
 * PD14 - Digital output. (DOUT_HS1).
 * PD15 - Digital output. (DOUT_HS2).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(0) |           \
                                     PIN_MODE_ALTERNATE(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |         \
                                     PIN_MODE_ALTERNATE(5) |   \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |           \
                                     PIN_MODE_OUTPUT(10) |          \
                                     PIN_MODE_OUTPUT(11) |          \
                                     PIN_MODE_OUTPUT(12) |          \
                                     PIN_MODE_OUTPUT(13) |          \
                                     PIN_MODE_OUTPUT(14) |          \
                                     PIN_MODE_OUTPUT(15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |      \
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |       \
                                     PIN_OTYPE_PUSHPULL(13) |       \
                                     PIN_OTYPE_PUSHPULL(14) |       \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_50M(0) |          \
                                     PIN_OSPEED_50M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |         \
                                     PIN_OSPEED_50M(5) |  \
                                     PIN_OSPEED_50M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_50M(8) |          \
                                     PIN_OSPEED_50M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |          \
                                     PIN_OSPEED_100M(13) |          \
                                     PIN_OSPEED_100M(14) |          \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(0) |         \
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |         \
                                     PIN_PUPDR_FLOATING(4) |      \
                                     PIN_PUPDR_FLOATING(5) |\
                                     PIN_PUPDR_FLOATING(6) |         \
                                     PIN_PUPDR_FLOATING(7) |         \
                                     PIN_PUPDR_FLOATING(8) |         \
                                     PIN_PUPDR_FLOATING(9) |         \
                                     PIN_PUPDR_FLOATING(10) |        \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |       \
                                     PIN_PUPDR_FLOATING(13) |       \
                                     PIN_PUPDR_FLOATING(14) |       \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_LOW(3) |             \
                                     PIN_ODR_LOW(4) |            \
                                     PIN_ODR_HIGH(5) |     \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_LOW(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_LOW(10) |            \
                                     PIN_ODR_LOW(11) |            \
                                     PIN_ODR_LOW(12) |              \
                                     PIN_ODR_LOW(13) |              \
                                     PIN_ODR_LOW(14) |              \
                                     PIN_ODR_LOW(15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0, 9) |           \
                                     PIN_AFIO_AF(1, 9) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |          \
                                     PIN_AFIO_AF(5, 7) |   \
                                     PIN_AFIO_AF(6, 7) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8, 7) |           \
                                     PIN_AFIO_AF(9, 7) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |           \
                                     PIN_AFIO_AF(13, 0) |           \
                                     PIN_AFIO_AF(14, 0) |           \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port E setup:
 * PE0  - Digital input                    (IMU_MAG_DRDY)
 * PE1  - Digital input                    (IMU_ACC_DRDY)
 * PE2  - Digital input                    (IMU_GYRO_DRDY)
 * PE3  - Push Pull output 50MHz.          (IMU_GYRO_SS)
 * PE4  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PE5  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PE6  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PE7  - Digital input. (DIN1).
 * PE8  - Digital input. (DIN2).
 * PE9  - Digital input. (DIN3).
 * PE10 - Digital input. (DIN4).
 * PE11 - Digital input. (DIN5).
 * PE12 - Digital input. (DIN6).
 * PE13 - Digital input. (DIN7).
 * PE14 - Digital input. (DIN8).
 * PE15 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_OUTPUT(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |     \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_50M(3) |        \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |     \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |           \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |         \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOF setup:
 *
 * PF0  - PF15                      (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |       \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOG setup:
 *
 * PG0  - PG15                      (input floating).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |       \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOH setup:
 *
 * PH0  - PH15                      (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |       \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOI setup:
 *
 * PI0  - PI15                      (input floating).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |       \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))


/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
// no LEDS here

/*
 * ADCs
 */
// AIN1 enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN10
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO0
#endif

// AIN2 enabled by default
#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN11
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO1
#endif

// AIN3 enabled by default
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN12
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO2
#endif

// AIN4 enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN13
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO3
#endif


// Internal Temperature sensor enabled by default
/*
#ifndef USE_ADC_5
#define USE_ADC_5 1
#define USE_ADC_SENSOR 1
#endif
#if USE_ADC_5
#define AD1_5_CHANNEL ADC_CHANNEL_SENSOR
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOC // dummy pin
#define ADC_5_GPIO_PIN GPIO3 // dummy pin
#endif
*/

// Add board defines
#define AIN_1 AD1_1_CHANNEL
#define AIN_2 AD1_2_CHANNEL
#define AIN_3 AD1_3_CHANNEL
#define AIN_4 AD1_4_CHANNEL

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
//#ifndef ADC_CHANNEL_VSUPPLY
//#define ADC_CHANNEL_VSUPPLY ADC_1
//#endif

//#define ADC_CHANNEL_CURRENT ADC_2

#define DefaultVoltageOfAdc(adc) (0.000805664*adc)//(0.004489*adc)
#define DefaultMilliAmpereOfAdc(adc) (0.004489*adc)

/*
 * PWM defines
 * no PWM outputs used
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
#define PWM_SERVO_1_DRIVER PWMD3
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
#define PWM_SERVO_2_AF GPIO_AF3
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


/**
 * PPM radio defines
 * no ppm inputs used
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_3
#define PPM_TIMER ICUD1

/**
 * I2C defines
 * No I2C devices used
 */
#define I2C1_CLOCK_SPEED 400000
#define I2C1_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

#define I2C2_CLOCK_SPEED 400000
#define I2C2_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

/**
 * SPI Config
 */
// SLAVE0  - unconnected
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN 9
// SLAVE1  - unconnected
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN 1
// SLAVE2 is ASPIRIN MPU600 CS
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN 12
// SLAVE3 is BARO_CS
#define SPI_SELECT_SLAVE3_PORT GPIOE
#define SPI_SELECT_SLAVE3_PIN 3
// SLAVE4 - unconnected
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN 2

/**
 * SD card
 * TODO: add defines for SD log
 */

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 0
#endif

/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

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
