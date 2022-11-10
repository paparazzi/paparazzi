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
#define BOARD_ST_STM32F4_LISA
#define BOARD_NAME                  "STMicroelectronics STM32F4-Lisa"


/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                12000000
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
 * PA0  - Alternate Push Pull output 50MHz (SERVO5-Timer5Ch1)
 * PA1  - Alternate Push Pull output 50MHz (SERVO6-Timer5Ch2)
 * PA2  - Alternate Push Pull output 50MHz (UART2_TX)
 * PA3  - Digital input                    (UART2_RX)
 * PA4  - Alternate Push Pull output 50MHz (EXTSPI1_SS)
 * PA5  - Alternate Push Pull output 50MHz (EXTSPI1_SCK)
 * PA6  - Digital input.                   (EXTSPI1_MISO)
 * PA7  - Alternate Push Pull output 50MHz (EXTSPI1_MOSI)
 * PA8  - Open Drain output 50MHz          (LED1)
 * PA9  - Digital input.                   (USB_VBUS)
 * PA10 - Digital input.                   (UART1_Rx / PPM_IN Timer1Ch3)
 * PA11 - Alternate input                  (USB_DM)
 * PA12 - Alternate input                  (USB_DP)
 * PA13 - Digital input                    (JTAG_TMS/SWDIO)
 * PA14 - Digital input                    (JTAG_TCK/SWCLCK)
 * PA15 - Digital input                    (JTAG_TDI)
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(0)| \
                                     PIN_MODE_ALTERNATE(1) | \
                                     PIN_MODE_ALTERNATE(2) | \
                                     PIN_MODE_ALTERNATE(3)     | \
                                     PIN_MODE_ALTERNATE(4) | \
                                     PIN_MODE_ALTERNATE(5) | \
                                     PIN_MODE_ALTERNATE(6)     | \
                                     PIN_MODE_ALTERNATE(7) | \
                                     PIN_MODE_OUTPUT(8)    | \
                                     PIN_MODE_INPUT(9)     | \
                                     PIN_MODE_ALTERNATE(10)    | \
                                     PIN_MODE_ALTERNATE(11)    | \
                                     PIN_MODE_ALTERNATE(12)    | \
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
                                     PIN_OTYPE_OPENDRAIN(8) | \
                                     PIN_OTYPE_PUSHPULL(9)  | \
                                     PIN_OTYPE_PUSHPULL(10) | \
                                     PIN_OTYPE_PUSHPULL(11) | \
                                     PIN_OTYPE_PUSHPULL(12) | \
                                     PIN_OTYPE_PUSHPULL(13) | \
                                     PIN_OTYPE_PUSHPULL(14) | \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_50M(0) | \
                                     PIN_OSPEED_50M(1)  | \
                                     PIN_OSPEED_50M(2)  | \
                                     PIN_OSPEED_50M(3)  | \
                                     PIN_OSPEED_50M(4)  | \
                                     PIN_OSPEED_50M(5)  | \
                                     PIN_OSPEED_50M(6)  | \
                                     PIN_OSPEED_50M(7)  | \
                                     PIN_OSPEED_50M(8)  | \
                                     PIN_OSPEED_50M(9)  | \
                                     PIN_OSPEED_50M(10) | \
                                     PIN_OSPEED_50M(11) | \
                                     PIN_OSPEED_50M(12) | \
                                     PIN_OSPEED_100M(13) | \
                                     PIN_OSPEED_100M(14) | \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(0) | \
                                     PIN_PUPDR_FLOATING(1)    | \
                                     PIN_PUPDR_FLOATING(2)    | \
                                     PIN_PUPDR_FLOATING(3)    | \
                                     PIN_PUPDR_FLOATING(4)  | \
                                     PIN_PUPDR_FLOATING(5)  | \
                                     PIN_PUPDR_FLOATING(6)  | \
                                     PIN_PUPDR_FLOATING(7)  | \
                                     PIN_PUPDR_FLOATING(8)  | \
                                     PIN_PUPDR_FLOATING(9)  | \
                                     PIN_PUPDR_FLOATING(10) | \
                                     PIN_PUPDR_FLOATING(11) | \
                                     PIN_PUPDR_FLOATING(12) | \
                                     PIN_PUPDR_FLOATING(13) | \
                                     PIN_PUPDR_FLOATING(14) | \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(0) | \
                                     PIN_ODR_HIGH(1)  | \
                                     PIN_ODR_HIGH(2)  | \
                                     PIN_ODR_HIGH(3)  | \
                                     PIN_ODR_HIGH(4)  | \
                                     PIN_ODR_HIGH(5)  | \
                                     PIN_ODR_HIGH(6)  | \
                                     PIN_ODR_HIGH(7)  | \
                                     PIN_ODR_HIGH(8)  | \
                                     PIN_ODR_HIGH(9)  | \
                                     PIN_ODR_HIGH(10) | \
                                     PIN_ODR_HIGH(11) | \
                                     PIN_ODR_HIGH(12) | \
                                     PIN_ODR_HIGH(13) | \
                                     PIN_ODR_HIGH(14) | \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0, 2) | \
                                     PIN_AFIO_AF(1, 2)  | \
                                     PIN_AFIO_AF(2, 7)  | \
                                     PIN_AFIO_AF(3, 7)  | \
                                     PIN_AFIO_AF(4, 5)  | \
                                     PIN_AFIO_AF(5, 5)  | \
                                     PIN_AFIO_AF(6, 5)  | \
                                     PIN_AFIO_AF(7, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8, 0)  | \
                                     PIN_AFIO_AF(9, 0)   | \
                                     PIN_AFIO_AF(10, 1) | \
                                     PIN_AFIO_AF(11, 10) | \
                                     PIN_AFIO_AF(12, 10) | \
                                     PIN_AFIO_AF(13, 0)  | \
                                     PIN_AFIO_AF(14, 0)  | \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port B setup:
 * PB0  - Digital input                    (BARO_DRDY)
 * PB1  - Digital input                    (EXTSPI1_DRDY)
 * PB2  - Digital input                    (IMU_ACC_DRDY)
 * PB3  - Digital input                    (JTAG_TDO/SWD)
 * PB4  - Open Drain output 50MHz          (LED2)
 * PB5  - Digital input                    (IMU_MAG_DRDY)
 * PB6  - Alternate Push Pull output 50MHz (SERVO7-Timer4Ch1)/USART1_TX
 * PB7  - Alternate Push Pull output 50MHz (SERVO8-Timer4Ch2)/USART1_RX
 * PB8  - Digital input.                   (CAN_RX)
 * PB9  - Open Drain output 50MHz.         (CAN_TX)
 * PB10 - Alternate Open Drain output 2MHz.(I2C2_SCL)
 * PB11 - Alternate Open Drain output 2MHz.(I2C2_SDA)
 * PB12 - Push Pull output 50MHz.          (IMU_ACC_SPI2_CS)
 * PB13 - Alternate Push Pull output 50MHz (IMU_SPI2_SCK)
 * PB14 - Digital input                    (IMU_SPI2_MISO)
 * PB15 - Alternate Push Pull output 50MHz (IMU_SPI_MOSI)
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_ALTERNATE(3) |        \
                                     PIN_MODE_OUTPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_ALTERNATE(6) |        \
                                     PIN_MODE_ALTERNATE(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |        \
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
                                     PIN_OTYPE_OPENDRAIN(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_OPENDRAIN(9) |       \
                                     PIN_OTYPE_OPENDRAIN(10) |     \
                                     PIN_OTYPE_OPENDRAIN(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_50M(0) |          \
                                     PIN_OSPEED_50M(1) |          \
                                     PIN_OSPEED_50M(2) |          \
                                     PIN_OSPEED_100M(3) |           \
                                     PIN_OSPEED_50M(4) |          \
                                     PIN_OSPEED_50M(5) |          \
                                     PIN_OSPEED_50M(6) |           \
                                     PIN_OSPEED_50M(7) |          \
                                     PIN_OSPEED_50M(8) |          \
                                     PIN_OSPEED_50M(9) |           \
                                     PIN_OSPEED_2M(10) |        \
                                     PIN_OSPEED_2M(11) |         \
                                     PIN_OSPEED_50M(12) |         \
                                     PIN_OSPEED_50M(13) |         \
                                     PIN_OSPEED_50M(14) |         \
                                     PIN_OSPEED_50M(15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(0) |         \
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |        \
                                     PIN_PUPDR_FLOATING(4) |         \
                                     PIN_PUPDR_FLOATING(5) |         \
                                     PIN_PUPDR_FLOATING(6) |        \
                                     PIN_PUPDR_FLOATING(7) |         \
                                     PIN_PUPDR_FLOATING(8) |         \
                                     PIN_PUPDR_FLOATING(9) |        \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |        \
                                     PIN_PUPDR_FLOATING(13) |        \
                                     PIN_PUPDR_FLOATING(14) |        \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |              \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |              \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |              \
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
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 7) |            \
                                     PIN_AFIO_AF(7, 7))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8, 9) |           \
                                     PIN_AFIO_AF(9, 9) |            \
                                     PIN_AFIO_AF(10, 4) |         \
                                     PIN_AFIO_AF(11, 4) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 5) |          \
                                     PIN_AFIO_AF(14, 5) |          \
                                     PIN_AFIO_AF(15, 5))

/*
 * Port C setup:
 * PC0  - Analog input                     (ADC2)
 * PC1  - Analog input                     (ADC3)
 * PC2  - Open Drain output 50MHz           (LED3)
 * PC3  - Analog input                     (ADC1)
 * PC4  - Analog input                     (VBAT_MEAS)
 * PC5  - Open Drain output 50MHz           (LED4)
 * PC6  - Alternate Push Pull output 50MHz (SERVO1-Timer3Ch1)
 * PC7  - Alternate Push Pull output 50MHz (SERVO2-Timer3Ch2)
 * PC8  - Alternate Push Pull output 50MHz (SERVO3-Timer3Ch3)
 * PC9  - Alternate Push Pull output 50MHz (SERVO4-Timer3Ch4)
 * PC10 - Alternate Push Pull output 50MHz (UART3_TX)
 * PC11 - Digital input                    (UART3_RX)
 * PC12 - Alternate Push Pull output 50MHz (PC12-UART5_TX)
 * PC13 - Push Pull output 50MHz.          (IMU_GYRO_SS-Baro_SS_SPI2)
 * PC14 - Digital input                    (IMU_GYRO_DRDY)
 * PC15 - Open Drain output 50MHz          (LED5)
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_OUTPUT(2) |           \
                                     PIN_MODE_INPUT(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_OUTPUT(5) |           \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_ALTERNATE(7) |       \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |           \
                                     PIN_MODE_ALTERNATE(10) |       \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_ALTERNATE(12) |       \
                                     PIN_MODE_OUTPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_OUTPUT(15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_OPENDRAIN(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |    \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_OPENDRAIN(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |       \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |       \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_OPENDRAIN(15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_50M(0) |\
                                     PIN_OSPEED_50M(1) |          \
                                     PIN_OSPEED_50M(2) |          \
                                     PIN_OSPEED_50M(3) |       \
                                     PIN_OSPEED_50M(4) |          \
                                     PIN_OSPEED_50M(5) |          \
                                     PIN_OSPEED_50M(6) |          \
                                     PIN_OSPEED_50M(7) |          \
                                     PIN_OSPEED_50M(8) |          \
                                     PIN_OSPEED_50M(9) |          \
                                     PIN_OSPEED_50M(10) |          \
                                     PIN_OSPEED_50M(11) |         \
                                     PIN_OSPEED_50M(12) |          \
                                     PIN_OSPEED_50M(13) |         \
                                     PIN_OSPEED_50M(14) |         \
                                     PIN_OSPEED_50M(15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0) |\
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |      \
                                     PIN_PUPDR_FLOATING(4) |         \
                                     PIN_PUPDR_FLOATING(5) |         \
                                     PIN_PUPDR_FLOATING(6) |         \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |         \
                                     PIN_PUPDR_FLOATING(9) |         \
                                     PIN_PUPDR_FLOATING(10) |       \
                                     PIN_PUPDR_FLOATING(11) |        \
                                     PIN_PUPDR_FLOATING(12) |       \
                                     PIN_PUPDR_FLOATING(13) |        \
                                     PIN_PUPDR_FLOATING(14) |        \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(0) |  \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |          \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |             \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |             \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0, 0) |\
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |        \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 2) |           \
                                     PIN_AFIO_AF(7, 2))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8, 2) |           \
                                     PIN_AFIO_AF(9, 2) |           \
                                     PIN_AFIO_AF(10, 7) |           \
                                     PIN_AFIO_AF(11, 7) |          \
                                     PIN_AFIO_AF(12, 8) |           \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port D setup:
 * PD0  - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_IN).
 * PD1  - Digital input with PullUp or PullDown resistor depending on ODR. (OSC_OUT).
 * PD2  - Digital input (UART5_RX).
 * PD3  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD4  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD5  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD6  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD7  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD8  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD9  - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD10 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD11 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD12 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD13 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD14 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 * PD15 - Digital input with PullUp or PullDown resistor depending on ODR. (unconnected).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_ALTERNATE(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |         \
                                     PIN_MODE_INPUT(5) |   \
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
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_50M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |         \
                                     PIN_OSPEED_100M(5) |  \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
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
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |            \
                                     PIN_ODR_HIGH(5) |     \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |              \
                                     PIN_ODR_HIGH(13) |              \
                                     PIN_ODR_HIGH(14) |              \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 8) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |          \
                                     PIN_AFIO_AF(5, 0) |   \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |           \
                                     PIN_AFIO_AF(13, 0) |           \
                                     PIN_AFIO_AF(14, 0) |           \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port E setup.
 * PE0 - PE15  - floating input
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |        \
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
                                     PIN_OSPEED_100M(3) |        \
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
/* 1 red, on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN 8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* 2 green, shared with JTAG_TRST */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN 4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/* 3 green, shared with ADC12 (ADC_6 on connector ANALOG2) */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN 2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* 4 red, shared with ADC15 (ADC_4 on connector ANALOG2) */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN 5
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set

/* 5 green, on PC15 */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOC
#define LED_5_GPIO_PIN 15
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// AUX 1
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN13
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO3
#endif

// AUX 2
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN10
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO0
#endif

// AUX 3
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN11
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO1
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL ADC_CHANNEL_IN14
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO4
#endif

// Internal Temperature sensor enabled by default
#ifndef USE_ADC_5
#define USE_ADC_5 1
#define USE_ADC_SENSOR 1
#endif
#if USE_ADC_5
#define AD1_5_CHANNEL ADC_CHANNEL_SENSOR
#define ADC_5 AD1_5
#define ADC_5_GPIO_PORT GPIOC
#define ADC_5_GPIO_PIN GPIO4
#endif



/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define DefaultVoltageOfAdc(adc) (0.004489*adc)

/*
 * PWM defines
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOC
#define PWM_SERVO_0_PIN GPIO6
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_DRIVER PWMD3
#define PWM_SERVO_0_CHANNEL 0
#define PWM_SERVO_0_CONF pwmcfg3
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO7
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_DRIVER PWMD3
#define PWM_SERVO_1_CHANNEL 1
#define PWM_SERVO_1_CONF pwmcfg3
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO8
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_DRIVER PWMD3
#define PWM_SERVO_2_CHANNEL 2
#define PWM_SERVO_2_CONF pwmcfg3
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO9
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD3
#define PWM_SERVO_3_CHANNEL 3
#define PWM_SERVO_3_CONF pwmcfg3
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO0
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_DRIVER PWMD5
#define PWM_SERVO_4_CHANNEL 0
#define PWM_SERVO_4_CONF pwmcfg5
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO1
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_DRIVER PWMD5
#define PWM_SERVO_5_CHANNEL 1
#define PWM_SERVO_5_CONF pwmcfg5
#endif


#if USE_SERVOS_7AND8
  #if USE_I2C1
    #error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
  #else /* !USE_I2C1 */
    #if USE_PWM6
    #define PWM_SERVO_6 6
    #define PWM_SERVO_6_GPIO GPIOB
    #define PWM_SERVO_6_PIN GPIO6
    #define PWM_SERVO_6_AF GPIO_AF2
    #define PWM_SERVO_6_DRIVER PWMD4
    #define PWM_SERVO_6_CHANNEL 0
    #define PWM_SERVO_6_CONF pwmcfg4
    #endif

    #if USE_PWM7
    #define PWM_SERVO_7 7
    #define PWM_SERVO_7_GPIO GPIOB
    #define PWM_SERVO_7_PIN GPIO7
    #define PWM_SERVO_7_AF GPIO_AF2
    #define PWM_SERVO_7_DRIVER PWMD4
    #define PWM_SERVO_7_CHANNEL 1
    #define PWM_SERVO_7_CONF pwmcfg4
    #endif
  #endif /* USE_I2C1 */
#endif /* USE_SERVOS_7AND8 */


/**
 * PPM radio defines
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD1

/**
 * I2C defines
 */
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 6U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 7U)
#define I2C1_CLOCK_SPEED 400000
#define I2C1_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define I2C2_CLOCK_SPEED 400000
#define I2C2_CFG_DEF {       \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           FAST_DUTY_CYCLE_2, \
           }

/**
 * SPI Config
 */
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

#define SPI3_GPIO_PORT_NSS GPIO

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
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
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
