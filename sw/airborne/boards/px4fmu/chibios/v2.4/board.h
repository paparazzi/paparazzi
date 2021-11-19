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
 * Setup for Pixhawk PX4FMU_2.4 board 
 * Note the MCU used is STM32F427VIT6
 */

/*
 * Board identifier.
 */
#define BOARD_PX4FMU_v2
#define BOARD_NAME  "Pixhawk PX4 FMU v 2.4"

/*
 * Board oscillators-related settings.
 * NOTE: LSE NOT? fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0 // originally was 32000 ?
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000
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
 * PA0  - Alternate Push Pull output 50MHz (UART4_TX)
 * PA1  - Alternate Digital input          (UART4_RX)
 * PA2  - Analog input                     (BATT VOLTAGE SENS - ADC in)
 * PA3  - Analog input                     (BATT CURRENT SENS - ADC in)
 * PA4  - Analog input                     (VDD 5V SENS - ADC in)
 * PA5  - Alternate Push Pull output 50MHz (EXTSPI1_SCK)
 * PA6  - Alternate Digital input.         (EXTSPI1_MISO)
 * PA7  - Alternate Push Pull output 50MHz (EXTSPI1_MOSI)
 * PA8  - Digital input                    (!VDD_5V_PERIPH_EN)
 * PA9  - Digital input.                   (USB_VBUS)
 * PA10 - Alternate Digital input          (UART1_Rx)
 * PA11 - Alternate input                  (USB_DM)
 * PA12 - Alternate input                  (USB_DP)
 * PA13 - Alternate Digital input          (SWDIO)
 * PA14 - Alternate Digital input          (SWCLCK)
 * PA15 - Digital output                   (ALARM)
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(0)| \
                                     PIN_MODE_ALTERNATE(1) | \
                                     PIN_MODE_ANALOG(2) | \
                                     PIN_MODE_ANALOG(3)     | \
                                     PIN_MODE_ANALOG(4) | \
                                     PIN_MODE_ALTERNATE(5) | \
                                     PIN_MODE_ALTERNATE(6)     | \
                                     PIN_MODE_ALTERNATE(7) | \
                                     PIN_MODE_INPUT(8)    | \
                                     PIN_MODE_INPUT(9)     | \
                                     PIN_MODE_ALTERNATE(10)    | \
                                     PIN_MODE_ALTERNATE(11)    | \
                                     PIN_MODE_ALTERNATE(12)    | \
                                     PIN_MODE_ALTERNATE(13)    | \
                                     PIN_MODE_ALTERNATE(14)    | \
                                     PIN_MODE_OUTPUT(15))
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
                                     PIN_OTYPE_OPENDRAIN(15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(0) | \
                                     PIN_OSPEED_100M(1)  | \
                                     PIN_OSPEED_100M(2)  | \
                                     PIN_OSPEED_100M(3)  | \
                                     PIN_OSPEED_100M(4)  | \
                                     PIN_OSPEED_50M(5)  | \
                                     PIN_OSPEED_50M(6)  | \
                                     PIN_OSPEED_50M(7)  | \
                                     PIN_OSPEED_100M(8)  | \
                                     PIN_OSPEED_100M(9)  | \
                                     PIN_OSPEED_100M(10) | \
                                     PIN_OSPEED_100M(11) | \
                                     PIN_OSPEED_100M(12) | \
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
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0, 8) | \
                                     PIN_AFIO_AF(1, 8)  | \
                                     PIN_AFIO_AF(2, 0)  | \
                                     PIN_AFIO_AF(3, 0)  | \
                                     PIN_AFIO_AF(4, 0)  | \
                                     PIN_AFIO_AF(5, 5)  | \
                                     PIN_AFIO_AF(6, 5)  | \
                                     PIN_AFIO_AF(7, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8, 0)  | \
                                     PIN_AFIO_AF(9, 0)   | \
                                     PIN_AFIO_AF(10, 7) | \
                                     PIN_AFIO_AF(11, 10) | \
                                     PIN_AFIO_AF(12, 10) | \
                                     PIN_AFIO_AF(13, 0)  | \
                                     PIN_AFIO_AF(14, 0)  | \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port B setup:
 * PB0  - Digital input                    (GYRO_DRDY) #
 * PB1  - Digital input                    (MAG_DRDY) #
 * PB2  - Digital input                    (BOOT) # ?
 * PB3  - Digital input                    (JTAG_TDO/SWD) #
 * PB4  - Digital input                    (ACCEL_DRDY) #
 * PB5  - Digital input                    (!VDD_BRICK_VALID) #
 * PB6  - Alternate Push Pull output 50MHz (CAN2_TX) 
 * PB7  - Digital input                    (!VDD_SERVO_VALID)
 * PB8  - Alternate Open Drain output 50MHz (I2C1_SCL)
 * PB9  - Alternate Open Drain output 50MHz (I2C1_SDA)
 * PB10 - Alternate Open Drain output 50MHz (I2C2_SCL)
 * PB11 - Alternate Open Drain output 50MHz (I2C2_SDA)
 * PB12 - Alternate Push Pull output 50MHz (CAN2_RX)
 * PB13 - Alternate Push Pull output 50MHz (FRAM_SPI2_SCK) #
 * PB14 - Alternate Digital input          (FRAM_SPI2_MISO) #
 * PB15 - Alternate Push Pull output 50MHz (FRAM_SPI2_MOSI) #
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_ALTERNATE(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_ALTERNATE(6) |        \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |        \
                                     PIN_MODE_ALTERNATE(10) |         \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_ALTERNATE(12) |          \
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
                                     PIN_OTYPE_OPENDRAIN(8) |       \
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
                                     PIN_OSPEED_50M(10) |        \
                                     PIN_OSPEED_50M(11) |         \
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
                                     PIN_AFIO_AF(6, 9) |            \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(8, 4) |           \
                                     PIN_AFIO_AF(9, 4) |            \
                                     PIN_AFIO_AF(10, 4) |         \
                                     PIN_AFIO_AF(11, 4) |          \
                                     PIN_AFIO_AF(12, 9) |          \
                                     PIN_AFIO_AF(13, 5) |          \
                                     PIN_AFIO_AF(14, 5) |          \
                                     PIN_AFIO_AF(15, 5))

/*
 * Port C setup:
 * PC0  - Digital input                    (!VBUS_VALID) #
 * PC1  - Analog input                     (ADC3) #
 * PC2  - Push Pull output 50MHz.          (!MPU_CS) #
 * PC3  - Analog input                     (ADC1) #
 * PC4  - Analog input                     (ADC2) #
 * PC5  - Analog input                     (PRESSURE_SENS - ADC) #
 * PC6  - Alternate Push Pull output 50MHz (UART6-TX)
 * PC7  - Alternate Digital intput         (UART6_RX)
 * PC8  - Alternate pullup 100MHz          (SDIO_D0)
 * PC9  - Alternate pullup 100MHz          (SDIO_D1)
 * PC10 - Alternate pullup 100MHz          (SDIO_D2)
 * PC11 - Alternate pullup 100MHz          (SDIO_D3)
 * PC12 - Alternate floating 100MHz        (SDIO_CK)
 * PC13 - Push Pull output 50MHz.          (!GYRO_SS) #
 * PC14 - Digital input                    (GPIO_EXT_1)
 * PC15 - Push Pull output 50MHz.          (!ACCEL_MAG_SS)
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_OUTPUT(2) |           \
                                     PIN_MODE_INPUT(3) |        \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
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
                                     PIN_OSPEED_2M(13) |         \
                                     PIN_OSPEED_50M(14) |         \
                                     PIN_OSPEED_2M(15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0) |\
                                     PIN_PUPDR_FLOATING(1) |         \
                                     PIN_PUPDR_FLOATING(2) |         \
                                     PIN_PUPDR_FLOATING(3) |      \
                                     PIN_PUPDR_FLOATING(4) |         \
                                     PIN_PUPDR_FLOATING(5) |         \
                                     PIN_PUPDR_FLOATING(6) |         \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_PULLUP(8) |         \
                                     PIN_PUPDR_PULLUP(9) |         \
                                     PIN_PUPDR_PULLUP(10) |       \
                                     PIN_PUPDR_PULLUP(11) |        \
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
                                     PIN_AFIO_AF(6, 8) |           \
                                     PIN_AFIO_AF(7, 8))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8, 12) |           \
                                     PIN_AFIO_AF(9, 12) |           \
                                     PIN_AFIO_AF(10, 12) |           \
                                     PIN_AFIO_AF(11, 12) |          \
                                     PIN_AFIO_AF(12, 12) |           \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port D setup:
 * PD0  - CAN1_RX.
 * PD1  - CAN1_TX
 * PD2  - SDIO_CMD
 * PD3  - UART2_CTS
 * PD4  - UART2_RTS
 * PD5  - UART2_TX
 * PD6  - UART2_RX
 * PD7  - !BARO_CS
 * PD8  - UART3_TX
 * PD9  - UART3_RX
 * PD10 - !FRAM_CS
 * PD11 - UART3_CTS
 * PD12 - UART3_RTS
 * PD13 - SERVO_5
 * PD14 - SERVO_6
 * PD15 - MPU_DRDY
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(0) |           \
                                     PIN_MODE_ALTERNATE(1) |           \
                                     PIN_MODE_ALTERNATE(2) |           \
                                     PIN_MODE_ALTERNATE(3) |           \
                                     PIN_MODE_ALTERNATE(4) |         \
                                     PIN_MODE_ALTERNATE(5) |   \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_OUTPUT(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_ALTERNATE(9) |           \
                                     PIN_MODE_OUTPUT(10) |          \
                                     PIN_MODE_ALTERNATE(11) |          \
                                     PIN_MODE_ALTERNATE(12) |          \
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
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |         \
                                     PIN_OSPEED_100M(5) |  \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_2M(7) |          \
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
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0, 9) |           \
                                     PIN_AFIO_AF(1, 9) |           \
                                     PIN_AFIO_AF(2, 12) |           \
                                     PIN_AFIO_AF(3, 7) |           \
                                     PIN_AFIO_AF(4, 7) |          \
                                     PIN_AFIO_AF(5, 7) |   \
                                     PIN_AFIO_AF(6, 7) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8, 7) |           \
                                     PIN_AFIO_AF(9, 7) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 7) |          \
                                     PIN_AFIO_AF(12, 7) |           \
                                     PIN_AFIO_AF(13, 0) |           \
                                     PIN_AFIO_AF(14, 0) |           \
                                     PIN_AFIO_AF(15, 0))

/*
 * Port E setup:
 * PE0  - UART8_RX
 * PE1  - UART8_TX
 * PE2  - SPI4_EXT_SCK
 * PE3  - VDD_3V3_SENSOR_EN
 * PE4  - !SPI4_EXT_NSS
 * PE5  - SPI4_EXT_MISO
 * PE6  - SPI4_EXT_MOSI
 * PE7  - UART7_RX
 * PE8  - UART7_TX
 * PE9  - SERVO_CH4
 * PE10 - !VDD_HIPOWER_OC
 * PE11 - SERVO_CH3
 * PE12 - LED_AMBER
 * PE13 - SERVO_CH2
 * PE14 - SERVO_CH1
 * PE15 - !VDD_5V_PERIPH_OC
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(0) |           \
                                     PIN_MODE_ALTERNATE(1) |           \
                                     PIN_MODE_ALTERNATE(2) |           \
                                     PIN_MODE_OUTPUT(3) |        \
                                     PIN_MODE_ALTERNATE(4) |           \
                                     PIN_MODE_ALTERNATE(5) |           \
                                     PIN_MODE_ALTERNATE(6) |           \
                                     PIN_MODE_ALTERNATE(7) |           \
                                     PIN_MODE_ALTERNATE(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_OUTPUT(12) |          \
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
                                     PIN_OTYPE_OPENDRAIN(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_50M(2) |          \
                                     PIN_OSPEED_100M(3) |        \
                                     PIN_OSPEED_50M(4) |          \
                                     PIN_OSPEED_50M(5) |          \
                                     PIN_OSPEED_50M(6) |          \
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
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(0, 8) |           \
                                     PIN_AFIO_AF(1, 8) |           \
                                     PIN_AFIO_AF(2, 5) |           \
                                     PIN_AFIO_AF(3, 0) |         \
                                     PIN_AFIO_AF(4, 5) |           \
                                     PIN_AFIO_AF(5, 5) |           \
                                     PIN_AFIO_AF(6, 5) |           \
                                     PIN_AFIO_AF(7, 8))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(8, 8) |           \
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
*/
#define VAL_GPIOH_MODER             0x00000000
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0x00000000
#define VAL_GPIOH_PUPDR             0x55555555 // all pullup
#define VAL_GPIOH_ODR               0xFFFFFFFF
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

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
 * Onboard LEDs
 */
/* red, on PE12 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOE
#define LED_1_GPIO_PIN GPIO12
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/*
 * ADCs TODO
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
 * PWM defines TODO
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
 * PPM radio defines TODO
 */
#define RC_PPM_TICKS_PER_USEC 2
#define PPM_TIMER_FREQUENCY 2000000
#define PPM_CHANNEL ICU_CHANNEL_1
#define PPM_TIMER ICUD1

/*
 * PWM input TODO
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
#define PWM_INPUT2_ICU            ICUD2
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT2_GPIO_PORT      GPIOA
#define PWM_INPUT2_GPIO_PIN       GPIO2
#define PWM_INPUT2_GPIO_AF        GPIO_AF3


/**
 * I2C defines TODO: getting DMA failiure -> check timers and DMA mapping
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
 * SPI1 si for sensors
 * SPI2 is for FRAM
 * SPI4 is external
 */

// SPI1_SLAVE0 -> slave select pin for the L3GD20 (gyro)
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO13
// SPI1_SLAVE1 -> slave select pin for the LSM303D (accel/mag)
#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO15
// SPI1_SLAVE3 -> slave select pin for the MS5611 baro
#define SPI_SELECT_SLAVE3_PORT GPIOD
#define SPI_SELECT_SLAVE3_PIN GPIO7

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/**
 *
 * Kludge for SD_LOG
 */
#define GPIOA_OTG_FS_VBUS           9

#define GPIOD_SDIO_CMD               2
#define GPIOC_SDIO_D0                8
#define GPIOC_SDIO_D1                9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CK               12

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
