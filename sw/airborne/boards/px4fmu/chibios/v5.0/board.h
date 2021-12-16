/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#pragma once

/*
 * Board identifier.
 */
#define BOARD_PX4FMU
#define BOARD_NAME                  "Pixhawk PX4 FMU v5.0"

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
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F767xx

/*
 * IO pins assignments.
 */
#define	ADC1                           0U
#define	ADC2                           1U
#define	ADC3                           2U
#define	ADC4                           3U
#define	ADC5                           4U
#define	FMU_CAP1                       5U
#define	SPI1_MISO                      6U
#define	HEATER                         7U
#define	CAN3_RX                        8U
#define	USB_VBUS                       9U
#define	SERVO2                         10U
#define	USB_DM                         11U
#define	USB_DP                         12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	CAN3_TX                        15U

#define	RSSI_IN                        0U
#define	LED1                           1U
#define	PB02                           2U
#define	FMU_CAP2                       3U
#define	DRDY1_ICM20689                 4U
#define	SPI6_MOSI                      5U
#define	USART1_TX                      6U
#define	USART1_RX                      7U
#define	I2C1_SCL                       8U
#define	I2C1_SDA                       9U
#define	SPI5_RESET                     10U
#define	FMU_CAP3                       11U
#define	CAN2_RX                        12U
#define	CAN2_TX                        13U
#define	DRDY2_BMI055_GYRO              14U
#define	DRDY2_BMI055_ACC               15U

#define	SCALED_V5                      0U
#define	SCALED_3V3_SENSORS             1U
#define	HW_VER_SENSE                   2U
#define	HW_REV_SENSE                   3U
#define	ADC6                           4U
#define	DRDY4_ICM20602                 5U
#define	LED2                           6U
#define	LED3                           7U
#define	SDIO_D0                        8U
#define	SDIO_D1                        9U
#define	SDIO_D2                        10U
#define	SDIO_D3                        11U
#define	SDIO_CK                        12U
#define	DRDY5_BMI055_GYRO              13U
#define	OSC32_IN                       14U
#define	OSC32_OUT                      15U

#define	UART4_RX                       0U
#define	UART4_TX                       1U
#define	SDIO_CMD                       2U
#define	UART2_CTS                      3U
#define	UART2_RTS                      4U
#define	UART2_TX                       5U
#define	UART2_RX                       6U
#define	SPI1_MOSI                      7U
#define	UART3_TX                       8U
#define	UART3_RX                       9U
#define	DRDY6_BMI055_ACC               10U
#define	UART3_CTS                      11U
#define	UART3_RTS                      12U
#define	SERVO5                         13U
#define	SERVO6                         14U
#define	DRDY7_EXTERNAL1                15U

#define	UART8_RX                       0U
#define	UART8_TX                       1U
#define	SPI4_SCK                       2U
#define	V3V3_SENSORS_EN                3U
#define	V3V3_SPEKTRUM_EN               4U
#define	BUZZER                         5U
#define	SPI4_MOSI                      6U
#define	DRDY8                          7U
#define	UART7_TX                       8U
#define	SERVO4                         9U
#define	SAFETY_SWITCH_IN               10U
#define	SERVO3                         11U
#define	LED4                           12U
#define	SPI4_MISO                      13U
#define	SERVO1                         14U
#define	V5V_PERIPH_OC                  15U

#define	I2C2_SDA                       0U
#define	I2C2_SCL                       1U
#define	SPI_SLAVE0                     2U
#define	SPI_SLAVE1                     3U
#define	SPI_SLAVE2                     4U
#define	SPI_SLAVE3                     5U
#define	UART7_RX                       6U
#define	SPI5_SCK                       7U
#define	SPI5_MISO                      8U
#define	SPI5_MOSI                      9U
#define	SPI_SLAVE4                     10U
#define	SPI_SLAVE5                     11U
#define	V5V_HIPOWER_EN                 12U
#define	V5V_HIPOWER_OC                 13U
#define	I2C4_SCL                       14U
#define	I2C4_SDA                       15U

#define	HW_VER_DRIVE                   0U
#define	POWER_IN_A                     1U
#define	POWER_IN_B                     2U
#define	POWER_IN_C                     3U
#define	V5V_PERIPH_EN                  4U
#define	V5V_RC_EN                      5U
#define	V5V_WIFI_EN                    6U
#define	V3V3_SD_CARD_EN                7U
#define	USART6_RTS                     8U
#define	USART6_RX                      9U
#define	SPI_SLAVE6                     10U
#define	SPI1_SCK                       11U
#define	SPI6_MISO                      12U
#define	SPI6_SCK                       13U
#define	USART6_TX                      14U
#define	USART6_CTS                     15U

#define	OSC_IN                         0U
#define	OSC_OUT                        1U
#define	CAN1_SILENT_S0                 2U
#define	CAN2_SILENT_S1                 3U
#define	CAN3_SILENT_S2                 4U
#define	SPI_SLAVE7                     5U
#define	SERVO7                         6U
#define	I2C3_SCL                       7U
#define	I2C3_SDA                       8U
#define	SERVO8                         9U
#define	LED5                           10U
#define	LED6                           11U
#define	LED7                           12U
#define	CAN1_TX                        13U
#define	HW_REV_DRIVE                   14U
#define	SPI5_SYNC                      15U

#define	ARMED                          0U
#define	SPI2_SCK                       1U
#define	SPI2_MISO                      2U
#define	SPI2_MOSI                      3U
#define	SPI_SLAVE8                     4U
#define	RC_INPUT                       5U
#define	SPI_SLAVE9                     6U
#define	SPI_SLAVE10                    7U
#define	SPI_SLAVE11                    8U
#define	CAN1_RX                        9U
#define	SPI_SLAVE12                    10U
#define	SPI_SLAVE13                    11U
#define	PI12                           12U
#define	PI13                           13U
#define	PI14                           14U
#define	PI15                           15U

#define	PJ00                           0U
#define	PJ01                           1U
#define	PJ02                           2U
#define	PJ03                           3U
#define	PJ04                           4U
#define	PJ05                           5U
#define	PJ06                           6U
#define	PJ07                           7U
#define	PJ08                           8U
#define	PJ09                           9U
#define	PJ10                           10U
#define	PJ11                           11U
#define	PJ12                           12U
#define	PJ13                           13U
#define	PJ14                           14U
#define	PJ15                           15U

#define	PK00                           0U
#define	PK01                           1U
#define	PK02                           2U
#define	PK03                           3U
#define	PK04                           4U
#define	PK05                           5U
#define	PK06                           6U
#define	PK07                           7U
#define	PK08                           8U
#define	PK09                           9U
#define	PK10                           10U
#define	PK11                           11U
#define	PK12                           12U
#define	PK13                           13U
#define	PK14                           14U
#define	PK15                           15U

/*
 * IO lines assignments.
 */
#define	LINE_ADC1                      PAL_LINE(GPIOA, 0U)
#define	LINE_ADC2                      PAL_LINE(GPIOA, 1U)
#define	LINE_ADC3                      PAL_LINE(GPIOA, 2U)
#define	LINE_ADC4                      PAL_LINE(GPIOA, 3U)
#define	LINE_ADC5                      PAL_LINE(GPIOA, 4U)
#define	LINE_FMU_CAP1                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_HEATER                    PAL_LINE(GPIOA, 7U)
#define	LINE_CAN3_RX                   PAL_LINE(GPIOA, 8U)
#define	LINE_USB_VBUS                  PAL_LINE(GPIOA, 9U)
#define	LINE_SERVO2                    PAL_LINE(GPIOA, 10U)
#define	LINE_USB_DM                    PAL_LINE(GPIOA, 11U)
#define	LINE_USB_DP                    PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_CAN3_TX                   PAL_LINE(GPIOA, 15U)

#define	LINE_RSSI_IN                   PAL_LINE(GPIOB, 0U)
#define	LINE_LED1                      PAL_LINE(GPIOB, 1U)
#define	LINE_FMU_CAP2                  PAL_LINE(GPIOB, 3U)
#define	LINE_DRDY1_ICM20689            PAL_LINE(GPIOB, 4U)
#define	LINE_SPI6_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_USART1_TX                 PAL_LINE(GPIOB, 6U)
#define	LINE_USART1_RX                 PAL_LINE(GPIOB, 7U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 9U)
#define	LINE_SPI5_RESET                PAL_LINE(GPIOB, 10U)
#define	LINE_FMU_CAP3                  PAL_LINE(GPIOB, 11U)
#define	LINE_CAN2_RX                   PAL_LINE(GPIOB, 12U)
#define	LINE_CAN2_TX                   PAL_LINE(GPIOB, 13U)
#define	LINE_DRDY2_BMI055_GYRO         PAL_LINE(GPIOB, 14U)
#define	LINE_DRDY2_BMI055_ACC          PAL_LINE(GPIOB, 15U)

#define	LINE_SCALED_V5                 PAL_LINE(GPIOC, 0U)
#define	LINE_SCALED_3V3_SENSORS        PAL_LINE(GPIOC, 1U)
#define	LINE_HW_VER_SENSE              PAL_LINE(GPIOC, 2U)
#define	LINE_HW_REV_SENSE              PAL_LINE(GPIOC, 3U)
#define	LINE_ADC6                      PAL_LINE(GPIOC, 4U)
#define	LINE_DRDY4_ICM20602            PAL_LINE(GPIOC, 5U)
#define	LINE_LED2                      PAL_LINE(GPIOC, 6U)
#define	LINE_LED3                      PAL_LINE(GPIOC, 7U)
#define	LINE_SDIO_D0                   PAL_LINE(GPIOC, 8U)
#define	LINE_SDIO_D1                   PAL_LINE(GPIOC, 9U)
#define	LINE_SDIO_D2                   PAL_LINE(GPIOC, 10U)
#define	LINE_SDIO_D3                   PAL_LINE(GPIOC, 11U)
#define	LINE_SDIO_CK                   PAL_LINE(GPIOC, 12U)
#define	LINE_DRDY5_BMI055_GYRO         PAL_LINE(GPIOC, 13U)
#define	LINE_OSC32_IN                  PAL_LINE(GPIOC, 14U)
#define	LINE_OSC32_OUT                 PAL_LINE(GPIOC, 15U)

#define	LINE_UART4_RX                  PAL_LINE(GPIOD, 0U)
#define	LINE_UART4_TX                  PAL_LINE(GPIOD, 1U)
#define	LINE_SDIO_CMD                  PAL_LINE(GPIOD, 2U)
#define	LINE_UART2_CTS                 PAL_LINE(GPIOD, 3U)
#define	LINE_UART2_RTS                 PAL_LINE(GPIOD, 4U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOD, 5U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOD, 6U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOD, 7U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOD, 8U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOD, 9U)
#define	LINE_DRDY6_BMI055_ACC          PAL_LINE(GPIOD, 10U)
#define	LINE_UART3_CTS                 PAL_LINE(GPIOD, 11U)
#define	LINE_UART3_RTS                 PAL_LINE(GPIOD, 12U)
#define	LINE_SERVO5                    PAL_LINE(GPIOD, 13U)
#define	LINE_SERVO6                    PAL_LINE(GPIOD, 14U)
#define	LINE_DRDY7_EXTERNAL1           PAL_LINE(GPIOD, 15U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE_SPI4_SCK                  PAL_LINE(GPIOE, 2U)
#define	LINE_V3V3_SENSORS_EN           PAL_LINE(GPIOE, 3U)
#define	LINE_V3V3_SPEKTRUM_EN          PAL_LINE(GPIOE, 4U)
#define	LINE_BUZZER                    PAL_LINE(GPIOE, 5U)
#define	LINE_SPI4_MOSI                 PAL_LINE(GPIOE, 6U)
#define	LINE_DRDY8                     PAL_LINE(GPIOE, 7U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOE, 8U)
#define	LINE_SERVO4                    PAL_LINE(GPIOE, 9U)
#define	LINE_SAFETY_SWITCH_IN          PAL_LINE(GPIOE, 10U)
#define	LINE_SERVO3                    PAL_LINE(GPIOE, 11U)
#define	LINE_LED4                      PAL_LINE(GPIOE, 12U)
#define	LINE_SPI4_MISO                 PAL_LINE(GPIOE, 13U)
#define	LINE_SERVO1                    PAL_LINE(GPIOE, 14U)
#define	LINE_V5V_PERIPH_OC             PAL_LINE(GPIOE, 15U)

#define	LINE_I2C2_SDA                  PAL_LINE(GPIOF, 0U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOF, 1U)
#define	LINE_SPI_SLAVE0                PAL_LINE(GPIOF, 2U)
#define	LINE_SPI_SLAVE1                PAL_LINE(GPIOF, 3U)
#define	LINE_SPI_SLAVE2                PAL_LINE(GPIOF, 4U)
#define	LINE_SPI_SLAVE3                PAL_LINE(GPIOF, 5U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOF, 6U)
#define	LINE_SPI5_SCK                  PAL_LINE(GPIOF, 7U)
#define	LINE_SPI5_MISO                 PAL_LINE(GPIOF, 8U)
#define	LINE_SPI5_MOSI                 PAL_LINE(GPIOF, 9U)
#define	LINE_SPI_SLAVE4                PAL_LINE(GPIOF, 10U)
#define	LINE_SPI_SLAVE5                PAL_LINE(GPIOF, 11U)
#define	LINE_V5V_HIPOWER_EN            PAL_LINE(GPIOF, 12U)
#define	LINE_V5V_HIPOWER_OC            PAL_LINE(GPIOF, 13U)
#define	LINE_I2C4_SCL                  PAL_LINE(GPIOF, 14U)
#define	LINE_I2C4_SDA                  PAL_LINE(GPIOF, 15U)

#define	LINE_HW_VER_DRIVE              PAL_LINE(GPIOG, 0U)
#define	LINE_POWER_IN_A                PAL_LINE(GPIOG, 1U)
#define	LINE_POWER_IN_B                PAL_LINE(GPIOG, 2U)
#define	LINE_POWER_IN_C                PAL_LINE(GPIOG, 3U)
#define	LINE_V5V_PERIPH_EN             PAL_LINE(GPIOG, 4U)
#define	LINE_V5V_RC_EN                 PAL_LINE(GPIOG, 5U)
#define	LINE_V5V_WIFI_EN               PAL_LINE(GPIOG, 6U)
#define	LINE_V3V3_SD_CARD_EN           PAL_LINE(GPIOG, 7U)
#define	LINE_USART6_RTS                PAL_LINE(GPIOG, 8U)
#define	LINE_USART6_RX                 PAL_LINE(GPIOG, 9U)
#define	LINE_SPI_SLAVE6                PAL_LINE(GPIOG, 10U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOG, 11U)
#define	LINE_SPI6_MISO                 PAL_LINE(GPIOG, 12U)
#define	LINE_SPI6_SCK                  PAL_LINE(GPIOG, 13U)
#define	LINE_USART6_TX                 PAL_LINE(GPIOG, 14U)
#define	LINE_USART6_CTS                PAL_LINE(GPIOG, 15U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)
#define	LINE_CAN1_SILENT_S0            PAL_LINE(GPIOH, 2U)
#define	LINE_CAN2_SILENT_S1            PAL_LINE(GPIOH, 3U)
#define	LINE_CAN3_SILENT_S2            PAL_LINE(GPIOH, 4U)
#define	LINE_SPI_SLAVE7                PAL_LINE(GPIOH, 5U)
#define	LINE_SERVO7                    PAL_LINE(GPIOH, 6U)
#define	LINE_I2C3_SCL                  PAL_LINE(GPIOH, 7U)
#define	LINE_I2C3_SDA                  PAL_LINE(GPIOH, 8U)
#define	LINE_SERVO8                    PAL_LINE(GPIOH, 9U)
#define	LINE_LED5                      PAL_LINE(GPIOH, 10U)
#define	LINE_LED6                      PAL_LINE(GPIOH, 11U)
#define	LINE_LED7                      PAL_LINE(GPIOH, 12U)
#define	LINE_CAN1_TX                   PAL_LINE(GPIOH, 13U)
#define	LINE_HW_REV_DRIVE              PAL_LINE(GPIOH, 14U)
#define	LINE_SPI5_SYNC                 PAL_LINE(GPIOH, 15U)

#define	LINE_ARMED                     PAL_LINE(GPIOI, 0U)
#define	LINE_SPI2_SCK                  PAL_LINE(GPIOI, 1U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOI, 2U)
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOI, 3U)
#define	LINE_SPI_SLAVE8                PAL_LINE(GPIOI, 4U)
#define	LINE_RC_INPUT                  PAL_LINE(GPIOI, 5U)
#define	LINE_SPI_SLAVE9                PAL_LINE(GPIOI, 6U)
#define	LINE_SPI_SLAVE10               PAL_LINE(GPIOI, 7U)
#define	LINE_SPI_SLAVE11               PAL_LINE(GPIOI, 8U)
#define	LINE_CAN1_RX                   PAL_LINE(GPIOI, 9U)
#define	LINE_SPI_SLAVE12               PAL_LINE(GPIOI, 10U)
#define	LINE_SPI_SLAVE13               PAL_LINE(GPIOI, 11U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define VAL_GPIOA_MODER                 (PIN_MODE_ANALOG(ADC1) | \
					 PIN_MODE_ANALOG(ADC2) | \
					 PIN_MODE_ANALOG(ADC3) | \
					 PIN_MODE_ANALOG(ADC4) | \
					 PIN_MODE_ANALOG(ADC5) | \
					 PIN_MODE_INPUT(FMU_CAP1) | \
					 PIN_MODE_ALTERNATE(SPI1_MISO) | \
					 PIN_MODE_INPUT(HEATER) | \
					 PIN_MODE_ALTERNATE(CAN3_RX) | \
					 PIN_MODE_INPUT(USB_VBUS) | \
					 PIN_MODE_ALTERNATE(SERVO2) | \
					 PIN_MODE_ALTERNATE(USB_DM) | \
					 PIN_MODE_ALTERNATE(USB_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_ALTERNATE(CAN3_TX))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(ADC1) | \
					 PIN_OTYPE_PUSHPULL(ADC2) | \
					 PIN_OTYPE_PUSHPULL(ADC3) | \
					 PIN_OTYPE_PUSHPULL(ADC4) | \
					 PIN_OTYPE_PUSHPULL(ADC5) | \
					 PIN_OTYPE_OPENDRAIN(FMU_CAP1) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MISO) | \
					 PIN_OTYPE_OPENDRAIN(HEATER) | \
					 PIN_OTYPE_PUSHPULL(CAN3_RX) | \
					 PIN_OTYPE_OPENDRAIN(USB_VBUS) | \
					 PIN_OTYPE_PUSHPULL(SERVO2) | \
					 PIN_OTYPE_PUSHPULL(USB_DM) | \
					 PIN_OTYPE_PUSHPULL(USB_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_PUSHPULL(CAN3_TX))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(ADC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC3) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC4) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC5) | \
					 PIN_OSPEED_SPEED_VERYLOW(FMU_CAP1) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(HEATER) | \
					 PIN_OSPEED_SPEED_HIGH(CAN3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(USB_VBUS) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO2) | \
					 PIN_OSPEED_SPEED_HIGH(USB_DM) | \
					 PIN_OSPEED_SPEED_HIGH(USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(CAN3_TX))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(ADC1) | \
					 PIN_PUPDR_FLOATING(ADC2) | \
					 PIN_PUPDR_FLOATING(ADC3) | \
					 PIN_PUPDR_FLOATING(ADC4) | \
					 PIN_PUPDR_FLOATING(ADC5) | \
					 PIN_PUPDR_PULLDOWN(FMU_CAP1) | \
					 PIN_PUPDR_FLOATING(SPI1_MISO) | \
					 PIN_PUPDR_PULLDOWN(HEATER) | \
					 PIN_PUPDR_FLOATING(CAN3_RX) | \
					 PIN_PUPDR_PULLDOWN(USB_VBUS) | \
					 PIN_PUPDR_FLOATING(SERVO2) | \
					 PIN_PUPDR_FLOATING(USB_DM) | \
					 PIN_PUPDR_FLOATING(USB_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_FLOATING(CAN3_TX))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(ADC1) | \
					 PIN_ODR_LEVEL_LOW(ADC2) | \
					 PIN_ODR_LEVEL_LOW(ADC3) | \
					 PIN_ODR_LEVEL_LOW(ADC4) | \
					 PIN_ODR_LEVEL_LOW(ADC5) | \
					 PIN_ODR_LEVEL_HIGH(FMU_CAP1) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(HEATER) | \
					 PIN_ODR_LEVEL_HIGH(CAN3_RX) | \
					 PIN_ODR_LEVEL_LOW(USB_VBUS) | \
					 PIN_ODR_LEVEL_LOW(SERVO2) | \
					 PIN_ODR_LEVEL_HIGH(USB_DM) | \
					 PIN_ODR_LEVEL_HIGH(USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(CAN3_TX))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(ADC1, 0) | \
					 PIN_AFIO_AF(ADC2, 0) | \
					 PIN_AFIO_AF(ADC3, 0) | \
					 PIN_AFIO_AF(ADC4, 0) | \
					 PIN_AFIO_AF(ADC5, 0) | \
					 PIN_AFIO_AF(FMU_CAP1, 0) | \
					 PIN_AFIO_AF(SPI1_MISO, 5) | \
					 PIN_AFIO_AF(HEATER, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(CAN3_RX, 11) | \
					 PIN_AFIO_AF(USB_VBUS, 0) | \
					 PIN_AFIO_AF(SERVO2, 1) | \
					 PIN_AFIO_AF(USB_DM, 10) | \
					 PIN_AFIO_AF(USB_DP, 10) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(CAN3_TX, 11))

#define VAL_GPIOB_MODER                 (PIN_MODE_ANALOG(RSSI_IN) | \
					 PIN_MODE_OUTPUT(LED1) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_INPUT(FMU_CAP2) | \
					 PIN_MODE_INPUT(DRDY1_ICM20689) | \
					 PIN_MODE_ALTERNATE(SPI6_MOSI) | \
					 PIN_MODE_ALTERNATE(USART1_TX) | \
					 PIN_MODE_ALTERNATE(USART1_RX) | \
					 PIN_MODE_ALTERNATE(I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(I2C1_SDA) | \
					 PIN_MODE_INPUT(SPI5_RESET) | \
					 PIN_MODE_INPUT(FMU_CAP3) | \
					 PIN_MODE_ALTERNATE(CAN2_RX) | \
					 PIN_MODE_ALTERNATE(CAN2_TX) | \
					 PIN_MODE_INPUT(DRDY2_BMI055_GYRO) | \
					 PIN_MODE_INPUT(DRDY2_BMI055_ACC))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(RSSI_IN) | \
					 PIN_OTYPE_PUSHPULL(LED1) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_OPENDRAIN(FMU_CAP2) | \
					 PIN_OTYPE_OPENDRAIN(DRDY1_ICM20689) | \
					 PIN_OTYPE_PUSHPULL(SPI6_MOSI) | \
					 PIN_OTYPE_PUSHPULL(USART1_TX) | \
					 PIN_OTYPE_PUSHPULL(USART1_RX) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SDA) | \
					 PIN_OTYPE_OPENDRAIN(SPI5_RESET) | \
					 PIN_OTYPE_OPENDRAIN(FMU_CAP3) | \
					 PIN_OTYPE_PUSHPULL(CAN2_RX) | \
					 PIN_OTYPE_PUSHPULL(CAN2_TX) | \
					 PIN_OTYPE_OPENDRAIN(DRDY2_BMI055_GYRO) | \
					 PIN_OTYPE_OPENDRAIN(DRDY2_BMI055_ACC))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(RSSI_IN) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_VERYLOW(FMU_CAP2) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY1_ICM20689) | \
					 PIN_OSPEED_SPEED_HIGH(SPI6_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(SPI5_RESET) | \
					 PIN_OSPEED_SPEED_VERYLOW(FMU_CAP3) | \
					 PIN_OSPEED_SPEED_HIGH(CAN2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(CAN2_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY2_BMI055_GYRO) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY2_BMI055_ACC))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(RSSI_IN) | \
					 PIN_PUPDR_FLOATING(LED1) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_PULLDOWN(FMU_CAP2) | \
					 PIN_PUPDR_PULLDOWN(DRDY1_ICM20689) | \
					 PIN_PUPDR_FLOATING(SPI6_MOSI) | \
					 PIN_PUPDR_FLOATING(USART1_TX) | \
					 PIN_PUPDR_FLOATING(USART1_RX) | \
					 PIN_PUPDR_PULLUP(I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(I2C1_SDA) | \
					 PIN_PUPDR_PULLDOWN(SPI5_RESET) | \
					 PIN_PUPDR_PULLDOWN(FMU_CAP3) | \
					 PIN_PUPDR_FLOATING(CAN2_RX) | \
					 PIN_PUPDR_FLOATING(CAN2_TX) | \
					 PIN_PUPDR_PULLDOWN(DRDY2_BMI055_GYRO) | \
					 PIN_PUPDR_PULLDOWN(DRDY2_BMI055_ACC))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(RSSI_IN) | \
					 PIN_ODR_LEVEL_LOW(LED1) | \
					 PIN_ODR_LEVEL_LOW(PB02) | \
					 PIN_ODR_LEVEL_HIGH(FMU_CAP2) | \
					 PIN_ODR_LEVEL_HIGH(DRDY1_ICM20689) | \
					 PIN_ODR_LEVEL_HIGH(SPI6_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(USART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(SPI5_RESET) | \
					 PIN_ODR_LEVEL_HIGH(FMU_CAP3) | \
					 PIN_ODR_LEVEL_HIGH(CAN2_RX) | \
					 PIN_ODR_LEVEL_HIGH(CAN2_TX) | \
					 PIN_ODR_LEVEL_HIGH(DRDY2_BMI055_GYRO) | \
					 PIN_ODR_LEVEL_HIGH(DRDY2_BMI055_ACC))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(RSSI_IN, 0) | \
					 PIN_AFIO_AF(LED1, 0) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(FMU_CAP2, 0) | \
					 PIN_AFIO_AF(DRDY1_ICM20689, 0) | \
					 PIN_AFIO_AF(SPI6_MOSI, 8) | \
					 PIN_AFIO_AF(USART1_TX, 7) | \
					 PIN_AFIO_AF(USART1_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(I2C1_SCL, 4) | \
					 PIN_AFIO_AF(I2C1_SDA, 4) | \
					 PIN_AFIO_AF(SPI5_RESET, 0) | \
					 PIN_AFIO_AF(FMU_CAP3, 0) | \
					 PIN_AFIO_AF(CAN2_RX, 9) | \
					 PIN_AFIO_AF(CAN2_TX, 9) | \
					 PIN_AFIO_AF(DRDY2_BMI055_GYRO, 0) | \
					 PIN_AFIO_AF(DRDY2_BMI055_ACC, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(SCALED_V5) | \
					 PIN_MODE_ANALOG(SCALED_3V3_SENSORS) | \
					 PIN_MODE_ANALOG(HW_VER_SENSE) | \
					 PIN_MODE_ANALOG(HW_REV_SENSE) | \
					 PIN_MODE_ANALOG(ADC6) | \
					 PIN_MODE_INPUT(DRDY4_ICM20602) | \
					 PIN_MODE_OUTPUT(LED2) | \
					 PIN_MODE_OUTPUT(LED3) | \
					 PIN_MODE_ALTERNATE(SDIO_D0) | \
					 PIN_MODE_ALTERNATE(SDIO_D1) | \
					 PIN_MODE_ALTERNATE(SDIO_D2) | \
					 PIN_MODE_ALTERNATE(SDIO_D3) | \
					 PIN_MODE_ALTERNATE(SDIO_CK) | \
					 PIN_MODE_INPUT(DRDY5_BMI055_GYRO) | \
					 PIN_MODE_ALTERNATE(OSC32_IN) | \
					 PIN_MODE_ALTERNATE(OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(SCALED_V5) | \
					 PIN_OTYPE_PUSHPULL(SCALED_3V3_SENSORS) | \
					 PIN_OTYPE_PUSHPULL(HW_VER_SENSE) | \
					 PIN_OTYPE_PUSHPULL(HW_REV_SENSE) | \
					 PIN_OTYPE_PUSHPULL(ADC6) | \
					 PIN_OTYPE_OPENDRAIN(DRDY4_ICM20602) | \
					 PIN_OTYPE_PUSHPULL(LED2) | \
					 PIN_OTYPE_PUSHPULL(LED3) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D0) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D1) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D2) | \
					 PIN_OTYPE_PUSHPULL(SDIO_D3) | \
					 PIN_OTYPE_PUSHPULL(SDIO_CK) | \
					 PIN_OTYPE_OPENDRAIN(DRDY5_BMI055_GYRO) | \
					 PIN_OTYPE_PUSHPULL(OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(SCALED_V5) | \
					 PIN_OSPEED_SPEED_VERYLOW(SCALED_3V3_SENSORS) | \
					 PIN_OSPEED_SPEED_VERYLOW(HW_VER_SENSE) | \
					 PIN_OSPEED_SPEED_VERYLOW(HW_REV_SENSE) | \
					 PIN_OSPEED_SPEED_VERYLOW(ADC6) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY4_ICM20602) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED2) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED3) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY5_BMI055_GYRO) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(SCALED_V5) | \
					 PIN_PUPDR_FLOATING(SCALED_3V3_SENSORS) | \
					 PIN_PUPDR_FLOATING(HW_VER_SENSE) | \
					 PIN_PUPDR_FLOATING(HW_REV_SENSE) | \
					 PIN_PUPDR_FLOATING(ADC6) | \
					 PIN_PUPDR_PULLDOWN(DRDY4_ICM20602) | \
					 PIN_PUPDR_FLOATING(LED2) | \
					 PIN_PUPDR_FLOATING(LED3) | \
					 PIN_PUPDR_PULLUP(SDIO_D0) | \
					 PIN_PUPDR_PULLUP(SDIO_D1) | \
					 PIN_PUPDR_PULLUP(SDIO_D2) | \
					 PIN_PUPDR_PULLUP(SDIO_D3) | \
					 PIN_PUPDR_PULLUP(SDIO_CK) | \
					 PIN_PUPDR_PULLDOWN(DRDY5_BMI055_GYRO) | \
					 PIN_PUPDR_FLOATING(OSC32_IN) | \
					 PIN_PUPDR_FLOATING(OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(SCALED_V5) | \
					 PIN_ODR_LEVEL_LOW(SCALED_3V3_SENSORS) | \
					 PIN_ODR_LEVEL_LOW(HW_VER_SENSE) | \
					 PIN_ODR_LEVEL_LOW(HW_REV_SENSE) | \
					 PIN_ODR_LEVEL_LOW(ADC6) | \
					 PIN_ODR_LEVEL_HIGH(DRDY4_ICM20602) | \
					 PIN_ODR_LEVEL_LOW(LED2) | \
					 PIN_ODR_LEVEL_LOW(LED3) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_CK) | \
					 PIN_ODR_LEVEL_HIGH(DRDY5_BMI055_GYRO) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(SCALED_V5, 0) | \
					 PIN_AFIO_AF(SCALED_3V3_SENSORS, 0) | \
					 PIN_AFIO_AF(HW_VER_SENSE, 0) | \
					 PIN_AFIO_AF(HW_REV_SENSE, 0) | \
					 PIN_AFIO_AF(ADC6, 0) | \
					 PIN_AFIO_AF(DRDY4_ICM20602, 0) | \
					 PIN_AFIO_AF(LED2, 0) | \
					 PIN_AFIO_AF(LED3, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDIO_D0, 12) | \
					 PIN_AFIO_AF(SDIO_D1, 12) | \
					 PIN_AFIO_AF(SDIO_D2, 12) | \
					 PIN_AFIO_AF(SDIO_D3, 12) | \
					 PIN_AFIO_AF(SDIO_CK, 12) | \
					 PIN_AFIO_AF(DRDY5_BMI055_GYRO, 0) | \
					 PIN_AFIO_AF(OSC32_IN, 0) | \
					 PIN_AFIO_AF(OSC32_OUT, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(UART4_RX) | \
					 PIN_MODE_ALTERNATE(UART4_TX) | \
					 PIN_MODE_ALTERNATE(SDIO_CMD) | \
					 PIN_MODE_INPUT(UART2_CTS) | \
					 PIN_MODE_INPUT(UART2_RTS) | \
					 PIN_MODE_ALTERNATE(UART2_TX) | \
					 PIN_MODE_ALTERNATE(UART2_RX) | \
					 PIN_MODE_ALTERNATE(SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(UART3_TX) | \
					 PIN_MODE_ALTERNATE(UART3_RX) | \
					 PIN_MODE_INPUT(DRDY6_BMI055_ACC) | \
					 PIN_MODE_INPUT(UART3_CTS) | \
					 PIN_MODE_INPUT(UART3_RTS) | \
					 PIN_MODE_ALTERNATE(SERVO5) | \
					 PIN_MODE_ALTERNATE(SERVO6) | \
					 PIN_MODE_INPUT(DRDY7_EXTERNAL1))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(SDIO_CMD) | \
					 PIN_OTYPE_OPENDRAIN(UART2_CTS) | \
					 PIN_OTYPE_OPENDRAIN(UART2_RTS) | \
					 PIN_OTYPE_PUSHPULL(UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(UART3_RX) | \
					 PIN_OTYPE_OPENDRAIN(DRDY6_BMI055_ACC) | \
					 PIN_OTYPE_OPENDRAIN(UART3_CTS) | \
					 PIN_OTYPE_OPENDRAIN(UART3_RTS) | \
					 PIN_OTYPE_PUSHPULL(SERVO5) | \
					 PIN_OTYPE_PUSHPULL(SERVO6) | \
					 PIN_OTYPE_OPENDRAIN(DRDY7_EXTERNAL1))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART4_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SDIO_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(UART2_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(UART2_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY6_BMI055_ACC) | \
					 PIN_OSPEED_SPEED_VERYLOW(UART3_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(UART3_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO5) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO6) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY7_EXTERNAL1))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(UART4_RX) | \
					 PIN_PUPDR_FLOATING(UART4_TX) | \
					 PIN_PUPDR_PULLUP(SDIO_CMD) | \
					 PIN_PUPDR_PULLDOWN(UART2_CTS) | \
					 PIN_PUPDR_PULLDOWN(UART2_RTS) | \
					 PIN_PUPDR_FLOATING(UART2_TX) | \
					 PIN_PUPDR_FLOATING(UART2_RX) | \
					 PIN_PUPDR_FLOATING(SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(UART3_TX) | \
					 PIN_PUPDR_FLOATING(UART3_RX) | \
					 PIN_PUPDR_PULLDOWN(DRDY6_BMI055_ACC) | \
					 PIN_PUPDR_PULLDOWN(UART3_CTS) | \
					 PIN_PUPDR_PULLDOWN(UART3_RTS) | \
					 PIN_PUPDR_FLOATING(SERVO5) | \
					 PIN_PUPDR_FLOATING(SERVO6) | \
					 PIN_PUPDR_PULLDOWN(DRDY7_EXTERNAL1))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(SDIO_CMD) | \
					 PIN_ODR_LEVEL_HIGH(UART2_CTS) | \
					 PIN_ODR_LEVEL_HIGH(UART2_RTS) | \
					 PIN_ODR_LEVEL_HIGH(UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART3_RX) | \
					 PIN_ODR_LEVEL_HIGH(DRDY6_BMI055_ACC) | \
					 PIN_ODR_LEVEL_HIGH(UART3_CTS) | \
					 PIN_ODR_LEVEL_HIGH(UART3_RTS) | \
					 PIN_ODR_LEVEL_LOW(SERVO5) | \
					 PIN_ODR_LEVEL_LOW(SERVO6) | \
					 PIN_ODR_LEVEL_HIGH(DRDY7_EXTERNAL1))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(UART4_RX, 8) | \
					 PIN_AFIO_AF(UART4_TX, 8) | \
					 PIN_AFIO_AF(SDIO_CMD, 12) | \
					 PIN_AFIO_AF(UART2_CTS, 0) | \
					 PIN_AFIO_AF(UART2_RTS, 0) | \
					 PIN_AFIO_AF(UART2_TX, 7) | \
					 PIN_AFIO_AF(UART2_RX, 7) | \
					 PIN_AFIO_AF(SPI1_MOSI, 5))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(UART3_TX, 7) | \
					 PIN_AFIO_AF(UART3_RX, 7) | \
					 PIN_AFIO_AF(DRDY6_BMI055_ACC, 0) | \
					 PIN_AFIO_AF(UART3_CTS, 0) | \
					 PIN_AFIO_AF(UART3_RTS, 0) | \
					 PIN_AFIO_AF(SERVO5, 2) | \
					 PIN_AFIO_AF(SERVO6, 2) | \
					 PIN_AFIO_AF(DRDY7_EXTERNAL1, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(UART8_RX) | \
					 PIN_MODE_ALTERNATE(UART8_TX) | \
					 PIN_MODE_ALTERNATE(SPI4_SCK) | \
					 PIN_MODE_OUTPUT(V3V3_SENSORS_EN) | \
					 PIN_MODE_OUTPUT(V3V3_SPEKTRUM_EN) | \
					 PIN_MODE_INPUT(BUZZER) | \
					 PIN_MODE_ALTERNATE(SPI4_MOSI) | \
					 PIN_MODE_INPUT(DRDY8) | \
					 PIN_MODE_ALTERNATE(UART7_TX) | \
					 PIN_MODE_ALTERNATE(SERVO4) | \
					 PIN_MODE_INPUT(SAFETY_SWITCH_IN) | \
					 PIN_MODE_ALTERNATE(SERVO3) | \
					 PIN_MODE_OUTPUT(LED4) | \
					 PIN_MODE_ALTERNATE(SPI4_MISO) | \
					 PIN_MODE_ALTERNATE(SERVO1) | \
					 PIN_MODE_INPUT(V5V_PERIPH_OC))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(UART8_TX) | \
					 PIN_OTYPE_PUSHPULL(SPI4_SCK) | \
					 PIN_OTYPE_PUSHPULL(V3V3_SENSORS_EN) | \
					 PIN_OTYPE_PUSHPULL(V3V3_SPEKTRUM_EN) | \
					 PIN_OTYPE_OPENDRAIN(BUZZER) | \
					 PIN_OTYPE_PUSHPULL(SPI4_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(DRDY8) | \
					 PIN_OTYPE_PUSHPULL(UART7_TX) | \
					 PIN_OTYPE_PUSHPULL(SERVO4) | \
					 PIN_OTYPE_OPENDRAIN(SAFETY_SWITCH_IN) | \
					 PIN_OTYPE_PUSHPULL(SERVO3) | \
					 PIN_OTYPE_PUSHPULL(LED4) | \
					 PIN_OTYPE_PUSHPULL(SPI4_MISO) | \
					 PIN_OTYPE_PUSHPULL(SERVO1) | \
					 PIN_OTYPE_OPENDRAIN(V5V_PERIPH_OC))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART8_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(V3V3_SENSORS_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(V3V3_SPEKTRUM_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(BUZZER) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(DRDY8) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO4) | \
					 PIN_OSPEED_SPEED_VERYLOW(SAFETY_SWITCH_IN) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO3) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED4) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO1) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_PERIPH_OC))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(UART8_RX) | \
					 PIN_PUPDR_FLOATING(UART8_TX) | \
					 PIN_PUPDR_FLOATING(SPI4_SCK) | \
					 PIN_PUPDR_FLOATING(V3V3_SENSORS_EN) | \
					 PIN_PUPDR_FLOATING(V3V3_SPEKTRUM_EN) | \
					 PIN_PUPDR_PULLDOWN(BUZZER) | \
					 PIN_PUPDR_FLOATING(SPI4_MOSI) | \
					 PIN_PUPDR_PULLDOWN(DRDY8) | \
					 PIN_PUPDR_FLOATING(UART7_TX) | \
					 PIN_PUPDR_FLOATING(SERVO4) | \
					 PIN_PUPDR_PULLDOWN(SAFETY_SWITCH_IN) | \
					 PIN_PUPDR_FLOATING(SERVO3) | \
					 PIN_PUPDR_FLOATING(LED4) | \
					 PIN_PUPDR_FLOATING(SPI4_MISO) | \
					 PIN_PUPDR_FLOATING(SERVO1) | \
					 PIN_PUPDR_PULLDOWN(V5V_PERIPH_OC))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART8_TX) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_SCK) | \
					 PIN_ODR_LEVEL_HIGH(V3V3_SENSORS_EN) | \
					 PIN_ODR_LEVEL_HIGH(V3V3_SPEKTRUM_EN) | \
					 PIN_ODR_LEVEL_HIGH(BUZZER) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(DRDY8) | \
					 PIN_ODR_LEVEL_HIGH(UART7_TX) | \
					 PIN_ODR_LEVEL_LOW(SERVO4) | \
					 PIN_ODR_LEVEL_LOW(SAFETY_SWITCH_IN) | \
					 PIN_ODR_LEVEL_LOW(SERVO3) | \
					 PIN_ODR_LEVEL_LOW(LED4) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_MISO) | \
					 PIN_ODR_LEVEL_LOW(SERVO1) | \
					 PIN_ODR_LEVEL_HIGH(V5V_PERIPH_OC))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(UART8_RX, 8) | \
					 PIN_AFIO_AF(UART8_TX, 8) | \
					 PIN_AFIO_AF(SPI4_SCK, 5) | \
					 PIN_AFIO_AF(V3V3_SENSORS_EN, 0) | \
					 PIN_AFIO_AF(V3V3_SPEKTRUM_EN, 0) | \
					 PIN_AFIO_AF(BUZZER, 0) | \
					 PIN_AFIO_AF(SPI4_MOSI, 5) | \
					 PIN_AFIO_AF(DRDY8, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(UART7_TX, 8) | \
					 PIN_AFIO_AF(SERVO4, 1) | \
					 PIN_AFIO_AF(SAFETY_SWITCH_IN, 0) | \
					 PIN_AFIO_AF(SERVO3, 1) | \
					 PIN_AFIO_AF(LED4, 0) | \
					 PIN_AFIO_AF(SPI4_MISO, 5) | \
					 PIN_AFIO_AF(SERVO1, 1) | \
					 PIN_AFIO_AF(V5V_PERIPH_OC, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_ALTERNATE(I2C2_SDA) | \
					 PIN_MODE_ALTERNATE(I2C2_SCL) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE0) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE1) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE2) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE3) | \
					 PIN_MODE_ALTERNATE(UART7_RX) | \
					 PIN_MODE_ALTERNATE(SPI5_SCK) | \
					 PIN_MODE_ALTERNATE(SPI5_MISO) | \
					 PIN_MODE_ALTERNATE(SPI5_MOSI) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE4) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE5) | \
					 PIN_MODE_OUTPUT(V5V_HIPOWER_EN) | \
					 PIN_MODE_INPUT(V5V_HIPOWER_OC) | \
					 PIN_MODE_ALTERNATE(I2C4_SCL) | \
					 PIN_MODE_ALTERNATE(I2C4_SDA))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_OPENDRAIN(I2C2_SDA) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SCL) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE0) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE1) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE2) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE3) | \
					 PIN_OTYPE_PUSHPULL(UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(SPI5_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI5_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI5_MOSI) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE4) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE5) | \
					 PIN_OTYPE_PUSHPULL(V5V_HIPOWER_EN) | \
					 PIN_OTYPE_OPENDRAIN(V5V_HIPOWER_OC) | \
					 PIN_OTYPE_OPENDRAIN(I2C4_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C4_SDA))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE0) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE1) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE2) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE3) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SPI5_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI5_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI5_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE4) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE5) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_HIPOWER_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_HIPOWER_OC) | \
					 PIN_OSPEED_SPEED_HIGH(I2C4_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C4_SDA))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLUP(I2C2_SDA) | \
					 PIN_PUPDR_PULLUP(I2C2_SCL) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE0) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE1) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE2) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE3) | \
					 PIN_PUPDR_FLOATING(UART7_RX) | \
					 PIN_PUPDR_FLOATING(SPI5_SCK) | \
					 PIN_PUPDR_FLOATING(SPI5_MISO) | \
					 PIN_PUPDR_FLOATING(SPI5_MOSI) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE4) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE5) | \
					 PIN_PUPDR_FLOATING(V5V_HIPOWER_EN) | \
					 PIN_PUPDR_PULLDOWN(V5V_HIPOWER_OC) | \
					 PIN_PUPDR_PULLUP(I2C4_SCL) | \
					 PIN_PUPDR_PULLUP(I2C4_SDA))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_HIGH(I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE0) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE1) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE2) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE3) | \
					 PIN_ODR_LEVEL_HIGH(UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(SPI5_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI5_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI5_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE4) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE5) | \
					 PIN_ODR_LEVEL_LOW(V5V_HIPOWER_EN) | \
					 PIN_ODR_LEVEL_HIGH(V5V_HIPOWER_OC) | \
					 PIN_ODR_LEVEL_HIGH(I2C4_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C4_SDA))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(I2C2_SDA, 4) | \
					 PIN_AFIO_AF(I2C2_SCL, 4) | \
					 PIN_AFIO_AF(SPI_SLAVE0, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE1, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE2, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE3, 0) | \
					 PIN_AFIO_AF(UART7_RX, 8) | \
					 PIN_AFIO_AF(SPI5_SCK, 5))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(SPI5_MISO, 5) | \
					 PIN_AFIO_AF(SPI5_MOSI, 5) | \
					 PIN_AFIO_AF(SPI_SLAVE4, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE5, 0) | \
					 PIN_AFIO_AF(V5V_HIPOWER_EN, 0) | \
					 PIN_AFIO_AF(V5V_HIPOWER_OC, 0) | \
					 PIN_AFIO_AF(I2C4_SCL, 4) | \
					 PIN_AFIO_AF(I2C4_SDA, 4))

#define VAL_GPIOG_MODER                 (PIN_MODE_OUTPUT(HW_VER_DRIVE) | \
					 PIN_MODE_INPUT(POWER_IN_A) | \
					 PIN_MODE_INPUT(POWER_IN_B) | \
					 PIN_MODE_INPUT(POWER_IN_C) | \
					 PIN_MODE_OUTPUT(V5V_PERIPH_EN) | \
					 PIN_MODE_OUTPUT(V5V_RC_EN) | \
					 PIN_MODE_OUTPUT(V5V_WIFI_EN) | \
					 PIN_MODE_OUTPUT(V3V3_SD_CARD_EN) | \
					 PIN_MODE_INPUT(USART6_RTS) | \
					 PIN_MODE_ALTERNATE(USART6_RX) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE6) | \
					 PIN_MODE_ALTERNATE(SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(SPI6_MISO) | \
					 PIN_MODE_ALTERNATE(SPI6_SCK) | \
					 PIN_MODE_ALTERNATE(USART6_TX) | \
					 PIN_MODE_INPUT(USART6_CTS))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(HW_VER_DRIVE) | \
					 PIN_OTYPE_OPENDRAIN(POWER_IN_A) | \
					 PIN_OTYPE_OPENDRAIN(POWER_IN_B) | \
					 PIN_OTYPE_OPENDRAIN(POWER_IN_C) | \
					 PIN_OTYPE_PUSHPULL(V5V_PERIPH_EN) | \
					 PIN_OTYPE_PUSHPULL(V5V_RC_EN) | \
					 PIN_OTYPE_PUSHPULL(V5V_WIFI_EN) | \
					 PIN_OTYPE_PUSHPULL(V3V3_SD_CARD_EN) | \
					 PIN_OTYPE_OPENDRAIN(USART6_RTS) | \
					 PIN_OTYPE_PUSHPULL(USART6_RX) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE6) | \
					 PIN_OTYPE_PUSHPULL(SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI6_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI6_SCK) | \
					 PIN_OTYPE_PUSHPULL(USART6_TX) | \
					 PIN_OTYPE_OPENDRAIN(USART6_CTS))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(HW_VER_DRIVE) | \
					 PIN_OSPEED_SPEED_VERYLOW(POWER_IN_A) | \
					 PIN_OSPEED_SPEED_VERYLOW(POWER_IN_B) | \
					 PIN_OSPEED_SPEED_VERYLOW(POWER_IN_C) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_PERIPH_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_RC_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(V5V_WIFI_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(V3V3_SD_CARD_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART6_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(USART6_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE6) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI6_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI6_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(USART6_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART6_CTS))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_FLOATING(HW_VER_DRIVE) | \
					 PIN_PUPDR_PULLDOWN(POWER_IN_A) | \
					 PIN_PUPDR_PULLDOWN(POWER_IN_B) | \
					 PIN_PUPDR_PULLDOWN(POWER_IN_C) | \
					 PIN_PUPDR_FLOATING(V5V_PERIPH_EN) | \
					 PIN_PUPDR_FLOATING(V5V_RC_EN) | \
					 PIN_PUPDR_FLOATING(V5V_WIFI_EN) | \
					 PIN_PUPDR_FLOATING(V3V3_SD_CARD_EN) | \
					 PIN_PUPDR_PULLDOWN(USART6_RTS) | \
					 PIN_PUPDR_FLOATING(USART6_RX) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE6) | \
					 PIN_PUPDR_FLOATING(SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(SPI6_MISO) | \
					 PIN_PUPDR_FLOATING(SPI6_SCK) | \
					 PIN_PUPDR_FLOATING(USART6_TX) | \
					 PIN_PUPDR_PULLDOWN(USART6_CTS))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(HW_VER_DRIVE) | \
					 PIN_ODR_LEVEL_HIGH(POWER_IN_A) | \
					 PIN_ODR_LEVEL_HIGH(POWER_IN_B) | \
					 PIN_ODR_LEVEL_HIGH(POWER_IN_C) | \
					 PIN_ODR_LEVEL_LOW(V5V_PERIPH_EN) | \
					 PIN_ODR_LEVEL_HIGH(V5V_RC_EN) | \
					 PIN_ODR_LEVEL_HIGH(V5V_WIFI_EN) | \
					 PIN_ODR_LEVEL_HIGH(V3V3_SD_CARD_EN) | \
					 PIN_ODR_LEVEL_HIGH(USART6_RTS) | \
					 PIN_ODR_LEVEL_HIGH(USART6_RX) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE6) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI6_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI6_SCK) | \
					 PIN_ODR_LEVEL_HIGH(USART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART6_CTS))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(HW_VER_DRIVE, 0) | \
					 PIN_AFIO_AF(POWER_IN_A, 0) | \
					 PIN_AFIO_AF(POWER_IN_B, 0) | \
					 PIN_AFIO_AF(POWER_IN_C, 0) | \
					 PIN_AFIO_AF(V5V_PERIPH_EN, 0) | \
					 PIN_AFIO_AF(V5V_RC_EN, 0) | \
					 PIN_AFIO_AF(V5V_WIFI_EN, 0) | \
					 PIN_AFIO_AF(V3V3_SD_CARD_EN, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(USART6_RTS, 0) | \
					 PIN_AFIO_AF(USART6_RX, 8) | \
					 PIN_AFIO_AF(SPI_SLAVE6, 0) | \
					 PIN_AFIO_AF(SPI1_SCK, 5) | \
					 PIN_AFIO_AF(SPI6_MISO, 5) | \
					 PIN_AFIO_AF(SPI6_SCK, 5) | \
					 PIN_AFIO_AF(USART6_TX, 8) | \
					 PIN_AFIO_AF(USART6_CTS, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(OSC_IN) | \
					 PIN_MODE_ALTERNATE(OSC_OUT) | \
					 PIN_MODE_OUTPUT(CAN1_SILENT_S0) | \
					 PIN_MODE_OUTPUT(CAN2_SILENT_S1) | \
					 PIN_MODE_OUTPUT(CAN3_SILENT_S2) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE7) | \
					 PIN_MODE_ALTERNATE(SERVO7) | \
					 PIN_MODE_ALTERNATE(I2C3_SCL) | \
					 PIN_MODE_ALTERNATE(I2C3_SDA) | \
					 PIN_MODE_ALTERNATE(SERVO8) | \
					 PIN_MODE_OUTPUT(LED5) | \
					 PIN_MODE_OUTPUT(LED6) | \
					 PIN_MODE_OUTPUT(LED7) | \
					 PIN_MODE_ALTERNATE(CAN1_TX) | \
					 PIN_MODE_OUTPUT(HW_REV_DRIVE) | \
					 PIN_MODE_INPUT(SPI5_SYNC))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(CAN1_SILENT_S0) | \
					 PIN_OTYPE_PUSHPULL(CAN2_SILENT_S1) | \
					 PIN_OTYPE_PUSHPULL(CAN3_SILENT_S2) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE7) | \
					 PIN_OTYPE_PUSHPULL(SERVO7) | \
					 PIN_OTYPE_OPENDRAIN(I2C3_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C3_SDA) | \
					 PIN_OTYPE_PUSHPULL(SERVO8) | \
					 PIN_OTYPE_PUSHPULL(LED5) | \
					 PIN_OTYPE_PUSHPULL(LED6) | \
					 PIN_OTYPE_PUSHPULL(LED7) | \
					 PIN_OTYPE_PUSHPULL(CAN1_TX) | \
					 PIN_OTYPE_PUSHPULL(HW_REV_DRIVE) | \
					 PIN_OTYPE_OPENDRAIN(SPI5_SYNC))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(CAN1_SILENT_S0) | \
					 PIN_OSPEED_SPEED_VERYLOW(CAN2_SILENT_S1) | \
					 PIN_OSPEED_SPEED_VERYLOW(CAN3_SILENT_S2) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE7) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO7) | \
					 PIN_OSPEED_SPEED_HIGH(I2C3_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C3_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(SERVO8) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED5) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED6) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED7) | \
					 PIN_OSPEED_SPEED_HIGH(CAN1_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(HW_REV_DRIVE) | \
					 PIN_OSPEED_SPEED_VERYLOW(SPI5_SYNC))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(OSC_IN) | \
					 PIN_PUPDR_FLOATING(OSC_OUT) | \
					 PIN_PUPDR_FLOATING(CAN1_SILENT_S0) | \
					 PIN_PUPDR_FLOATING(CAN2_SILENT_S1) | \
					 PIN_PUPDR_FLOATING(CAN3_SILENT_S2) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE7) | \
					 PIN_PUPDR_FLOATING(SERVO7) | \
					 PIN_PUPDR_PULLUP(I2C3_SCL) | \
					 PIN_PUPDR_PULLUP(I2C3_SDA) | \
					 PIN_PUPDR_FLOATING(SERVO8) | \
					 PIN_PUPDR_FLOATING(LED5) | \
					 PIN_PUPDR_FLOATING(LED6) | \
					 PIN_PUPDR_FLOATING(LED7) | \
					 PIN_PUPDR_FLOATING(CAN1_TX) | \
					 PIN_PUPDR_FLOATING(HW_REV_DRIVE) | \
					 PIN_PUPDR_PULLDOWN(SPI5_SYNC))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC_OUT) | \
					 PIN_ODR_LEVEL_LOW(CAN1_SILENT_S0) | \
					 PIN_ODR_LEVEL_LOW(CAN2_SILENT_S1) | \
					 PIN_ODR_LEVEL_LOW(CAN3_SILENT_S2) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE7) | \
					 PIN_ODR_LEVEL_LOW(SERVO7) | \
					 PIN_ODR_LEVEL_HIGH(I2C3_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C3_SDA) | \
					 PIN_ODR_LEVEL_LOW(SERVO8) | \
					 PIN_ODR_LEVEL_LOW(LED5) | \
					 PIN_ODR_LEVEL_LOW(LED6) | \
					 PIN_ODR_LEVEL_LOW(LED7) | \
					 PIN_ODR_LEVEL_HIGH(CAN1_TX) | \
					 PIN_ODR_LEVEL_LOW(HW_REV_DRIVE) | \
					 PIN_ODR_LEVEL_HIGH(SPI5_SYNC))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(OSC_IN, 0) | \
					 PIN_AFIO_AF(OSC_OUT, 0) | \
					 PIN_AFIO_AF(CAN1_SILENT_S0, 0) | \
					 PIN_AFIO_AF(CAN2_SILENT_S1, 0) | \
					 PIN_AFIO_AF(CAN3_SILENT_S2, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE7, 0) | \
					 PIN_AFIO_AF(SERVO7, 9) | \
					 PIN_AFIO_AF(I2C3_SCL, 4))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(I2C3_SDA, 4) | \
					 PIN_AFIO_AF(SERVO8, 9) | \
					 PIN_AFIO_AF(LED5, 0) | \
					 PIN_AFIO_AF(LED6, 0) | \
					 PIN_AFIO_AF(LED7, 0) | \
					 PIN_AFIO_AF(CAN1_TX, 9) | \
					 PIN_AFIO_AF(HW_REV_DRIVE, 0) | \
					 PIN_AFIO_AF(SPI5_SYNC, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_OUTPUT(ARMED) | \
					 PIN_MODE_ALTERNATE(SPI2_SCK) | \
					 PIN_MODE_ALTERNATE(SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(SPI2_MOSI) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE8) | \
					 PIN_MODE_ALTERNATE(RC_INPUT) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE9) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE10) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE11) | \
					 PIN_MODE_ALTERNATE(CAN1_RX) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE12) | \
					 PIN_MODE_OUTPUT(SPI_SLAVE13) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(ARMED) | \
					 PIN_OTYPE_PUSHPULL(SPI2_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MOSI) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE8) | \
					 PIN_OTYPE_PUSHPULL(RC_INPUT) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE9) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE10) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE11) | \
					 PIN_OTYPE_PUSHPULL(CAN1_RX) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE12) | \
					 PIN_OTYPE_PUSHPULL(SPI_SLAVE13) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(ARMED) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE8) | \
					 PIN_OSPEED_SPEED_HIGH(RC_INPUT) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE9) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE10) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE11) | \
					 PIN_OSPEED_SPEED_HIGH(CAN1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE12) | \
					 PIN_OSPEED_SPEED_HIGH(SPI_SLAVE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_FLOATING(ARMED) | \
					 PIN_PUPDR_FLOATING(SPI2_SCK) | \
					 PIN_PUPDR_FLOATING(SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(SPI2_MOSI) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE8) | \
					 PIN_PUPDR_FLOATING(RC_INPUT) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE9) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE10) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE11) | \
					 PIN_PUPDR_FLOATING(CAN1_RX) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE12) | \
					 PIN_PUPDR_FLOATING(SPI_SLAVE13) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(ARMED) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE8) | \
					 PIN_ODR_LEVEL_HIGH(RC_INPUT) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE9) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE10) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE11) | \
					 PIN_ODR_LEVEL_HIGH(CAN1_RX) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE12) | \
					 PIN_ODR_LEVEL_HIGH(SPI_SLAVE13) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(ARMED, 0) | \
					 PIN_AFIO_AF(SPI2_SCK, 5) | \
					 PIN_AFIO_AF(SPI2_MISO, 5) | \
					 PIN_AFIO_AF(SPI2_MOSI, 5) | \
					 PIN_AFIO_AF(SPI_SLAVE8, 0) | \
					 PIN_AFIO_AF(RC_INPUT, 3) | \
					 PIN_AFIO_AF(SPI_SLAVE9, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE10, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(SPI_SLAVE11, 0) | \
					 PIN_AFIO_AF(CAN1_RX, 9) | \
					 PIN_AFIO_AF(SPI_SLAVE12, 0) | \
					 PIN_AFIO_AF(SPI_SLAVE13, 0) | \
					 PIN_AFIO_AF(PI12, 0) | \
					 PIN_AFIO_AF(PI13, 0) | \
					 PIN_AFIO_AF(PI14, 0) | \
					 PIN_AFIO_AF(PI15, 0))

#define VAL_GPIOJ_MODER                 (PIN_MODE_INPUT(PJ00) | \
					 PIN_MODE_INPUT(PJ01) | \
					 PIN_MODE_INPUT(PJ02) | \
					 PIN_MODE_INPUT(PJ03) | \
					 PIN_MODE_INPUT(PJ04) | \
					 PIN_MODE_INPUT(PJ05) | \
					 PIN_MODE_INPUT(PJ06) | \
					 PIN_MODE_INPUT(PJ07) | \
					 PIN_MODE_INPUT(PJ08) | \
					 PIN_MODE_INPUT(PJ09) | \
					 PIN_MODE_INPUT(PJ10) | \
					 PIN_MODE_INPUT(PJ11) | \
					 PIN_MODE_INPUT(PJ12) | \
					 PIN_MODE_INPUT(PJ13) | \
					 PIN_MODE_INPUT(PJ14) | \
					 PIN_MODE_INPUT(PJ15))

#define VAL_GPIOJ_OTYPER                (PIN_OTYPE_PUSHPULL(PJ00) | \
					 PIN_OTYPE_PUSHPULL(PJ01) | \
					 PIN_OTYPE_PUSHPULL(PJ02) | \
					 PIN_OTYPE_PUSHPULL(PJ03) | \
					 PIN_OTYPE_PUSHPULL(PJ04) | \
					 PIN_OTYPE_PUSHPULL(PJ05) | \
					 PIN_OTYPE_PUSHPULL(PJ06) | \
					 PIN_OTYPE_PUSHPULL(PJ07) | \
					 PIN_OTYPE_PUSHPULL(PJ08) | \
					 PIN_OTYPE_PUSHPULL(PJ09) | \
					 PIN_OTYPE_PUSHPULL(PJ10) | \
					 PIN_OTYPE_PUSHPULL(PJ11) | \
					 PIN_OTYPE_PUSHPULL(PJ12) | \
					 PIN_OTYPE_PUSHPULL(PJ13) | \
					 PIN_OTYPE_PUSHPULL(PJ14) | \
					 PIN_OTYPE_PUSHPULL(PJ15))

#define VAL_GPIOJ_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PJ00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ15))

#define VAL_GPIOJ_PUPDR                 (PIN_PUPDR_PULLDOWN(PJ00) | \
					 PIN_PUPDR_PULLDOWN(PJ01) | \
					 PIN_PUPDR_PULLDOWN(PJ02) | \
					 PIN_PUPDR_PULLDOWN(PJ03) | \
					 PIN_PUPDR_PULLDOWN(PJ04) | \
					 PIN_PUPDR_PULLDOWN(PJ05) | \
					 PIN_PUPDR_PULLDOWN(PJ06) | \
					 PIN_PUPDR_PULLDOWN(PJ07) | \
					 PIN_PUPDR_PULLDOWN(PJ08) | \
					 PIN_PUPDR_PULLDOWN(PJ09) | \
					 PIN_PUPDR_PULLDOWN(PJ10) | \
					 PIN_PUPDR_PULLDOWN(PJ11) | \
					 PIN_PUPDR_PULLDOWN(PJ12) | \
					 PIN_PUPDR_PULLDOWN(PJ13) | \
					 PIN_PUPDR_PULLDOWN(PJ14) | \
					 PIN_PUPDR_PULLDOWN(PJ15))

#define VAL_GPIOJ_ODR                   (PIN_ODR_LEVEL_LOW(PJ00) | \
					 PIN_ODR_LEVEL_LOW(PJ01) | \
					 PIN_ODR_LEVEL_LOW(PJ02) | \
					 PIN_ODR_LEVEL_LOW(PJ03) | \
					 PIN_ODR_LEVEL_LOW(PJ04) | \
					 PIN_ODR_LEVEL_LOW(PJ05) | \
					 PIN_ODR_LEVEL_LOW(PJ06) | \
					 PIN_ODR_LEVEL_LOW(PJ07) | \
					 PIN_ODR_LEVEL_LOW(PJ08) | \
					 PIN_ODR_LEVEL_LOW(PJ09) | \
					 PIN_ODR_LEVEL_LOW(PJ10) | \
					 PIN_ODR_LEVEL_LOW(PJ11) | \
					 PIN_ODR_LEVEL_LOW(PJ12) | \
					 PIN_ODR_LEVEL_LOW(PJ13) | \
					 PIN_ODR_LEVEL_LOW(PJ14) | \
					 PIN_ODR_LEVEL_LOW(PJ15))

#define VAL_GPIOJ_AFRL			(PIN_AFIO_AF(PJ00, 0) | \
					 PIN_AFIO_AF(PJ01, 0) | \
					 PIN_AFIO_AF(PJ02, 0) | \
					 PIN_AFIO_AF(PJ03, 0) | \
					 PIN_AFIO_AF(PJ04, 0) | \
					 PIN_AFIO_AF(PJ05, 0) | \
					 PIN_AFIO_AF(PJ06, 0) | \
					 PIN_AFIO_AF(PJ07, 0))

#define VAL_GPIOJ_AFRH			(PIN_AFIO_AF(PJ08, 0) | \
					 PIN_AFIO_AF(PJ09, 0) | \
					 PIN_AFIO_AF(PJ10, 0) | \
					 PIN_AFIO_AF(PJ11, 0) | \
					 PIN_AFIO_AF(PJ12, 0) | \
					 PIN_AFIO_AF(PJ13, 0) | \
					 PIN_AFIO_AF(PJ14, 0) | \
					 PIN_AFIO_AF(PJ15, 0))

#define VAL_GPIOK_MODER                 (PIN_MODE_INPUT(PK00) | \
					 PIN_MODE_INPUT(PK01) | \
					 PIN_MODE_INPUT(PK02) | \
					 PIN_MODE_INPUT(PK03) | \
					 PIN_MODE_INPUT(PK04) | \
					 PIN_MODE_INPUT(PK05) | \
					 PIN_MODE_INPUT(PK06) | \
					 PIN_MODE_INPUT(PK07) | \
					 PIN_MODE_INPUT(PK08) | \
					 PIN_MODE_INPUT(PK09) | \
					 PIN_MODE_INPUT(PK10) | \
					 PIN_MODE_INPUT(PK11) | \
					 PIN_MODE_INPUT(PK12) | \
					 PIN_MODE_INPUT(PK13) | \
					 PIN_MODE_INPUT(PK14) | \
					 PIN_MODE_INPUT(PK15))

#define VAL_GPIOK_OTYPER                (PIN_OTYPE_PUSHPULL(PK00) | \
					 PIN_OTYPE_PUSHPULL(PK01) | \
					 PIN_OTYPE_PUSHPULL(PK02) | \
					 PIN_OTYPE_PUSHPULL(PK03) | \
					 PIN_OTYPE_PUSHPULL(PK04) | \
					 PIN_OTYPE_PUSHPULL(PK05) | \
					 PIN_OTYPE_PUSHPULL(PK06) | \
					 PIN_OTYPE_PUSHPULL(PK07) | \
					 PIN_OTYPE_PUSHPULL(PK08) | \
					 PIN_OTYPE_PUSHPULL(PK09) | \
					 PIN_OTYPE_PUSHPULL(PK10) | \
					 PIN_OTYPE_PUSHPULL(PK11) | \
					 PIN_OTYPE_PUSHPULL(PK12) | \
					 PIN_OTYPE_PUSHPULL(PK13) | \
					 PIN_OTYPE_PUSHPULL(PK14) | \
					 PIN_OTYPE_PUSHPULL(PK15))

#define VAL_GPIOK_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PK00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK15))

#define VAL_GPIOK_PUPDR                 (PIN_PUPDR_PULLDOWN(PK00) | \
					 PIN_PUPDR_PULLDOWN(PK01) | \
					 PIN_PUPDR_PULLDOWN(PK02) | \
					 PIN_PUPDR_PULLDOWN(PK03) | \
					 PIN_PUPDR_PULLDOWN(PK04) | \
					 PIN_PUPDR_PULLDOWN(PK05) | \
					 PIN_PUPDR_PULLDOWN(PK06) | \
					 PIN_PUPDR_PULLDOWN(PK07) | \
					 PIN_PUPDR_PULLDOWN(PK08) | \
					 PIN_PUPDR_PULLDOWN(PK09) | \
					 PIN_PUPDR_PULLDOWN(PK10) | \
					 PIN_PUPDR_PULLDOWN(PK11) | \
					 PIN_PUPDR_PULLDOWN(PK12) | \
					 PIN_PUPDR_PULLDOWN(PK13) | \
					 PIN_PUPDR_PULLDOWN(PK14) | \
					 PIN_PUPDR_PULLDOWN(PK15))

#define VAL_GPIOK_ODR                   (PIN_ODR_LEVEL_LOW(PK00) | \
					 PIN_ODR_LEVEL_LOW(PK01) | \
					 PIN_ODR_LEVEL_LOW(PK02) | \
					 PIN_ODR_LEVEL_LOW(PK03) | \
					 PIN_ODR_LEVEL_LOW(PK04) | \
					 PIN_ODR_LEVEL_LOW(PK05) | \
					 PIN_ODR_LEVEL_LOW(PK06) | \
					 PIN_ODR_LEVEL_LOW(PK07) | \
					 PIN_ODR_LEVEL_LOW(PK08) | \
					 PIN_ODR_LEVEL_LOW(PK09) | \
					 PIN_ODR_LEVEL_LOW(PK10) | \
					 PIN_ODR_LEVEL_LOW(PK11) | \
					 PIN_ODR_LEVEL_LOW(PK12) | \
					 PIN_ODR_LEVEL_LOW(PK13) | \
					 PIN_ODR_LEVEL_LOW(PK14) | \
					 PIN_ODR_LEVEL_LOW(PK15))

#define VAL_GPIOK_AFRL			(PIN_AFIO_AF(PK00, 0) | \
					 PIN_AFIO_AF(PK01, 0) | \
					 PIN_AFIO_AF(PK02, 0) | \
					 PIN_AFIO_AF(PK03, 0) | \
					 PIN_AFIO_AF(PK04, 0) | \
					 PIN_AFIO_AF(PK05, 0) | \
					 PIN_AFIO_AF(PK06, 0) | \
					 PIN_AFIO_AF(PK07, 0))

#define VAL_GPIOK_AFRH			(PIN_AFIO_AF(PK08, 0) | \
					 PIN_AFIO_AF(PK09, 0) | \
					 PIN_AFIO_AF(PK10, 0) | \
					 PIN_AFIO_AF(PK11, 0) | \
					 PIN_AFIO_AF(PK12, 0) | \
					 PIN_AFIO_AF(PK13, 0) | \
					 PIN_AFIO_AF(PK14, 0) | \
					 PIN_AFIO_AF(PK15, 0))

#define AF_SPI1_MISO                     5U
#define AF_LINE_SPI1_MISO                5U
#define AF_CAN3_RX                       11U
#define AF_LINE_CAN3_RX                  11U
#define AF_SERVO2                        1U
#define AF_LINE_SERVO2                   1U
#define AF_USB_DM                        10U
#define AF_LINE_USB_DM                   10U
#define AF_USB_DP                        10U
#define AF_LINE_USB_DP                   10U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_CAN3_TX                       11U
#define AF_LINE_CAN3_TX                  11U
#define AF_SPI6_MOSI                     8U
#define AF_LINE_SPI6_MOSI                8U
#define AF_USART1_TX                     7U
#define AF_LINE_USART1_TX                7U
#define AF_USART1_RX                     7U
#define AF_LINE_USART1_RX                7U
#define AF_I2C1_SCL                      4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_I2C1_SDA                      4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_CAN2_RX                       9U
#define AF_LINE_CAN2_RX                  9U
#define AF_CAN2_TX                       9U
#define AF_LINE_CAN2_TX                  9U
#define AF_SDIO_D0                       12U
#define AF_LINE_SDIO_D0                  12U
#define AF_SDIO_D1                       12U
#define AF_LINE_SDIO_D1                  12U
#define AF_SDIO_D2                       12U
#define AF_LINE_SDIO_D2                  12U
#define AF_SDIO_D3                       12U
#define AF_LINE_SDIO_D3                  12U
#define AF_SDIO_CK                       12U
#define AF_LINE_SDIO_CK                  12U
#define AF_OSC32_IN                      0U
#define AF_LINE_OSC32_IN                 0U
#define AF_OSC32_OUT                     0U
#define AF_LINE_OSC32_OUT                0U
#define AF_UART4_RX                      8U
#define AF_LINE_UART4_RX                 8U
#define AF_UART4_TX                      8U
#define AF_LINE_UART4_TX                 8U
#define AF_SDIO_CMD                      12U
#define AF_LINE_SDIO_CMD                 12U
#define AF_UART2_TX                      7U
#define AF_LINE_UART2_TX                 7U
#define AF_UART2_RX                      7U
#define AF_LINE_UART2_RX                 7U
#define AF_SPI1_MOSI                     5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_UART3_TX                      7U
#define AF_LINE_UART3_TX                 7U
#define AF_UART3_RX                      7U
#define AF_LINE_UART3_RX                 7U
#define AF_SERVO5                        2U
#define AF_LINE_SERVO5                   2U
#define AF_SERVO6                        2U
#define AF_LINE_SERVO6                   2U
#define AF_UART8_RX                      8U
#define AF_LINE_UART8_RX                 8U
#define AF_UART8_TX                      8U
#define AF_LINE_UART8_TX                 8U
#define AF_SPI4_SCK                      5U
#define AF_LINE_SPI4_SCK                 5U
#define AF_SPI4_MOSI                     5U
#define AF_LINE_SPI4_MOSI                5U
#define AF_UART7_TX                      8U
#define AF_LINE_UART7_TX                 8U
#define AF_SERVO4                        1U
#define AF_LINE_SERVO4                   1U
#define AF_SERVO3                        1U
#define AF_LINE_SERVO3                   1U
#define AF_SPI4_MISO                     5U
#define AF_LINE_SPI4_MISO                5U
#define AF_SERVO1                        1U
#define AF_LINE_SERVO1                   1U
#define AF_I2C2_SDA                      4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_I2C2_SCL                      4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_UART7_RX                      8U
#define AF_LINE_UART7_RX                 8U
#define AF_SPI5_SCK                      5U
#define AF_LINE_SPI5_SCK                 5U
#define AF_SPI5_MISO                     5U
#define AF_LINE_SPI5_MISO                5U
#define AF_SPI5_MOSI                     5U
#define AF_LINE_SPI5_MOSI                5U
#define AF_I2C4_SCL                      4U
#define AF_LINE_I2C4_SCL                 4U
#define AF_I2C4_SDA                      4U
#define AF_LINE_I2C4_SDA                 4U
#define AF_USART6_RX                     8U
#define AF_LINE_USART6_RX                8U
#define AF_SPI1_SCK                      5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_SPI6_MISO                     5U
#define AF_LINE_SPI6_MISO                5U
#define AF_SPI6_SCK                      5U
#define AF_LINE_SPI6_SCK                 5U
#define AF_USART6_TX                     8U
#define AF_LINE_USART6_TX                8U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U
#define AF_SERVO7                        9U
#define AF_LINE_SERVO7                   9U
#define AF_I2C3_SCL                      4U
#define AF_LINE_I2C3_SCL                 4U
#define AF_I2C3_SDA                      4U
#define AF_LINE_I2C3_SDA                 4U
#define AF_SERVO8                        9U
#define AF_LINE_SERVO8                   9U
#define AF_CAN1_TX                       9U
#define AF_LINE_CAN1_TX                  9U
#define AF_SPI2_SCK                      5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_SPI2_MISO                     5U
#define AF_LINE_SPI2_MISO                5U
#define AF_SPI2_MOSI                     5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_RC_INPUT                      3U
#define AF_LINE_RC_INPUT                 3U
#define AF_CAN1_RX                       9U
#define AF_LINE_CAN1_RX                  9U



#define BOARD_GROUP_FOREACH(line, group) \
  for (ioline_t i=0, lines[] = {group}, line = lines[i]; (i < group ## _SIZE) && (line=lines[i]); i++)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

