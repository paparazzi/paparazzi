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
#define	PA00_ADC1                      0U
#define	PA01_ADC2                      1U
#define	PA02_ADC3                      2U
#define	PA03_ADC4                      3U
#define	PA04_ADC1_SPARE2               4U
#define	PA05_FMU_CAP1                  5U
#define	PA06_SPI1_MISO                 6U
#define	PA07_HEATER                    7U
#define	PA08_CAN3_RX                   8U
#define	PA09_USB_VBUS                  9U
#define	PA10_SERVO2                    10U
#define	PA11_USB_DM                    11U
#define	PA12_USB_DP                    12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15_CAN3_TX                   15U

#define	PB00_RSSI_IN                   0U
#define	PB01_LED1                      1U
#define	PB02                           2U
#define	PB03_FMU_CAP2                  3U
#define	PB04_DRDY1_ICM20689            4U
#define	PB05_SPI6_MOSI                 5U
#define	PB06_USART1_TX                 6U
#define	PB07_USART1_RX                 7U
#define	PB08_I2C1_SCL                  8U
#define	PB09_I2C1_SDA                  9U
#define	PB10_SPI5_RESET                10U
#define	PB11_FMU_CAP3                  11U
#define	PB12_CAN2_RX                   12U
#define	PB13_CAN2_TX                   13U
#define	PB14_DRDY2_BMI055_GYRO         14U
#define	PB15_DRDY2_BMI055_ACC          15U

#define	PC00_SCALED_V5                 0U
#define	PC01_SCALED_3V3_SENSORS        1U
#define	PC02_HW_VER_SENSE              2U
#define	PC03_HW_REV_SENSE              3U
#define	PC04_ADC1_SPARE1               4U
#define	PC05_DRDY4_ICM20602            5U
#define	PC06_LED2                      6U
#define	PC07_LED3                      7U
#define	PC08_SDIO_D0                   8U
#define	PC09_SDIO_D1                   9U
#define	PC10_SDIO_D2                   10U
#define	PC11_SDIO_D3                   11U
#define	PC12_SDIO_CK                   12U
#define	PC13_DRDY5_BMI055_GYRO         13U
#define	PC14_OSC32_IN                  14U
#define	PC15_OSC32_OUT                 15U

#define	PD00_UART4_RX                  0U
#define	PD01_UART4_TX                  1U
#define	PD02_SDIO_CMD                  2U
#define	PD03_UART2_CTS                 3U
#define	PD04_UART2_RTS                 4U
#define	PD05_UART2_TX                  5U
#define	PD06_UART2_RX                  6U
#define	PD07_SPI1_MOSI                 7U
#define	PD08_UART3_TX                  8U
#define	PD09_UART3_RX                  9U
#define	PD10_DRDY6_BMI055_ACC          10U
#define	PD11_UART3_CTS                 11U
#define	PD12_UART3_RTS                 12U
#define	PD13_SERVO5                    13U
#define	PD14_SERVO6                    14U
#define	PD15_DRDY7_EXTERNAL1           15U

#define	PE00_UART8_RX                  0U
#define	PE01_UART8_TX                  1U
#define	PE02_SPI4_SCK                  2U
#define	PE03_V3V3_SENSORS_EN           3U
#define	PE04_V3V3_SPEKTRUM_EN          4U
#define	PE05_BUZZER                    5U
#define	PE06_SPI4_MOSI                 6U
#define	PE07_DRDY8                     7U
#define	PE08_UART7_TX                  8U
#define	PE09_SERVO4                    9U
#define	PE10_SAFETY_SWITCH_IN          10U
#define	PE11_SERVO3                    11U
#define	PE12_LED4                      12U
#define	PE13_SPI4_MISO                 13U
#define	PE14_SERVO1                    14U
#define	PE15_V5V_PERIPH_OC             15U

#define	PF00_I2C2_SDA                  0U
#define	PF01_I2C2_SCL                  1U
#define	PF02_SPI_SLAVE0                2U
#define	PF03_SPI_SLAVE1                3U
#define	PF04_SPI_SLAVE2                4U
#define	PF05_SPI_SLAVE3                5U
#define	PF06_UART7_RX                  6U
#define	PF07_SPI5_SCK                  7U
#define	PF08_SPI5_MISO                 8U
#define	PF09_SPI5_MOSI                 9U
#define	PF10_SPI_SLAVE4                10U
#define	PF11_SPI_SLAVE5                11U
#define	PF12_V5V_HIPOWER_EN            12U
#define	PF13_V5V_HIPOWER_OC            13U
#define	PF14_I2C4_SCL                  14U
#define	PF15_I2C4_SDA                  15U

#define	PG00_HW_VER_DRIVE              0U
#define	PG01_POWER_IN_A                1U
#define	PG02_POWER_IN_B                2U
#define	PG03_POWER_IN_C                3U
#define	PG04_V5V_PERIPH_EN             4U
#define	PG05_V5V_RC_EN                 5U
#define	PG06_V5V_WIFI_EN               6U
#define	PG07_V3V3_SD_CARD_EN           7U
#define	PG08_USART6_RTS                8U
#define	PG09_USART6_RX                 9U
#define	PG10_SPI_SLAVE6                10U
#define	PG11_SPI1_SCK                  11U
#define	PG12_SPI6_MISO                 12U
#define	PG13_SPI6_SCK                  13U
#define	PG14_USART6_TX                 14U
#define	PG15_USART6_CTS                15U

#define	PH00_OSC_IN                    0U
#define	PH01_OSC_OUT                   1U
#define	PH02_CAN1_SILENT_S0            2U
#define	PH03_CAN2_SILENT_S1            3U
#define	PH04_CAN3_SILENT_S2            4U
#define	PH05_SPI_SLAVE7                5U
#define	PH06_SERVO7                    6U
#define	PH07_I2C3_SCL                  7U
#define	PH08_I2C3_SDA                  8U
#define	PH09_SERVO8                    9U
#define	PH10_LED5                      10U
#define	PH11_LED6                      11U
#define	PH12_LED7                      12U
#define	PH13_CAN1_TX                   13U
#define	PH14_HW_REV_DRIVE              14U
#define	PH15_SPI5_SYNC                 15U

#define	PI00_ARMED                     0U
#define	PI01_SPI2_SCK                  1U
#define	PI02_SPI2_MISO                 2U
#define	PI03_SPI2_MOSI                 3U
#define	PI04_SPI_SLAVE8                4U
#define	PI05_RC_INPUT                  5U
#define	PI06_SPI_SLAVE9                6U
#define	PI07_SPI_SLAVE10               7U
#define	PI08_SPI_SLAVE11               8U
#define	PI09_CAN1_RX                   9U
#define	PI10_SPI_SLAVE12               10U
#define	PI11_SPI_SLAVE13               11U
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
#define	LINE_ADC1_SPARE2               PAL_LINE(GPIOA, 4U)
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
#define	LINE_ADC1_SPARE1               PAL_LINE(GPIOC, 4U)
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

#define VAL_GPIOA_MODER                 (PIN_MODE_ANALOG(PA00_ADC1) | \
					 PIN_MODE_ANALOG(PA01_ADC2) | \
					 PIN_MODE_ANALOG(PA02_ADC3) | \
					 PIN_MODE_ANALOG(PA03_ADC4) | \
					 PIN_MODE_ANALOG(PA04_ADC1_SPARE2) | \
					 PIN_MODE_INPUT(PA05_FMU_CAP1) | \
					 PIN_MODE_ALTERNATE(PA06_SPI1_MISO) | \
					 PIN_MODE_INPUT(PA07_HEATER) | \
					 PIN_MODE_ALTERNATE(PA08_CAN3_RX) | \
					 PIN_MODE_INPUT(PA09_USB_VBUS) | \
					 PIN_MODE_ALTERNATE(PA10_SERVO2) | \
					 PIN_MODE_ALTERNATE(PA11_USB_DM) | \
					 PIN_MODE_ALTERNATE(PA12_USB_DP) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_ALTERNATE(PA15_CAN3_TX))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_ADC1) | \
					 PIN_OTYPE_PUSHPULL(PA01_ADC2) | \
					 PIN_OTYPE_PUSHPULL(PA02_ADC3) | \
					 PIN_OTYPE_PUSHPULL(PA03_ADC4) | \
					 PIN_OTYPE_PUSHPULL(PA04_ADC1_SPARE2) | \
					 PIN_OTYPE_OPENDRAIN(PA05_FMU_CAP1) | \
					 PIN_OTYPE_PUSHPULL(PA06_SPI1_MISO) | \
					 PIN_OTYPE_OPENDRAIN(PA07_HEATER) | \
					 PIN_OTYPE_PUSHPULL(PA08_CAN3_RX) | \
					 PIN_OTYPE_OPENDRAIN(PA09_USB_VBUS) | \
					 PIN_OTYPE_PUSHPULL(PA10_SERVO2) | \
					 PIN_OTYPE_PUSHPULL(PA11_USB_DM) | \
					 PIN_OTYPE_PUSHPULL(PA12_USB_DP) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15_CAN3_TX))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PA00_ADC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA01_ADC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA02_ADC3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA03_ADC4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04_ADC1_SPARE2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA05_FMU_CAP1) | \
					 PIN_OSPEED_SPEED_HIGH(PA06_SPI1_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA07_HEATER) | \
					 PIN_OSPEED_SPEED_HIGH(PA08_CAN3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA09_USB_VBUS) | \
					 PIN_OSPEED_SPEED_HIGH(PA10_SERVO2) | \
					 PIN_OSPEED_SPEED_HIGH(PA11_USB_DM) | \
					 PIN_OSPEED_SPEED_HIGH(PA12_USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(PA15_CAN3_TX))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_ADC1) | \
					 PIN_PUPDR_FLOATING(PA01_ADC2) | \
					 PIN_PUPDR_FLOATING(PA02_ADC3) | \
					 PIN_PUPDR_FLOATING(PA03_ADC4) | \
					 PIN_PUPDR_FLOATING(PA04_ADC1_SPARE2) | \
					 PIN_PUPDR_PULLDOWN(PA05_FMU_CAP1) | \
					 PIN_PUPDR_FLOATING(PA06_SPI1_MISO) | \
					 PIN_PUPDR_PULLDOWN(PA07_HEATER) | \
					 PIN_PUPDR_FLOATING(PA08_CAN3_RX) | \
					 PIN_PUPDR_PULLDOWN(PA09_USB_VBUS) | \
					 PIN_PUPDR_FLOATING(PA10_SERVO2) | \
					 PIN_PUPDR_FLOATING(PA11_USB_DM) | \
					 PIN_PUPDR_FLOATING(PA12_USB_DP) | \
					 PIN_PUPDR_FLOATING(PA13_SWDIO) | \
					 PIN_PUPDR_FLOATING(PA14_SWCLK) | \
					 PIN_PUPDR_FLOATING(PA15_CAN3_TX))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(PA00_ADC1) | \
					 PIN_ODR_LEVEL_LOW(PA01_ADC2) | \
					 PIN_ODR_LEVEL_LOW(PA02_ADC3) | \
					 PIN_ODR_LEVEL_LOW(PA03_ADC4) | \
					 PIN_ODR_LEVEL_LOW(PA04_ADC1_SPARE2) | \
					 PIN_ODR_LEVEL_HIGH(PA05_FMU_CAP1) | \
					 PIN_ODR_LEVEL_HIGH(PA06_SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PA07_HEATER) | \
					 PIN_ODR_LEVEL_HIGH(PA08_CAN3_RX) | \
					 PIN_ODR_LEVEL_LOW(PA09_USB_VBUS) | \
					 PIN_ODR_LEVEL_LOW(PA10_SERVO2) | \
					 PIN_ODR_LEVEL_HIGH(PA11_USB_DM) | \
					 PIN_ODR_LEVEL_HIGH(PA12_USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(PA15_CAN3_TX))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_ADC1, 0) | \
					 PIN_AFIO_AF(PA01_ADC2, 0) | \
					 PIN_AFIO_AF(PA02_ADC3, 0) | \
					 PIN_AFIO_AF(PA03_ADC4, 0) | \
					 PIN_AFIO_AF(PA04_ADC1_SPARE2, 0) | \
					 PIN_AFIO_AF(PA05_FMU_CAP1, 0) | \
					 PIN_AFIO_AF(PA06_SPI1_MISO, 5) | \
					 PIN_AFIO_AF(PA07_HEATER, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_CAN3_RX, 11) | \
					 PIN_AFIO_AF(PA09_USB_VBUS, 0) | \
					 PIN_AFIO_AF(PA10_SERVO2, 1) | \
					 PIN_AFIO_AF(PA11_USB_DM, 10) | \
					 PIN_AFIO_AF(PA12_USB_DP, 10) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15_CAN3_TX, 11))

#define VAL_GPIOB_MODER                 (PIN_MODE_ANALOG(PB00_RSSI_IN) | \
					 PIN_MODE_OUTPUT(PB01_LED1) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_INPUT(PB03_FMU_CAP2) | \
					 PIN_MODE_INPUT(PB04_DRDY1_ICM20689) | \
					 PIN_MODE_ALTERNATE(PB05_SPI6_MOSI) | \
					 PIN_MODE_ALTERNATE(PB06_USART1_TX) | \
					 PIN_MODE_ALTERNATE(PB07_USART1_RX) | \
					 PIN_MODE_ALTERNATE(PB08_I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(PB09_I2C1_SDA) | \
					 PIN_MODE_INPUT(PB10_SPI5_RESET) | \
					 PIN_MODE_INPUT(PB11_FMU_CAP3) | \
					 PIN_MODE_ALTERNATE(PB12_CAN2_RX) | \
					 PIN_MODE_ALTERNATE(PB13_CAN2_TX) | \
					 PIN_MODE_INPUT(PB14_DRDY2_BMI055_GYRO) | \
					 PIN_MODE_INPUT(PB15_DRDY2_BMI055_ACC))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00_RSSI_IN) | \
					 PIN_OTYPE_PUSHPULL(PB01_LED1) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_OPENDRAIN(PB03_FMU_CAP2) | \
					 PIN_OTYPE_OPENDRAIN(PB04_DRDY1_ICM20689) | \
					 PIN_OTYPE_PUSHPULL(PB05_SPI6_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PB06_USART1_TX) | \
					 PIN_OTYPE_PUSHPULL(PB07_USART1_RX) | \
					 PIN_OTYPE_OPENDRAIN(PB08_I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PB09_I2C1_SDA) | \
					 PIN_OTYPE_OPENDRAIN(PB10_SPI5_RESET) | \
					 PIN_OTYPE_OPENDRAIN(PB11_FMU_CAP3) | \
					 PIN_OTYPE_PUSHPULL(PB12_CAN2_RX) | \
					 PIN_OTYPE_PUSHPULL(PB13_CAN2_TX) | \
					 PIN_OTYPE_OPENDRAIN(PB14_DRDY2_BMI055_GYRO) | \
					 PIN_OTYPE_OPENDRAIN(PB15_DRDY2_BMI055_ACC))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PB00_RSSI_IN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB01_LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB03_FMU_CAP2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB04_DRDY1_ICM20689) | \
					 PIN_OSPEED_SPEED_HIGH(PB05_SPI6_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_USART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PB07_USART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB08_I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PB09_I2C1_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB10_SPI5_RESET) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB11_FMU_CAP3) | \
					 PIN_OSPEED_SPEED_HIGH(PB12_CAN2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB13_CAN2_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB14_DRDY2_BMI055_GYRO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB15_DRDY2_BMI055_ACC))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PB00_RSSI_IN) | \
					 PIN_PUPDR_FLOATING(PB01_LED1) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_PULLDOWN(PB03_FMU_CAP2) | \
					 PIN_PUPDR_PULLDOWN(PB04_DRDY1_ICM20689) | \
					 PIN_PUPDR_FLOATING(PB05_SPI6_MOSI) | \
					 PIN_PUPDR_FLOATING(PB06_USART1_TX) | \
					 PIN_PUPDR_FLOATING(PB07_USART1_RX) | \
					 PIN_PUPDR_PULLUP(PB08_I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(PB09_I2C1_SDA) | \
					 PIN_PUPDR_PULLDOWN(PB10_SPI5_RESET) | \
					 PIN_PUPDR_PULLDOWN(PB11_FMU_CAP3) | \
					 PIN_PUPDR_FLOATING(PB12_CAN2_RX) | \
					 PIN_PUPDR_FLOATING(PB13_CAN2_TX) | \
					 PIN_PUPDR_PULLDOWN(PB14_DRDY2_BMI055_GYRO) | \
					 PIN_PUPDR_PULLDOWN(PB15_DRDY2_BMI055_ACC))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00_RSSI_IN) | \
					 PIN_ODR_LEVEL_LOW(PB01_LED1) | \
					 PIN_ODR_LEVEL_LOW(PB02) | \
					 PIN_ODR_LEVEL_HIGH(PB03_FMU_CAP2) | \
					 PIN_ODR_LEVEL_HIGH(PB04_DRDY1_ICM20689) | \
					 PIN_ODR_LEVEL_HIGH(PB05_SPI6_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PB06_USART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB07_USART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB08_I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PB09_I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PB10_SPI5_RESET) | \
					 PIN_ODR_LEVEL_HIGH(PB11_FMU_CAP3) | \
					 PIN_ODR_LEVEL_HIGH(PB12_CAN2_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB13_CAN2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB14_DRDY2_BMI055_GYRO) | \
					 PIN_ODR_LEVEL_HIGH(PB15_DRDY2_BMI055_ACC))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_RSSI_IN, 0) | \
					 PIN_AFIO_AF(PB01_LED1, 0) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(PB03_FMU_CAP2, 0) | \
					 PIN_AFIO_AF(PB04_DRDY1_ICM20689, 0) | \
					 PIN_AFIO_AF(PB05_SPI6_MOSI, 8) | \
					 PIN_AFIO_AF(PB06_USART1_TX, 7) | \
					 PIN_AFIO_AF(PB07_USART1_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08_I2C1_SCL, 4) | \
					 PIN_AFIO_AF(PB09_I2C1_SDA, 4) | \
					 PIN_AFIO_AF(PB10_SPI5_RESET, 0) | \
					 PIN_AFIO_AF(PB11_FMU_CAP3, 0) | \
					 PIN_AFIO_AF(PB12_CAN2_RX, 9) | \
					 PIN_AFIO_AF(PB13_CAN2_TX, 9) | \
					 PIN_AFIO_AF(PB14_DRDY2_BMI055_GYRO, 0) | \
					 PIN_AFIO_AF(PB15_DRDY2_BMI055_ACC, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(PC00_SCALED_V5) | \
					 PIN_MODE_ANALOG(PC01_SCALED_3V3_SENSORS) | \
					 PIN_MODE_ANALOG(PC02_HW_VER_SENSE) | \
					 PIN_MODE_ANALOG(PC03_HW_REV_SENSE) | \
					 PIN_MODE_ANALOG(PC04_ADC1_SPARE1) | \
					 PIN_MODE_INPUT(PC05_DRDY4_ICM20602) | \
					 PIN_MODE_OUTPUT(PC06_LED2) | \
					 PIN_MODE_OUTPUT(PC07_LED3) | \
					 PIN_MODE_ALTERNATE(PC08_SDIO_D0) | \
					 PIN_MODE_ALTERNATE(PC09_SDIO_D1) | \
					 PIN_MODE_ALTERNATE(PC10_SDIO_D2) | \
					 PIN_MODE_ALTERNATE(PC11_SDIO_D3) | \
					 PIN_MODE_ALTERNATE(PC12_SDIO_CK) | \
					 PIN_MODE_INPUT(PC13_DRDY5_BMI055_GYRO) | \
					 PIN_MODE_ALTERNATE(PC14_OSC32_IN) | \
					 PIN_MODE_ALTERNATE(PC15_OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(PC00_SCALED_V5) | \
					 PIN_OTYPE_PUSHPULL(PC01_SCALED_3V3_SENSORS) | \
					 PIN_OTYPE_PUSHPULL(PC02_HW_VER_SENSE) | \
					 PIN_OTYPE_PUSHPULL(PC03_HW_REV_SENSE) | \
					 PIN_OTYPE_PUSHPULL(PC04_ADC1_SPARE1) | \
					 PIN_OTYPE_OPENDRAIN(PC05_DRDY4_ICM20602) | \
					 PIN_OTYPE_PUSHPULL(PC06_LED2) | \
					 PIN_OTYPE_PUSHPULL(PC07_LED3) | \
					 PIN_OTYPE_PUSHPULL(PC08_SDIO_D0) | \
					 PIN_OTYPE_PUSHPULL(PC09_SDIO_D1) | \
					 PIN_OTYPE_PUSHPULL(PC10_SDIO_D2) | \
					 PIN_OTYPE_PUSHPULL(PC11_SDIO_D3) | \
					 PIN_OTYPE_PUSHPULL(PC12_SDIO_CK) | \
					 PIN_OTYPE_OPENDRAIN(PC13_DRDY5_BMI055_GYRO) | \
					 PIN_OTYPE_PUSHPULL(PC14_OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(PC15_OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00_SCALED_V5) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01_SCALED_3V3_SENSORS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02_HW_VER_SENSE) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03_HW_REV_SENSE) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04_ADC1_SPARE1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05_DRDY4_ICM20602) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC06_LED2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC07_LED3) | \
					 PIN_OSPEED_SPEED_HIGH(PC08_SDIO_D0) | \
					 PIN_OSPEED_SPEED_HIGH(PC09_SDIO_D1) | \
					 PIN_OSPEED_SPEED_HIGH(PC10_SDIO_D2) | \
					 PIN_OSPEED_SPEED_HIGH(PC11_SDIO_D3) | \
					 PIN_OSPEED_SPEED_HIGH(PC12_SDIO_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13_DRDY5_BMI055_GYRO) | \
					 PIN_OSPEED_SPEED_HIGH(PC14_OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PC15_OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(PC00_SCALED_V5) | \
					 PIN_PUPDR_FLOATING(PC01_SCALED_3V3_SENSORS) | \
					 PIN_PUPDR_FLOATING(PC02_HW_VER_SENSE) | \
					 PIN_PUPDR_FLOATING(PC03_HW_REV_SENSE) | \
					 PIN_PUPDR_FLOATING(PC04_ADC1_SPARE1) | \
					 PIN_PUPDR_PULLDOWN(PC05_DRDY4_ICM20602) | \
					 PIN_PUPDR_FLOATING(PC06_LED2) | \
					 PIN_PUPDR_FLOATING(PC07_LED3) | \
					 PIN_PUPDR_PULLUP(PC08_SDIO_D0) | \
					 PIN_PUPDR_PULLUP(PC09_SDIO_D1) | \
					 PIN_PUPDR_PULLUP(PC10_SDIO_D2) | \
					 PIN_PUPDR_PULLUP(PC11_SDIO_D3) | \
					 PIN_PUPDR_PULLUP(PC12_SDIO_CK) | \
					 PIN_PUPDR_PULLDOWN(PC13_DRDY5_BMI055_GYRO) | \
					 PIN_PUPDR_FLOATING(PC14_OSC32_IN) | \
					 PIN_PUPDR_FLOATING(PC15_OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(PC00_SCALED_V5) | \
					 PIN_ODR_LEVEL_LOW(PC01_SCALED_3V3_SENSORS) | \
					 PIN_ODR_LEVEL_LOW(PC02_HW_VER_SENSE) | \
					 PIN_ODR_LEVEL_LOW(PC03_HW_REV_SENSE) | \
					 PIN_ODR_LEVEL_LOW(PC04_ADC1_SPARE1) | \
					 PIN_ODR_LEVEL_HIGH(PC05_DRDY4_ICM20602) | \
					 PIN_ODR_LEVEL_LOW(PC06_LED2) | \
					 PIN_ODR_LEVEL_LOW(PC07_LED3) | \
					 PIN_ODR_LEVEL_HIGH(PC08_SDIO_D0) | \
					 PIN_ODR_LEVEL_HIGH(PC09_SDIO_D1) | \
					 PIN_ODR_LEVEL_HIGH(PC10_SDIO_D2) | \
					 PIN_ODR_LEVEL_HIGH(PC11_SDIO_D3) | \
					 PIN_ODR_LEVEL_HIGH(PC12_SDIO_CK) | \
					 PIN_ODR_LEVEL_HIGH(PC13_DRDY5_BMI055_GYRO) | \
					 PIN_ODR_LEVEL_HIGH(PC14_OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(PC15_OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00_SCALED_V5, 0) | \
					 PIN_AFIO_AF(PC01_SCALED_3V3_SENSORS, 0) | \
					 PIN_AFIO_AF(PC02_HW_VER_SENSE, 0) | \
					 PIN_AFIO_AF(PC03_HW_REV_SENSE, 0) | \
					 PIN_AFIO_AF(PC04_ADC1_SPARE1, 0) | \
					 PIN_AFIO_AF(PC05_DRDY4_ICM20602, 0) | \
					 PIN_AFIO_AF(PC06_LED2, 0) | \
					 PIN_AFIO_AF(PC07_LED3, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08_SDIO_D0, 12) | \
					 PIN_AFIO_AF(PC09_SDIO_D1, 12) | \
					 PIN_AFIO_AF(PC10_SDIO_D2, 12) | \
					 PIN_AFIO_AF(PC11_SDIO_D3, 12) | \
					 PIN_AFIO_AF(PC12_SDIO_CK, 12) | \
					 PIN_AFIO_AF(PC13_DRDY5_BMI055_GYRO, 0) | \
					 PIN_AFIO_AF(PC14_OSC32_IN, 0) | \
					 PIN_AFIO_AF(PC15_OSC32_OUT, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(PD00_UART4_RX) | \
					 PIN_MODE_ALTERNATE(PD01_UART4_TX) | \
					 PIN_MODE_ALTERNATE(PD02_SDIO_CMD) | \
					 PIN_MODE_INPUT(PD03_UART2_CTS) | \
					 PIN_MODE_INPUT(PD04_UART2_RTS) | \
					 PIN_MODE_ALTERNATE(PD05_UART2_TX) | \
					 PIN_MODE_ALTERNATE(PD06_UART2_RX) | \
					 PIN_MODE_ALTERNATE(PD07_SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(PD08_UART3_TX) | \
					 PIN_MODE_ALTERNATE(PD09_UART3_RX) | \
					 PIN_MODE_INPUT(PD10_DRDY6_BMI055_ACC) | \
					 PIN_MODE_INPUT(PD11_UART3_CTS) | \
					 PIN_MODE_INPUT(PD12_UART3_RTS) | \
					 PIN_MODE_ALTERNATE(PD13_SERVO5) | \
					 PIN_MODE_ALTERNATE(PD14_SERVO6) | \
					 PIN_MODE_INPUT(PD15_DRDY7_EXTERNAL1))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00_UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(PD01_UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(PD02_SDIO_CMD) | \
					 PIN_OTYPE_OPENDRAIN(PD03_UART2_CTS) | \
					 PIN_OTYPE_OPENDRAIN(PD04_UART2_RTS) | \
					 PIN_OTYPE_PUSHPULL(PD05_UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(PD06_UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(PD07_SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PD08_UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(PD09_UART3_RX) | \
					 PIN_OTYPE_OPENDRAIN(PD10_DRDY6_BMI055_ACC) | \
					 PIN_OTYPE_OPENDRAIN(PD11_UART3_CTS) | \
					 PIN_OTYPE_OPENDRAIN(PD12_UART3_RTS) | \
					 PIN_OTYPE_PUSHPULL(PD13_SERVO5) | \
					 PIN_OTYPE_PUSHPULL(PD14_SERVO6) | \
					 PIN_OTYPE_OPENDRAIN(PD15_DRDY7_EXTERNAL1))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PD00_UART4_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PD01_UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD02_SDIO_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03_UART2_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04_UART2_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(PD05_UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD06_UART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PD07_SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PD08_UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD09_UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10_DRDY6_BMI055_ACC) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11_UART3_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12_UART3_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(PD13_SERVO5) | \
					 PIN_OSPEED_SPEED_HIGH(PD14_SERVO6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15_DRDY7_EXTERNAL1))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(PD00_UART4_RX) | \
					 PIN_PUPDR_FLOATING(PD01_UART4_TX) | \
					 PIN_PUPDR_PULLUP(PD02_SDIO_CMD) | \
					 PIN_PUPDR_PULLDOWN(PD03_UART2_CTS) | \
					 PIN_PUPDR_PULLDOWN(PD04_UART2_RTS) | \
					 PIN_PUPDR_FLOATING(PD05_UART2_TX) | \
					 PIN_PUPDR_FLOATING(PD06_UART2_RX) | \
					 PIN_PUPDR_FLOATING(PD07_SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(PD08_UART3_TX) | \
					 PIN_PUPDR_FLOATING(PD09_UART3_RX) | \
					 PIN_PUPDR_PULLDOWN(PD10_DRDY6_BMI055_ACC) | \
					 PIN_PUPDR_PULLDOWN(PD11_UART3_CTS) | \
					 PIN_PUPDR_PULLDOWN(PD12_UART3_RTS) | \
					 PIN_PUPDR_FLOATING(PD13_SERVO5) | \
					 PIN_PUPDR_FLOATING(PD14_SERVO6) | \
					 PIN_PUPDR_PULLDOWN(PD15_DRDY7_EXTERNAL1))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(PD00_UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD01_UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD02_SDIO_CMD) | \
					 PIN_ODR_LEVEL_HIGH(PD03_UART2_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PD04_UART2_RTS) | \
					 PIN_ODR_LEVEL_HIGH(PD05_UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD06_UART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD07_SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PD08_UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD09_UART3_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD10_DRDY6_BMI055_ACC) | \
					 PIN_ODR_LEVEL_HIGH(PD11_UART3_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PD12_UART3_RTS) | \
					 PIN_ODR_LEVEL_LOW(PD13_SERVO5) | \
					 PIN_ODR_LEVEL_LOW(PD14_SERVO6) | \
					 PIN_ODR_LEVEL_HIGH(PD15_DRDY7_EXTERNAL1))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00_UART4_RX, 8) | \
					 PIN_AFIO_AF(PD01_UART4_TX, 8) | \
					 PIN_AFIO_AF(PD02_SDIO_CMD, 12) | \
					 PIN_AFIO_AF(PD03_UART2_CTS, 0) | \
					 PIN_AFIO_AF(PD04_UART2_RTS, 0) | \
					 PIN_AFIO_AF(PD05_UART2_TX, 7) | \
					 PIN_AFIO_AF(PD06_UART2_RX, 7) | \
					 PIN_AFIO_AF(PD07_SPI1_MOSI, 5))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08_UART3_TX, 7) | \
					 PIN_AFIO_AF(PD09_UART3_RX, 7) | \
					 PIN_AFIO_AF(PD10_DRDY6_BMI055_ACC, 0) | \
					 PIN_AFIO_AF(PD11_UART3_CTS, 0) | \
					 PIN_AFIO_AF(PD12_UART3_RTS, 0) | \
					 PIN_AFIO_AF(PD13_SERVO5, 2) | \
					 PIN_AFIO_AF(PD14_SERVO6, 2) | \
					 PIN_AFIO_AF(PD15_DRDY7_EXTERNAL1, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(PE00_UART8_RX) | \
					 PIN_MODE_ALTERNATE(PE01_UART8_TX) | \
					 PIN_MODE_ALTERNATE(PE02_SPI4_SCK) | \
					 PIN_MODE_OUTPUT(PE03_V3V3_SENSORS_EN) | \
					 PIN_MODE_OUTPUT(PE04_V3V3_SPEKTRUM_EN) | \
					 PIN_MODE_INPUT(PE05_BUZZER) | \
					 PIN_MODE_ALTERNATE(PE06_SPI4_MOSI) | \
					 PIN_MODE_INPUT(PE07_DRDY8) | \
					 PIN_MODE_ALTERNATE(PE08_UART7_TX) | \
					 PIN_MODE_ALTERNATE(PE09_SERVO4) | \
					 PIN_MODE_INPUT(PE10_SAFETY_SWITCH_IN) | \
					 PIN_MODE_ALTERNATE(PE11_SERVO3) | \
					 PIN_MODE_OUTPUT(PE12_LED4) | \
					 PIN_MODE_ALTERNATE(PE13_SPI4_MISO) | \
					 PIN_MODE_ALTERNATE(PE14_SERVO1) | \
					 PIN_MODE_INPUT(PE15_V5V_PERIPH_OC))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00_UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(PE01_UART8_TX) | \
					 PIN_OTYPE_PUSHPULL(PE02_SPI4_SCK) | \
					 PIN_OTYPE_PUSHPULL(PE03_V3V3_SENSORS_EN) | \
					 PIN_OTYPE_PUSHPULL(PE04_V3V3_SPEKTRUM_EN) | \
					 PIN_OTYPE_OPENDRAIN(PE05_BUZZER) | \
					 PIN_OTYPE_PUSHPULL(PE06_SPI4_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(PE07_DRDY8) | \
					 PIN_OTYPE_PUSHPULL(PE08_UART7_TX) | \
					 PIN_OTYPE_PUSHPULL(PE09_SERVO4) | \
					 PIN_OTYPE_OPENDRAIN(PE10_SAFETY_SWITCH_IN) | \
					 PIN_OTYPE_PUSHPULL(PE11_SERVO3) | \
					 PIN_OTYPE_PUSHPULL(PE12_LED4) | \
					 PIN_OTYPE_PUSHPULL(PE13_SPI4_MISO) | \
					 PIN_OTYPE_PUSHPULL(PE14_SERVO1) | \
					 PIN_OTYPE_OPENDRAIN(PE15_V5V_PERIPH_OC))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PE00_UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PE01_UART8_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PE02_SPI4_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03_V3V3_SENSORS_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04_V3V3_SPEKTRUM_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05_BUZZER) | \
					 PIN_OSPEED_SPEED_HIGH(PE06_SPI4_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07_DRDY8) | \
					 PIN_OSPEED_SPEED_HIGH(PE08_UART7_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PE09_SERVO4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10_SAFETY_SWITCH_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PE11_SERVO3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12_LED4) | \
					 PIN_OSPEED_SPEED_HIGH(PE13_SPI4_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PE14_SERVO1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15_V5V_PERIPH_OC))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(PE00_UART8_RX) | \
					 PIN_PUPDR_FLOATING(PE01_UART8_TX) | \
					 PIN_PUPDR_FLOATING(PE02_SPI4_SCK) | \
					 PIN_PUPDR_FLOATING(PE03_V3V3_SENSORS_EN) | \
					 PIN_PUPDR_FLOATING(PE04_V3V3_SPEKTRUM_EN) | \
					 PIN_PUPDR_PULLDOWN(PE05_BUZZER) | \
					 PIN_PUPDR_FLOATING(PE06_SPI4_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PE07_DRDY8) | \
					 PIN_PUPDR_FLOATING(PE08_UART7_TX) | \
					 PIN_PUPDR_FLOATING(PE09_SERVO4) | \
					 PIN_PUPDR_PULLDOWN(PE10_SAFETY_SWITCH_IN) | \
					 PIN_PUPDR_FLOATING(PE11_SERVO3) | \
					 PIN_PUPDR_FLOATING(PE12_LED4) | \
					 PIN_PUPDR_FLOATING(PE13_SPI4_MISO) | \
					 PIN_PUPDR_FLOATING(PE14_SERVO1) | \
					 PIN_PUPDR_PULLDOWN(PE15_V5V_PERIPH_OC))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(PE00_UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(PE01_UART8_TX) | \
					 PIN_ODR_LEVEL_HIGH(PE02_SPI4_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PE03_V3V3_SENSORS_EN) | \
					 PIN_ODR_LEVEL_HIGH(PE04_V3V3_SPEKTRUM_EN) | \
					 PIN_ODR_LEVEL_HIGH(PE05_BUZZER) | \
					 PIN_ODR_LEVEL_HIGH(PE06_SPI4_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PE07_DRDY8) | \
					 PIN_ODR_LEVEL_HIGH(PE08_UART7_TX) | \
					 PIN_ODR_LEVEL_LOW(PE09_SERVO4) | \
					 PIN_ODR_LEVEL_LOW(PE10_SAFETY_SWITCH_IN) | \
					 PIN_ODR_LEVEL_LOW(PE11_SERVO3) | \
					 PIN_ODR_LEVEL_LOW(PE12_LED4) | \
					 PIN_ODR_LEVEL_HIGH(PE13_SPI4_MISO) | \
					 PIN_ODR_LEVEL_LOW(PE14_SERVO1) | \
					 PIN_ODR_LEVEL_HIGH(PE15_V5V_PERIPH_OC))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00_UART8_RX, 8) | \
					 PIN_AFIO_AF(PE01_UART8_TX, 8) | \
					 PIN_AFIO_AF(PE02_SPI4_SCK, 5) | \
					 PIN_AFIO_AF(PE03_V3V3_SENSORS_EN, 0) | \
					 PIN_AFIO_AF(PE04_V3V3_SPEKTRUM_EN, 0) | \
					 PIN_AFIO_AF(PE05_BUZZER, 0) | \
					 PIN_AFIO_AF(PE06_SPI4_MOSI, 5) | \
					 PIN_AFIO_AF(PE07_DRDY8, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08_UART7_TX, 8) | \
					 PIN_AFIO_AF(PE09_SERVO4, 1) | \
					 PIN_AFIO_AF(PE10_SAFETY_SWITCH_IN, 0) | \
					 PIN_AFIO_AF(PE11_SERVO3, 1) | \
					 PIN_AFIO_AF(PE12_LED4, 0) | \
					 PIN_AFIO_AF(PE13_SPI4_MISO, 5) | \
					 PIN_AFIO_AF(PE14_SERVO1, 1) | \
					 PIN_AFIO_AF(PE15_V5V_PERIPH_OC, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_ALTERNATE(PF00_I2C2_SDA) | \
					 PIN_MODE_ALTERNATE(PF01_I2C2_SCL) | \
					 PIN_MODE_OUTPUT(PF02_SPI_SLAVE0) | \
					 PIN_MODE_OUTPUT(PF03_SPI_SLAVE1) | \
					 PIN_MODE_OUTPUT(PF04_SPI_SLAVE2) | \
					 PIN_MODE_OUTPUT(PF05_SPI_SLAVE3) | \
					 PIN_MODE_ALTERNATE(PF06_UART7_RX) | \
					 PIN_MODE_ALTERNATE(PF07_SPI5_SCK) | \
					 PIN_MODE_ALTERNATE(PF08_SPI5_MISO) | \
					 PIN_MODE_ALTERNATE(PF09_SPI5_MOSI) | \
					 PIN_MODE_OUTPUT(PF10_SPI_SLAVE4) | \
					 PIN_MODE_OUTPUT(PF11_SPI_SLAVE5) | \
					 PIN_MODE_OUTPUT(PF12_V5V_HIPOWER_EN) | \
					 PIN_MODE_INPUT(PF13_V5V_HIPOWER_OC) | \
					 PIN_MODE_ALTERNATE(PF14_I2C4_SCL) | \
					 PIN_MODE_ALTERNATE(PF15_I2C4_SDA))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_OPENDRAIN(PF00_I2C2_SDA) | \
					 PIN_OTYPE_OPENDRAIN(PF01_I2C2_SCL) | \
					 PIN_OTYPE_PUSHPULL(PF02_SPI_SLAVE0) | \
					 PIN_OTYPE_PUSHPULL(PF03_SPI_SLAVE1) | \
					 PIN_OTYPE_PUSHPULL(PF04_SPI_SLAVE2) | \
					 PIN_OTYPE_PUSHPULL(PF05_SPI_SLAVE3) | \
					 PIN_OTYPE_PUSHPULL(PF06_UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(PF07_SPI5_SCK) | \
					 PIN_OTYPE_PUSHPULL(PF08_SPI5_MISO) | \
					 PIN_OTYPE_PUSHPULL(PF09_SPI5_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PF10_SPI_SLAVE4) | \
					 PIN_OTYPE_PUSHPULL(PF11_SPI_SLAVE5) | \
					 PIN_OTYPE_PUSHPULL(PF12_V5V_HIPOWER_EN) | \
					 PIN_OTYPE_OPENDRAIN(PF13_V5V_HIPOWER_OC) | \
					 PIN_OTYPE_OPENDRAIN(PF14_I2C4_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PF15_I2C4_SDA))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PF00_I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PF01_I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PF02_SPI_SLAVE0) | \
					 PIN_OSPEED_SPEED_HIGH(PF03_SPI_SLAVE1) | \
					 PIN_OSPEED_SPEED_HIGH(PF04_SPI_SLAVE2) | \
					 PIN_OSPEED_SPEED_HIGH(PF05_SPI_SLAVE3) | \
					 PIN_OSPEED_SPEED_HIGH(PF06_UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PF07_SPI5_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PF08_SPI5_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PF09_SPI5_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PF10_SPI_SLAVE4) | \
					 PIN_OSPEED_SPEED_HIGH(PF11_SPI_SLAVE5) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12_V5V_HIPOWER_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13_V5V_HIPOWER_OC) | \
					 PIN_OSPEED_SPEED_HIGH(PF14_I2C4_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PF15_I2C4_SDA))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLUP(PF00_I2C2_SDA) | \
					 PIN_PUPDR_PULLUP(PF01_I2C2_SCL) | \
					 PIN_PUPDR_FLOATING(PF02_SPI_SLAVE0) | \
					 PIN_PUPDR_FLOATING(PF03_SPI_SLAVE1) | \
					 PIN_PUPDR_FLOATING(PF04_SPI_SLAVE2) | \
					 PIN_PUPDR_FLOATING(PF05_SPI_SLAVE3) | \
					 PIN_PUPDR_FLOATING(PF06_UART7_RX) | \
					 PIN_PUPDR_FLOATING(PF07_SPI5_SCK) | \
					 PIN_PUPDR_FLOATING(PF08_SPI5_MISO) | \
					 PIN_PUPDR_FLOATING(PF09_SPI5_MOSI) | \
					 PIN_PUPDR_FLOATING(PF10_SPI_SLAVE4) | \
					 PIN_PUPDR_FLOATING(PF11_SPI_SLAVE5) | \
					 PIN_PUPDR_FLOATING(PF12_V5V_HIPOWER_EN) | \
					 PIN_PUPDR_PULLDOWN(PF13_V5V_HIPOWER_OC) | \
					 PIN_PUPDR_PULLUP(PF14_I2C4_SCL) | \
					 PIN_PUPDR_PULLUP(PF15_I2C4_SDA))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_HIGH(PF00_I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PF01_I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PF02_SPI_SLAVE0) | \
					 PIN_ODR_LEVEL_HIGH(PF03_SPI_SLAVE1) | \
					 PIN_ODR_LEVEL_HIGH(PF04_SPI_SLAVE2) | \
					 PIN_ODR_LEVEL_HIGH(PF05_SPI_SLAVE3) | \
					 PIN_ODR_LEVEL_HIGH(PF06_UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(PF07_SPI5_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PF08_SPI5_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PF09_SPI5_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PF10_SPI_SLAVE4) | \
					 PIN_ODR_LEVEL_HIGH(PF11_SPI_SLAVE5) | \
					 PIN_ODR_LEVEL_LOW(PF12_V5V_HIPOWER_EN) | \
					 PIN_ODR_LEVEL_HIGH(PF13_V5V_HIPOWER_OC) | \
					 PIN_ODR_LEVEL_HIGH(PF14_I2C4_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PF15_I2C4_SDA))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00_I2C2_SDA, 4) | \
					 PIN_AFIO_AF(PF01_I2C2_SCL, 4) | \
					 PIN_AFIO_AF(PF02_SPI_SLAVE0, 0) | \
					 PIN_AFIO_AF(PF03_SPI_SLAVE1, 0) | \
					 PIN_AFIO_AF(PF04_SPI_SLAVE2, 0) | \
					 PIN_AFIO_AF(PF05_SPI_SLAVE3, 0) | \
					 PIN_AFIO_AF(PF06_UART7_RX, 8) | \
					 PIN_AFIO_AF(PF07_SPI5_SCK, 5))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08_SPI5_MISO, 5) | \
					 PIN_AFIO_AF(PF09_SPI5_MOSI, 5) | \
					 PIN_AFIO_AF(PF10_SPI_SLAVE4, 0) | \
					 PIN_AFIO_AF(PF11_SPI_SLAVE5, 0) | \
					 PIN_AFIO_AF(PF12_V5V_HIPOWER_EN, 0) | \
					 PIN_AFIO_AF(PF13_V5V_HIPOWER_OC, 0) | \
					 PIN_AFIO_AF(PF14_I2C4_SCL, 4) | \
					 PIN_AFIO_AF(PF15_I2C4_SDA, 4))

#define VAL_GPIOG_MODER                 (PIN_MODE_OUTPUT(PG00_HW_VER_DRIVE) | \
					 PIN_MODE_INPUT(PG01_POWER_IN_A) | \
					 PIN_MODE_INPUT(PG02_POWER_IN_B) | \
					 PIN_MODE_INPUT(PG03_POWER_IN_C) | \
					 PIN_MODE_OUTPUT(PG04_V5V_PERIPH_EN) | \
					 PIN_MODE_OUTPUT(PG05_V5V_RC_EN) | \
					 PIN_MODE_OUTPUT(PG06_V5V_WIFI_EN) | \
					 PIN_MODE_OUTPUT(PG07_V3V3_SD_CARD_EN) | \
					 PIN_MODE_INPUT(PG08_USART6_RTS) | \
					 PIN_MODE_ALTERNATE(PG09_USART6_RX) | \
					 PIN_MODE_OUTPUT(PG10_SPI_SLAVE6) | \
					 PIN_MODE_ALTERNATE(PG11_SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(PG12_SPI6_MISO) | \
					 PIN_MODE_ALTERNATE(PG13_SPI6_SCK) | \
					 PIN_MODE_ALTERNATE(PG14_USART6_TX) | \
					 PIN_MODE_INPUT(PG15_USART6_CTS))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(PG00_HW_VER_DRIVE) | \
					 PIN_OTYPE_OPENDRAIN(PG01_POWER_IN_A) | \
					 PIN_OTYPE_OPENDRAIN(PG02_POWER_IN_B) | \
					 PIN_OTYPE_OPENDRAIN(PG03_POWER_IN_C) | \
					 PIN_OTYPE_PUSHPULL(PG04_V5V_PERIPH_EN) | \
					 PIN_OTYPE_PUSHPULL(PG05_V5V_RC_EN) | \
					 PIN_OTYPE_PUSHPULL(PG06_V5V_WIFI_EN) | \
					 PIN_OTYPE_PUSHPULL(PG07_V3V3_SD_CARD_EN) | \
					 PIN_OTYPE_OPENDRAIN(PG08_USART6_RTS) | \
					 PIN_OTYPE_PUSHPULL(PG09_USART6_RX) | \
					 PIN_OTYPE_PUSHPULL(PG10_SPI_SLAVE6) | \
					 PIN_OTYPE_PUSHPULL(PG11_SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(PG12_SPI6_MISO) | \
					 PIN_OTYPE_PUSHPULL(PG13_SPI6_SCK) | \
					 PIN_OTYPE_PUSHPULL(PG14_USART6_TX) | \
					 PIN_OTYPE_OPENDRAIN(PG15_USART6_CTS))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00_HW_VER_DRIVE) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01_POWER_IN_A) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02_POWER_IN_B) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03_POWER_IN_C) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04_V5V_PERIPH_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05_V5V_RC_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06_V5V_WIFI_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07_V3V3_SD_CARD_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08_USART6_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(PG09_USART6_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PG10_SPI_SLAVE6) | \
					 PIN_OSPEED_SPEED_HIGH(PG11_SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PG12_SPI6_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PG13_SPI6_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PG14_USART6_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15_USART6_CTS))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_FLOATING(PG00_HW_VER_DRIVE) | \
					 PIN_PUPDR_PULLDOWN(PG01_POWER_IN_A) | \
					 PIN_PUPDR_PULLDOWN(PG02_POWER_IN_B) | \
					 PIN_PUPDR_PULLDOWN(PG03_POWER_IN_C) | \
					 PIN_PUPDR_FLOATING(PG04_V5V_PERIPH_EN) | \
					 PIN_PUPDR_FLOATING(PG05_V5V_RC_EN) | \
					 PIN_PUPDR_FLOATING(PG06_V5V_WIFI_EN) | \
					 PIN_PUPDR_FLOATING(PG07_V3V3_SD_CARD_EN) | \
					 PIN_PUPDR_PULLDOWN(PG08_USART6_RTS) | \
					 PIN_PUPDR_FLOATING(PG09_USART6_RX) | \
					 PIN_PUPDR_FLOATING(PG10_SPI_SLAVE6) | \
					 PIN_PUPDR_FLOATING(PG11_SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(PG12_SPI6_MISO) | \
					 PIN_PUPDR_FLOATING(PG13_SPI6_SCK) | \
					 PIN_PUPDR_FLOATING(PG14_USART6_TX) | \
					 PIN_PUPDR_PULLDOWN(PG15_USART6_CTS))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(PG00_HW_VER_DRIVE) | \
					 PIN_ODR_LEVEL_HIGH(PG01_POWER_IN_A) | \
					 PIN_ODR_LEVEL_HIGH(PG02_POWER_IN_B) | \
					 PIN_ODR_LEVEL_HIGH(PG03_POWER_IN_C) | \
					 PIN_ODR_LEVEL_LOW(PG04_V5V_PERIPH_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG05_V5V_RC_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG06_V5V_WIFI_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG07_V3V3_SD_CARD_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG08_USART6_RTS) | \
					 PIN_ODR_LEVEL_HIGH(PG09_USART6_RX) | \
					 PIN_ODR_LEVEL_HIGH(PG10_SPI_SLAVE6) | \
					 PIN_ODR_LEVEL_HIGH(PG11_SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PG12_SPI6_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PG13_SPI6_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PG14_USART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(PG15_USART6_CTS))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00_HW_VER_DRIVE, 0) | \
					 PIN_AFIO_AF(PG01_POWER_IN_A, 0) | \
					 PIN_AFIO_AF(PG02_POWER_IN_B, 0) | \
					 PIN_AFIO_AF(PG03_POWER_IN_C, 0) | \
					 PIN_AFIO_AF(PG04_V5V_PERIPH_EN, 0) | \
					 PIN_AFIO_AF(PG05_V5V_RC_EN, 0) | \
					 PIN_AFIO_AF(PG06_V5V_WIFI_EN, 0) | \
					 PIN_AFIO_AF(PG07_V3V3_SD_CARD_EN, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08_USART6_RTS, 0) | \
					 PIN_AFIO_AF(PG09_USART6_RX, 8) | \
					 PIN_AFIO_AF(PG10_SPI_SLAVE6, 0) | \
					 PIN_AFIO_AF(PG11_SPI1_SCK, 5) | \
					 PIN_AFIO_AF(PG12_SPI6_MISO, 5) | \
					 PIN_AFIO_AF(PG13_SPI6_SCK, 5) | \
					 PIN_AFIO_AF(PG14_USART6_TX, 8) | \
					 PIN_AFIO_AF(PG15_USART6_CTS, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(PH00_OSC_IN) | \
					 PIN_MODE_ALTERNATE(PH01_OSC_OUT) | \
					 PIN_MODE_OUTPUT(PH02_CAN1_SILENT_S0) | \
					 PIN_MODE_OUTPUT(PH03_CAN2_SILENT_S1) | \
					 PIN_MODE_OUTPUT(PH04_CAN3_SILENT_S2) | \
					 PIN_MODE_OUTPUT(PH05_SPI_SLAVE7) | \
					 PIN_MODE_ALTERNATE(PH06_SERVO7) | \
					 PIN_MODE_ALTERNATE(PH07_I2C3_SCL) | \
					 PIN_MODE_ALTERNATE(PH08_I2C3_SDA) | \
					 PIN_MODE_ALTERNATE(PH09_SERVO8) | \
					 PIN_MODE_OUTPUT(PH10_LED5) | \
					 PIN_MODE_OUTPUT(PH11_LED6) | \
					 PIN_MODE_OUTPUT(PH12_LED7) | \
					 PIN_MODE_ALTERNATE(PH13_CAN1_TX) | \
					 PIN_MODE_OUTPUT(PH14_HW_REV_DRIVE) | \
					 PIN_MODE_INPUT(PH15_SPI5_SYNC))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(PH00_OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(PH01_OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(PH02_CAN1_SILENT_S0) | \
					 PIN_OTYPE_PUSHPULL(PH03_CAN2_SILENT_S1) | \
					 PIN_OTYPE_PUSHPULL(PH04_CAN3_SILENT_S2) | \
					 PIN_OTYPE_PUSHPULL(PH05_SPI_SLAVE7) | \
					 PIN_OTYPE_PUSHPULL(PH06_SERVO7) | \
					 PIN_OTYPE_OPENDRAIN(PH07_I2C3_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PH08_I2C3_SDA) | \
					 PIN_OTYPE_PUSHPULL(PH09_SERVO8) | \
					 PIN_OTYPE_PUSHPULL(PH10_LED5) | \
					 PIN_OTYPE_PUSHPULL(PH11_LED6) | \
					 PIN_OTYPE_PUSHPULL(PH12_LED7) | \
					 PIN_OTYPE_PUSHPULL(PH13_CAN1_TX) | \
					 PIN_OTYPE_PUSHPULL(PH14_HW_REV_DRIVE) | \
					 PIN_OTYPE_OPENDRAIN(PH15_SPI5_SYNC))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PH00_OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PH01_OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02_CAN1_SILENT_S0) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03_CAN2_SILENT_S1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04_CAN3_SILENT_S2) | \
					 PIN_OSPEED_SPEED_HIGH(PH05_SPI_SLAVE7) | \
					 PIN_OSPEED_SPEED_HIGH(PH06_SERVO7) | \
					 PIN_OSPEED_SPEED_HIGH(PH07_I2C3_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PH08_I2C3_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PH09_SERVO8) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH10_LED5) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH11_LED6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH12_LED7) | \
					 PIN_OSPEED_SPEED_HIGH(PH13_CAN1_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH14_HW_REV_DRIVE) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15_SPI5_SYNC))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(PH00_OSC_IN) | \
					 PIN_PUPDR_FLOATING(PH01_OSC_OUT) | \
					 PIN_PUPDR_FLOATING(PH02_CAN1_SILENT_S0) | \
					 PIN_PUPDR_FLOATING(PH03_CAN2_SILENT_S1) | \
					 PIN_PUPDR_FLOATING(PH04_CAN3_SILENT_S2) | \
					 PIN_PUPDR_FLOATING(PH05_SPI_SLAVE7) | \
					 PIN_PUPDR_FLOATING(PH06_SERVO7) | \
					 PIN_PUPDR_PULLUP(PH07_I2C3_SCL) | \
					 PIN_PUPDR_PULLUP(PH08_I2C3_SDA) | \
					 PIN_PUPDR_FLOATING(PH09_SERVO8) | \
					 PIN_PUPDR_FLOATING(PH10_LED5) | \
					 PIN_PUPDR_FLOATING(PH11_LED6) | \
					 PIN_PUPDR_FLOATING(PH12_LED7) | \
					 PIN_PUPDR_FLOATING(PH13_CAN1_TX) | \
					 PIN_PUPDR_FLOATING(PH14_HW_REV_DRIVE) | \
					 PIN_PUPDR_PULLDOWN(PH15_SPI5_SYNC))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(PH00_OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(PH01_OSC_OUT) | \
					 PIN_ODR_LEVEL_LOW(PH02_CAN1_SILENT_S0) | \
					 PIN_ODR_LEVEL_LOW(PH03_CAN2_SILENT_S1) | \
					 PIN_ODR_LEVEL_LOW(PH04_CAN3_SILENT_S2) | \
					 PIN_ODR_LEVEL_HIGH(PH05_SPI_SLAVE7) | \
					 PIN_ODR_LEVEL_LOW(PH06_SERVO7) | \
					 PIN_ODR_LEVEL_HIGH(PH07_I2C3_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PH08_I2C3_SDA) | \
					 PIN_ODR_LEVEL_LOW(PH09_SERVO8) | \
					 PIN_ODR_LEVEL_LOW(PH10_LED5) | \
					 PIN_ODR_LEVEL_LOW(PH11_LED6) | \
					 PIN_ODR_LEVEL_LOW(PH12_LED7) | \
					 PIN_ODR_LEVEL_HIGH(PH13_CAN1_TX) | \
					 PIN_ODR_LEVEL_LOW(PH14_HW_REV_DRIVE) | \
					 PIN_ODR_LEVEL_HIGH(PH15_SPI5_SYNC))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(PH00_OSC_IN, 0) | \
					 PIN_AFIO_AF(PH01_OSC_OUT, 0) | \
					 PIN_AFIO_AF(PH02_CAN1_SILENT_S0, 0) | \
					 PIN_AFIO_AF(PH03_CAN2_SILENT_S1, 0) | \
					 PIN_AFIO_AF(PH04_CAN3_SILENT_S2, 0) | \
					 PIN_AFIO_AF(PH05_SPI_SLAVE7, 0) | \
					 PIN_AFIO_AF(PH06_SERVO7, 9) | \
					 PIN_AFIO_AF(PH07_I2C3_SCL, 4))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08_I2C3_SDA, 4) | \
					 PIN_AFIO_AF(PH09_SERVO8, 9) | \
					 PIN_AFIO_AF(PH10_LED5, 0) | \
					 PIN_AFIO_AF(PH11_LED6, 0) | \
					 PIN_AFIO_AF(PH12_LED7, 0) | \
					 PIN_AFIO_AF(PH13_CAN1_TX, 9) | \
					 PIN_AFIO_AF(PH14_HW_REV_DRIVE, 0) | \
					 PIN_AFIO_AF(PH15_SPI5_SYNC, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_OUTPUT(PI00_ARMED) | \
					 PIN_MODE_ALTERNATE(PI01_SPI2_SCK) | \
					 PIN_MODE_ALTERNATE(PI02_SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(PI03_SPI2_MOSI) | \
					 PIN_MODE_OUTPUT(PI04_SPI_SLAVE8) | \
					 PIN_MODE_ALTERNATE(PI05_RC_INPUT) | \
					 PIN_MODE_OUTPUT(PI06_SPI_SLAVE9) | \
					 PIN_MODE_OUTPUT(PI07_SPI_SLAVE10) | \
					 PIN_MODE_OUTPUT(PI08_SPI_SLAVE11) | \
					 PIN_MODE_ALTERNATE(PI09_CAN1_RX) | \
					 PIN_MODE_OUTPUT(PI10_SPI_SLAVE12) | \
					 PIN_MODE_OUTPUT(PI11_SPI_SLAVE13) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00_ARMED) | \
					 PIN_OTYPE_PUSHPULL(PI01_SPI2_SCK) | \
					 PIN_OTYPE_PUSHPULL(PI02_SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(PI03_SPI2_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PI04_SPI_SLAVE8) | \
					 PIN_OTYPE_PUSHPULL(PI05_RC_INPUT) | \
					 PIN_OTYPE_PUSHPULL(PI06_SPI_SLAVE9) | \
					 PIN_OTYPE_PUSHPULL(PI07_SPI_SLAVE10) | \
					 PIN_OTYPE_PUSHPULL(PI08_SPI_SLAVE11) | \
					 PIN_OTYPE_PUSHPULL(PI09_CAN1_RX) | \
					 PIN_OTYPE_PUSHPULL(PI10_SPI_SLAVE12) | \
					 PIN_OTYPE_PUSHPULL(PI11_SPI_SLAVE13) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PI00_ARMED) | \
					 PIN_OSPEED_SPEED_HIGH(PI01_SPI2_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PI02_SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PI03_SPI2_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PI04_SPI_SLAVE8) | \
					 PIN_OSPEED_SPEED_HIGH(PI05_RC_INPUT) | \
					 PIN_OSPEED_SPEED_HIGH(PI06_SPI_SLAVE9) | \
					 PIN_OSPEED_SPEED_HIGH(PI07_SPI_SLAVE10) | \
					 PIN_OSPEED_SPEED_HIGH(PI08_SPI_SLAVE11) | \
					 PIN_OSPEED_SPEED_HIGH(PI09_CAN1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PI10_SPI_SLAVE12) | \
					 PIN_OSPEED_SPEED_HIGH(PI11_SPI_SLAVE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_FLOATING(PI00_ARMED) | \
					 PIN_PUPDR_FLOATING(PI01_SPI2_SCK) | \
					 PIN_PUPDR_FLOATING(PI02_SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(PI03_SPI2_MOSI) | \
					 PIN_PUPDR_FLOATING(PI04_SPI_SLAVE8) | \
					 PIN_PUPDR_FLOATING(PI05_RC_INPUT) | \
					 PIN_PUPDR_FLOATING(PI06_SPI_SLAVE9) | \
					 PIN_PUPDR_FLOATING(PI07_SPI_SLAVE10) | \
					 PIN_PUPDR_FLOATING(PI08_SPI_SLAVE11) | \
					 PIN_PUPDR_FLOATING(PI09_CAN1_RX) | \
					 PIN_PUPDR_FLOATING(PI10_SPI_SLAVE12) | \
					 PIN_PUPDR_FLOATING(PI11_SPI_SLAVE13) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00_ARMED) | \
					 PIN_ODR_LEVEL_HIGH(PI01_SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PI02_SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PI03_SPI2_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PI04_SPI_SLAVE8) | \
					 PIN_ODR_LEVEL_HIGH(PI05_RC_INPUT) | \
					 PIN_ODR_LEVEL_HIGH(PI06_SPI_SLAVE9) | \
					 PIN_ODR_LEVEL_HIGH(PI07_SPI_SLAVE10) | \
					 PIN_ODR_LEVEL_HIGH(PI08_SPI_SLAVE11) | \
					 PIN_ODR_LEVEL_HIGH(PI09_CAN1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PI10_SPI_SLAVE12) | \
					 PIN_ODR_LEVEL_HIGH(PI11_SPI_SLAVE13) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(PI00_ARMED, 0) | \
					 PIN_AFIO_AF(PI01_SPI2_SCK, 5) | \
					 PIN_AFIO_AF(PI02_SPI2_MISO, 5) | \
					 PIN_AFIO_AF(PI03_SPI2_MOSI, 5) | \
					 PIN_AFIO_AF(PI04_SPI_SLAVE8, 0) | \
					 PIN_AFIO_AF(PI05_RC_INPUT, 3) | \
					 PIN_AFIO_AF(PI06_SPI_SLAVE9, 0) | \
					 PIN_AFIO_AF(PI07_SPI_SLAVE10, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(PI08_SPI_SLAVE11, 0) | \
					 PIN_AFIO_AF(PI09_CAN1_RX, 9) | \
					 PIN_AFIO_AF(PI10_SPI_SLAVE12, 0) | \
					 PIN_AFIO_AF(PI11_SPI_SLAVE13, 0) | \
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

#define AF_PA06_SPI1_MISO                5U
#define AF_LINE_SPI1_MISO                5U
#define AF_PA08_CAN3_RX                  11U
#define AF_LINE_CAN3_RX                  11U
#define AF_PA10_SERVO2                   1U
#define AF_LINE_SERVO2                   1U
#define AF_PA11_USB_DM                   10U
#define AF_LINE_USB_DM                   10U
#define AF_PA12_USB_DP                   10U
#define AF_LINE_USB_DP                   10U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PA15_CAN3_TX                  11U
#define AF_LINE_CAN3_TX                  11U
#define AF_PB05_SPI6_MOSI                8U
#define AF_LINE_SPI6_MOSI                8U
#define AF_PB06_USART1_TX                7U
#define AF_LINE_USART1_TX                7U
#define AF_PB07_USART1_RX                7U
#define AF_LINE_USART1_RX                7U
#define AF_PB08_I2C1_SCL                 4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_PB09_I2C1_SDA                 4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_PB12_CAN2_RX                  9U
#define AF_LINE_CAN2_RX                  9U
#define AF_PB13_CAN2_TX                  9U
#define AF_LINE_CAN2_TX                  9U
#define AF_PC08_SDIO_D0                  12U
#define AF_LINE_SDIO_D0                  12U
#define AF_PC09_SDIO_D1                  12U
#define AF_LINE_SDIO_D1                  12U
#define AF_PC10_SDIO_D2                  12U
#define AF_LINE_SDIO_D2                  12U
#define AF_PC11_SDIO_D3                  12U
#define AF_LINE_SDIO_D3                  12U
#define AF_PC12_SDIO_CK                  12U
#define AF_LINE_SDIO_CK                  12U
#define AF_PC14_OSC32_IN                 0U
#define AF_LINE_OSC32_IN                 0U
#define AF_PC15_OSC32_OUT                0U
#define AF_LINE_OSC32_OUT                0U
#define AF_PD00_UART4_RX                 8U
#define AF_LINE_UART4_RX                 8U
#define AF_PD01_UART4_TX                 8U
#define AF_LINE_UART4_TX                 8U
#define AF_PD02_SDIO_CMD                 12U
#define AF_LINE_SDIO_CMD                 12U
#define AF_PD05_UART2_TX                 7U
#define AF_LINE_UART2_TX                 7U
#define AF_PD06_UART2_RX                 7U
#define AF_LINE_UART2_RX                 7U
#define AF_PD07_SPI1_MOSI                5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_PD08_UART3_TX                 7U
#define AF_LINE_UART3_TX                 7U
#define AF_PD09_UART3_RX                 7U
#define AF_LINE_UART3_RX                 7U
#define AF_PD13_SERVO5                   2U
#define AF_LINE_SERVO5                   2U
#define AF_PD14_SERVO6                   2U
#define AF_LINE_SERVO6                   2U
#define AF_PE00_UART8_RX                 8U
#define AF_LINE_UART8_RX                 8U
#define AF_PE01_UART8_TX                 8U
#define AF_LINE_UART8_TX                 8U
#define AF_PE02_SPI4_SCK                 5U
#define AF_LINE_SPI4_SCK                 5U
#define AF_PE06_SPI4_MOSI                5U
#define AF_LINE_SPI4_MOSI                5U
#define AF_PE08_UART7_TX                 8U
#define AF_LINE_UART7_TX                 8U
#define AF_PE09_SERVO4                   1U
#define AF_LINE_SERVO4                   1U
#define AF_PE11_SERVO3                   1U
#define AF_LINE_SERVO3                   1U
#define AF_PE13_SPI4_MISO                5U
#define AF_LINE_SPI4_MISO                5U
#define AF_PE14_SERVO1                   1U
#define AF_LINE_SERVO1                   1U
#define AF_PF00_I2C2_SDA                 4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_PF01_I2C2_SCL                 4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_PF06_UART7_RX                 8U
#define AF_LINE_UART7_RX                 8U
#define AF_PF07_SPI5_SCK                 5U
#define AF_LINE_SPI5_SCK                 5U
#define AF_PF08_SPI5_MISO                5U
#define AF_LINE_SPI5_MISO                5U
#define AF_PF09_SPI5_MOSI                5U
#define AF_LINE_SPI5_MOSI                5U
#define AF_PF14_I2C4_SCL                 4U
#define AF_LINE_I2C4_SCL                 4U
#define AF_PF15_I2C4_SDA                 4U
#define AF_LINE_I2C4_SDA                 4U
#define AF_PG09_USART6_RX                8U
#define AF_LINE_USART6_RX                8U
#define AF_PG11_SPI1_SCK                 5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_PG12_SPI6_MISO                5U
#define AF_LINE_SPI6_MISO                5U
#define AF_PG13_SPI6_SCK                 5U
#define AF_LINE_SPI6_SCK                 5U
#define AF_PG14_USART6_TX                8U
#define AF_LINE_USART6_TX                8U
#define AF_PH00_OSC_IN                   0U
#define AF_LINE_OSC_IN                   0U
#define AF_PH01_OSC_OUT                  0U
#define AF_LINE_OSC_OUT                  0U
#define AF_PH06_SERVO7                   9U
#define AF_LINE_SERVO7                   9U
#define AF_PH07_I2C3_SCL                 4U
#define AF_LINE_I2C3_SCL                 4U
#define AF_PH08_I2C3_SDA                 4U
#define AF_LINE_I2C3_SDA                 4U
#define AF_PH09_SERVO8                   9U
#define AF_LINE_SERVO8                   9U
#define AF_PH13_CAN1_TX                  9U
#define AF_LINE_CAN1_TX                  9U
#define AF_PI01_SPI2_SCK                 5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_PI02_SPI2_MISO                5U
#define AF_LINE_SPI2_MISO                5U
#define AF_PI03_SPI2_MOSI                5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_PI05_RC_INPUT                 3U
#define AF_LINE_RC_INPUT                 3U
#define AF_PI09_CAN1_RX                  9U
#define AF_LINE_CAN1_RX                  9U


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

