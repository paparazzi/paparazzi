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
#define BOARD_PX4FMU_6X
#define BOARD_NAME                  "PX4FMU 6X"

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
#define STM32H753xx
#define MCUCONF_H7

/*
 * PWM TIM defines
 * enable TIM4, TIM5 and TIM12 by default
 */
#ifndef USE_PWM_TIM4
#define USE_PWM_TIM4 1
#endif

#ifndef USE_PWM_TIM5
#define USE_PWM_TIM5 1
#endif

#ifndef USE_PWM_TIM12
//#define USE_PWM_TIM12 1
#endif

/*
 * IO pins assignments.
 */
#define	PA00_ADC1                      0U
#define	PA01_ETH_RMII_REF_CLK          1U
#define	PA02_ETH_MDIO                  2U
#define	PA03_UART2                     3U
#define	PA04_ADC2                      4U
#define	PA05_SPI1_SCK                  5U
#define	PA06_SPI6_MISO                 6U
#define	PA07_ETH_RMII_CRS_DV           7U
#define	PA08_I2C3_SCL                  8U
#define	PA09_USB_VBUS                  9U
#define	PA10_SPI2_DRDY                 10U
#define	PA11_USB_DM                    11U
#define	PA12_USB_DP                    12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15                           15U

#define	PB00_ADC3                      0U
#define	PB01_ADC5                      1U
#define	PB02_SPI3_MOSI                 2U
#define	PB03_SPI6_SCK                  3U
#define	PB04_SDIO_D3                   4U
#define	PB05_SPI1_MOSI                 5U
#define	PB06_UART1_TX                  6U
#define	PB07_UART1_RX                  7U
#define	PB08_I2C1_SCL                  8U
#define	PB09_I2C1_SDA                  9U
#define	PB10_IMU_HEATER                10U
#define	PB11_ETH_RMII_TX_EN            11U
#define	PB12_CAN2_RX                   12U
#define	PB13_CAN2_TX                   13U
#define	PB14_SDIO_D0                   14U
#define	PB15_SDIO_D1                   15U

#define	PC00_NFC_GPIO                  0U
#define	PC01_ETH_MDC                   1U
#define	PC02_ADC6                      2U
#define	PC03_ADC7                      3U
#define	PC04_ETH_RMII_RXD0             4U
#define	PC05_ETH_RMII_RXD1             5U
#define	PC06_UART6_TX                  6U
#define	PC07_UART6_RX                  7U
#define	PC08_UART5_RTS                 8U
#define	PC09_UART5_CTS                 9U
#define	PC10_SPI3_SCK                  10U
#define	PC11_SPI3_MISO                 11U
#define	PC12_UART5_TX                  12U
#define	PC13_VDD_3V3_SD_CARD_EN        13U
#define	PC14                           14U
#define	PC15                           15U

#define	PD00_CAN1_RX                   0U
#define	PD01_CAN1_TX                   1U
#define	PD02_UART5_RX                  2U
#define	PD03_UART2_CTS                 3U
#define	PD04_UART2_RTS                 4U
#define	PD05_UART2_TX                  5U
#define	PD06_SDIO_CK                   6U
#define	PD07_SDIO_CMD                  7U
#define	PD08_UART3_TX                  8U
#define	PD09_UART3_RX                  9U
#define	PD10_LED4                      10U
#define	PD11                           11U
#define	PD12                           12U
#define	PD13_SERVO5                    13U
#define	PD14_SERVO6                    14U
#define	PD15                           15U

#define	PE00_UART8_RX                  0U
#define	PE01_UART8_TX                  1U
#define	PE02                           2U
#define	PE03_LED1                      3U
#define	PE04_LED2                      4U
#define	PE05_LED3                      5U
#define	PE06_NARMED                    6U
#define	PE07_VDD_3V3_SENSORS3_EN       7U
#define	PE08_UART7_TX                  8U
#define	PE09                           9U
#define	PE10_UART7_CTS                 10U
#define	PE11_FMU_CAP1                  11U
#define	PE12_SPI4_SCK                  12U
#define	PE13_SPI4_MISO                 13U
#define	PE14_SPI4_MOSI                 14U
#define	PE15_VDD_5V_PERIPH_OC          15U

#define	PF00_I2C2_SDA                  0U
#define	PF01_I2C2_SCL                  1U
#define	PF02                           2U
#define	PF03_SPI4_DRDY1                3U
#define	PF04_VDD_3V3_SENSORS2_EN       4U
#define	PF05_SAFETY_IN                 5U
#define	PF06_UART7_RX                  6U
#define	PF07_SPI5_SCK                  7U
#define	PF08_UART7_RTS                 8U
#define	PF09_ALARM                     9U
#define	PF10                           10U
#define	PF11_SPI5_MOSI                 11U
#define	PF12_ADC4                      12U
#define	PF13_VDD_5V_HIPOWER_OC         13U
#define	PF14_I2C4_SCL                  14U
#define	PF15_I2C4_SDA                  15U

#define	PG00_HW_VER_REV_DRIVE          0U
#define	PG01_VDD_BRICK_VALID           1U
#define	PG02_VDD_BRICK2_VALID          2U
#define	PG03_VDD_BRICK3_VALID          3U
#define	PG04_VDD_5V_PERIPH_EN          4U
#define	PG05_DRDY1_BMP388              5U
#define	PG06                           6U
#define	PG07_SPI_SLAVE6                7U
#define	PG08_VDD_3V3_SENSORS4_EN       8U
#define	PG09_SPI1_MISO                 9U
#define	PG10_VDD_5V_HIPOWER_EN         10U
#define	PG11_SDIO_D2                   11U
#define	PG12_ETH_RMII_TXD1             12U
#define	PG13_ETH_RMII_TXD0             13U
#define	PG14_SPI6_MOSI                 14U
#define	PG15_ETH_POWER_EN              15U

#define	PH00_OSC_IN                    0U
#define	PH01_OSC_OUT                   1U
#define	PH02_SPEKTRUM_PWR_EN           2U
#define	PH03_HW_VER_SENS               3U
#define	PH04_HW_REV_SENS               4U
#define	PH05_SPI_SLAVE2                5U
#define	PH06                           6U
#define	PH07_SPI5_MISO                 7U
#define	PH08_I2C3_SDA                  8U
#define	PH09                           9U
#define	PH10_SERVO4                    10U
#define	PH11_SERVO3                    11U
#define	PH12_SERVO2                    12U
#define	PH13_UART4_TX                  13U
#define	PH14_UART4_RX                  14U
#define	PH15_SPI_SLAVE5                15U

#define	PI00_SERVO1                    0U
#define	PI01_SPI2_SCK                  1U
#define	PI02_SPI2_MISO                 2U
#define	PI03_SPI2_MOSI                 3U
#define	PI04_SPI_SLAVE3                4U
#define	PI05_PWM_INPUT1                5U
#define	PI06_SPI3_DRDY1                6U
#define	PI07_SPI3_DRDY2                7U
#define	PI08_SPI_SLAVE4                8U
#define	PI09_SPI_SLAVE1                9U
#define	PI10_SPI_SLAVE7                10U
#define	PI11_VDD_3V3_SENSORS1_EN       11U
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
#define	LINE_ETH_RMII_REF_CLK          PAL_LINE(GPIOA, 1U)
#define	LINE_ETH_MDIO                  PAL_LINE(GPIOA, 2U)
#define	LINE_UART2                     PAL_LINE(GPIOA, 3U)
#define	LINE_ADC2                      PAL_LINE(GPIOA, 4U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI6_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_ETH_RMII_CRS_DV           PAL_LINE(GPIOA, 7U)
#define	LINE_I2C3_SCL                  PAL_LINE(GPIOA, 8U)
#define	LINE_USB_VBUS                  PAL_LINE(GPIOA, 9U)
#define	LINE_SPI2_DRDY                 PAL_LINE(GPIOA, 10U)
#define	LINE_USB_DM                    PAL_LINE(GPIOA, 11U)
#define	LINE_USB_DP                    PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)

#define	LINE_ADC3                      PAL_LINE(GPIOB, 0U)
#define	LINE_ADC5                      PAL_LINE(GPIOB, 1U)
#define	LINE_SPI3_MOSI                 PAL_LINE(GPIOB, 2U)
#define	LINE_SPI6_SCK                  PAL_LINE(GPIOB, 3U)
#define	LINE_SDIO_D3                   PAL_LINE(GPIOB, 4U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_UART1_TX                  PAL_LINE(GPIOB, 6U)
#define	LINE_UART1_RX                  PAL_LINE(GPIOB, 7U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 9U)
#define	LINE_IMU_HEATER                PAL_LINE(GPIOB, 10U)
#define	LINE_ETH_RMII_TX_EN            PAL_LINE(GPIOB, 11U)
#define	LINE_CAN2_RX                   PAL_LINE(GPIOB, 12U)
#define	LINE_CAN2_TX                   PAL_LINE(GPIOB, 13U)
#define	LINE_SDIO_D0                   PAL_LINE(GPIOB, 14U)
#define	LINE_SDIO_D1                   PAL_LINE(GPIOB, 15U)

#define	LINE_NFC_GPIO                  PAL_LINE(GPIOC, 0U)
#define	LINE_ETH_MDC                   PAL_LINE(GPIOC, 1U)
#define	LINE_ADC6                      PAL_LINE(GPIOC, 2U)
#define	LINE_ADC7                      PAL_LINE(GPIOC, 3U)
#define	LINE_ETH_RMII_RXD0             PAL_LINE(GPIOC, 4U)
#define	LINE_ETH_RMII_RXD1             PAL_LINE(GPIOC, 5U)
#define	LINE_UART6_TX                  PAL_LINE(GPIOC, 6U)
#define	LINE_UART6_RX                  PAL_LINE(GPIOC, 7U)
#define	LINE_UART5_RTS                 PAL_LINE(GPIOC, 8U)
#define	LINE_UART5_CTS                 PAL_LINE(GPIOC, 9U)
#define	LINE_SPI3_SCK                  PAL_LINE(GPIOC, 10U)
#define	LINE_SPI3_MISO                 PAL_LINE(GPIOC, 11U)
#define	LINE_UART5_TX                  PAL_LINE(GPIOC, 12U)
#define	LINE_VDD_3V3_SD_CARD_EN        PAL_LINE(GPIOC, 13U)

#define	LINE_CAN1_RX                   PAL_LINE(GPIOD, 0U)
#define	LINE_CAN1_TX                   PAL_LINE(GPIOD, 1U)
#define	LINE_UART5_RX                  PAL_LINE(GPIOD, 2U)
#define	LINE_UART2_CTS                 PAL_LINE(GPIOD, 3U)
#define	LINE_UART2_RTS                 PAL_LINE(GPIOD, 4U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOD, 5U)
#define	LINE_SDIO_CK                   PAL_LINE(GPIOD, 6U)
#define	LINE_SDIO_CMD                  PAL_LINE(GPIOD, 7U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOD, 8U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOD, 9U)
#define	LINE_LED4                      PAL_LINE(GPIOD, 10U)
#define	LINE_SERVO5                    PAL_LINE(GPIOD, 13U)
#define	LINE_SERVO6                    PAL_LINE(GPIOD, 14U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE_LED1                      PAL_LINE(GPIOE, 3U)
#define	LINE_LED2                      PAL_LINE(GPIOE, 4U)
#define	LINE_LED3                      PAL_LINE(GPIOE, 5U)
#define	LINE_NARMED                    PAL_LINE(GPIOE, 6U)
#define	LINE_VDD_3V3_SENSORS3_EN       PAL_LINE(GPIOE, 7U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOE, 8U)
#define	LINE_UART7_CTS                 PAL_LINE(GPIOE, 10U)
#define	LINE_FMU_CAP1                  PAL_LINE(GPIOE, 11U)
#define	LINE_SPI4_SCK                  PAL_LINE(GPIOE, 12U)
#define	LINE_SPI4_MISO                 PAL_LINE(GPIOE, 13U)
#define	LINE_SPI4_MOSI                 PAL_LINE(GPIOE, 14U)
#define	LINE_VDD_5V_PERIPH_OC          PAL_LINE(GPIOE, 15U)

#define	LINE_I2C2_SDA                  PAL_LINE(GPIOF, 0U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOF, 1U)
#define	LINE_SPI4_DRDY1                PAL_LINE(GPIOF, 3U)
#define	LINE_VDD_3V3_SENSORS2_EN       PAL_LINE(GPIOF, 4U)
#define	LINE_SAFETY_IN                 PAL_LINE(GPIOF, 5U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOF, 6U)
#define	LINE_SPI5_SCK                  PAL_LINE(GPIOF, 7U)
#define	LINE_UART7_RTS                 PAL_LINE(GPIOF, 8U)
#define	LINE_ALARM                     PAL_LINE(GPIOF, 9U)
#define	LINE_SPI5_MOSI                 PAL_LINE(GPIOF, 11U)
#define	LINE_ADC4                      PAL_LINE(GPIOF, 12U)
#define	LINE_VDD_5V_HIPOWER_OC         PAL_LINE(GPIOF, 13U)
#define	LINE_I2C4_SCL                  PAL_LINE(GPIOF, 14U)
#define	LINE_I2C4_SDA                  PAL_LINE(GPIOF, 15U)

#define	LINE_HW_VER_REV_DRIVE          PAL_LINE(GPIOG, 0U)
#define	LINE_VDD_BRICK_VALID           PAL_LINE(GPIOG, 1U)
#define	LINE_VDD_BRICK2_VALID          PAL_LINE(GPIOG, 2U)
#define	LINE_VDD_BRICK3_VALID          PAL_LINE(GPIOG, 3U)
#define	LINE_VDD_5V_PERIPH_EN          PAL_LINE(GPIOG, 4U)
#define	LINE_DRDY1_BMP388              PAL_LINE(GPIOG, 5U)
#define	LINE_SPI_SLAVE6                PAL_LINE(GPIOG, 7U)
#define	LINE_VDD_3V3_SENSORS4_EN       PAL_LINE(GPIOG, 8U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOG, 9U)
#define	LINE_VDD_5V_HIPOWER_EN         PAL_LINE(GPIOG, 10U)
#define	LINE_SDIO_D2                   PAL_LINE(GPIOG, 11U)
#define	LINE_ETH_RMII_TXD1             PAL_LINE(GPIOG, 12U)
#define	LINE_ETH_RMII_TXD0             PAL_LINE(GPIOG, 13U)
#define	LINE_SPI6_MOSI                 PAL_LINE(GPIOG, 14U)
#define	LINE_ETH_POWER_EN              PAL_LINE(GPIOG, 15U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)
#define	LINE_SPEKTRUM_PWR_EN           PAL_LINE(GPIOH, 2U)
#define	LINE_HW_VER_SENS               PAL_LINE(GPIOH, 3U)
#define	LINE_HW_REV_SENS               PAL_LINE(GPIOH, 4U)
#define	LINE_SPI_SLAVE2                PAL_LINE(GPIOH, 5U)
#define	LINE_SPI5_MISO                 PAL_LINE(GPIOH, 7U)
#define	LINE_I2C3_SDA                  PAL_LINE(GPIOH, 8U)
#define	LINE_SERVO4                    PAL_LINE(GPIOH, 10U)
#define	LINE_SERVO3                    PAL_LINE(GPIOH, 11U)
#define	LINE_SERVO2                    PAL_LINE(GPIOH, 12U)
#define	LINE_UART4_TX                  PAL_LINE(GPIOH, 13U)
#define	LINE_UART4_RX                  PAL_LINE(GPIOH, 14U)
#define	LINE_SPI_SLAVE5                PAL_LINE(GPIOH, 15U)

#define	LINE_SERVO1                    PAL_LINE(GPIOI, 0U)
#define	LINE_SPI2_SCK                  PAL_LINE(GPIOI, 1U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOI, 2U)
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOI, 3U)
#define	LINE_SPI_SLAVE3                PAL_LINE(GPIOI, 4U)
#define	LINE_PWM_INPUT1                PAL_LINE(GPIOI, 5U)
#define	LINE_SPI3_DRDY1                PAL_LINE(GPIOI, 6U)
#define	LINE_SPI3_DRDY2                PAL_LINE(GPIOI, 7U)
#define	LINE_SPI_SLAVE4                PAL_LINE(GPIOI, 8U)
#define	LINE_SPI_SLAVE1                PAL_LINE(GPIOI, 9U)
#define	LINE_SPI_SLAVE7                PAL_LINE(GPIOI, 10U)
#define	LINE_VDD_3V3_SENSORS1_EN       PAL_LINE(GPIOI, 11U)


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
					 PIN_MODE_INPUT(PA01_ETH_RMII_REF_CLK) | \
					 PIN_MODE_INPUT(PA02_ETH_MDIO) | \
					 PIN_MODE_ALTERNATE(PA03_UART2) | \
					 PIN_MODE_ANALOG(PA04_ADC2) | \
					 PIN_MODE_ALTERNATE(PA05_SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(PA06_SPI6_MISO) | \
					 PIN_MODE_INPUT(PA07_ETH_RMII_CRS_DV) | \
					 PIN_MODE_ALTERNATE(PA08_I2C3_SCL) | \
					 PIN_MODE_INPUT(PA09_USB_VBUS) | \
					 PIN_MODE_INPUT(PA10_SPI2_DRDY) | \
					 PIN_MODE_ALTERNATE(PA11_USB_DM) | \
					 PIN_MODE_ALTERNATE(PA12_USB_DP) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_INPUT(PA15))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_ADC1) | \
					 PIN_OTYPE_OPENDRAIN(PA01_ETH_RMII_REF_CLK) | \
					 PIN_OTYPE_OPENDRAIN(PA02_ETH_MDIO) | \
					 PIN_OTYPE_PUSHPULL(PA03_UART2) | \
					 PIN_OTYPE_PUSHPULL(PA04_ADC2) | \
					 PIN_OTYPE_PUSHPULL(PA05_SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(PA06_SPI6_MISO) | \
					 PIN_OTYPE_OPENDRAIN(PA07_ETH_RMII_CRS_DV) | \
					 PIN_OTYPE_OPENDRAIN(PA08_I2C3_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PA09_USB_VBUS) | \
					 PIN_OTYPE_OPENDRAIN(PA10_SPI2_DRDY) | \
					 PIN_OTYPE_PUSHPULL(PA11_USB_DM) | \
					 PIN_OTYPE_PUSHPULL(PA12_USB_DP) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PA00_ADC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA01_ETH_RMII_REF_CLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA02_ETH_MDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA03_UART2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04_ADC2) | \
					 PIN_OSPEED_SPEED_HIGH(PA05_SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PA06_SPI6_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA07_ETH_RMII_CRS_DV) | \
					 PIN_OSPEED_SPEED_HIGH(PA08_I2C3_SCL) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA09_USB_VBUS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA10_SPI2_DRDY) | \
					 PIN_OSPEED_SPEED_HIGH(PA11_USB_DM) | \
					 PIN_OSPEED_SPEED_HIGH(PA12_USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA15))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_ADC1) | \
					 PIN_PUPDR_PULLDOWN(PA01_ETH_RMII_REF_CLK) | \
					 PIN_PUPDR_PULLDOWN(PA02_ETH_MDIO) | \
					 PIN_PUPDR_FLOATING(PA03_UART2) | \
					 PIN_PUPDR_FLOATING(PA04_ADC2) | \
					 PIN_PUPDR_FLOATING(PA05_SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(PA06_SPI6_MISO) | \
					 PIN_PUPDR_PULLDOWN(PA07_ETH_RMII_CRS_DV) | \
					 PIN_PUPDR_PULLUP(PA08_I2C3_SCL) | \
					 PIN_PUPDR_PULLDOWN(PA09_USB_VBUS) | \
					 PIN_PUPDR_PULLDOWN(PA10_SPI2_DRDY) | \
					 PIN_PUPDR_FLOATING(PA11_USB_DM) | \
					 PIN_PUPDR_FLOATING(PA12_USB_DP) | \
					 PIN_PUPDR_FLOATING(PA13_SWDIO) | \
					 PIN_PUPDR_FLOATING(PA14_SWCLK) | \
					 PIN_PUPDR_PULLDOWN(PA15))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(PA00_ADC1) | \
					 PIN_ODR_LEVEL_HIGH(PA01_ETH_RMII_REF_CLK) | \
					 PIN_ODR_LEVEL_HIGH(PA02_ETH_MDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA03_UART2) | \
					 PIN_ODR_LEVEL_LOW(PA04_ADC2) | \
					 PIN_ODR_LEVEL_HIGH(PA05_SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PA06_SPI6_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PA07_ETH_RMII_CRS_DV) | \
					 PIN_ODR_LEVEL_HIGH(PA08_I2C3_SCL) | \
					 PIN_ODR_LEVEL_LOW(PA09_USB_VBUS) | \
					 PIN_ODR_LEVEL_HIGH(PA10_SPI2_DRDY) | \
					 PIN_ODR_LEVEL_HIGH(PA11_USB_DM) | \
					 PIN_ODR_LEVEL_HIGH(PA12_USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PA15))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_ADC1, 0) | \
					 PIN_AFIO_AF(PA01_ETH_RMII_REF_CLK, 0) | \
					 PIN_AFIO_AF(PA02_ETH_MDIO, 0) | \
					 PIN_AFIO_AF(PA03_UART2, 7) | \
					 PIN_AFIO_AF(PA04_ADC2, 0) | \
					 PIN_AFIO_AF(PA05_SPI1_SCK, 5) | \
					 PIN_AFIO_AF(PA06_SPI6_MISO, 8) | \
					 PIN_AFIO_AF(PA07_ETH_RMII_CRS_DV, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_I2C3_SCL, 4) | \
					 PIN_AFIO_AF(PA09_USB_VBUS, 0) | \
					 PIN_AFIO_AF(PA10_SPI2_DRDY, 0) | \
					 PIN_AFIO_AF(PA11_USB_DM, 10) | \
					 PIN_AFIO_AF(PA12_USB_DP, 10) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_ANALOG(PB00_ADC3) | \
					 PIN_MODE_ANALOG(PB01_ADC5) | \
					 PIN_MODE_ALTERNATE(PB02_SPI3_MOSI) | \
					 PIN_MODE_ALTERNATE(PB03_SPI6_SCK) | \
					 PIN_MODE_ALTERNATE(PB04_SDIO_D3) | \
					 PIN_MODE_ALTERNATE(PB05_SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(PB06_UART1_TX) | \
					 PIN_MODE_ALTERNATE(PB07_UART1_RX) | \
					 PIN_MODE_ALTERNATE(PB08_I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(PB09_I2C1_SDA) | \
					 PIN_MODE_OUTPUT(PB10_IMU_HEATER) | \
					 PIN_MODE_INPUT(PB11_ETH_RMII_TX_EN) | \
					 PIN_MODE_ALTERNATE(PB12_CAN2_RX) | \
					 PIN_MODE_ALTERNATE(PB13_CAN2_TX) | \
					 PIN_MODE_ALTERNATE(PB14_SDIO_D0) | \
					 PIN_MODE_ALTERNATE(PB15_SDIO_D1))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00_ADC3) | \
					 PIN_OTYPE_PUSHPULL(PB01_ADC5) | \
					 PIN_OTYPE_PUSHPULL(PB02_SPI3_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PB03_SPI6_SCK) | \
					 PIN_OTYPE_PUSHPULL(PB04_SDIO_D3) | \
					 PIN_OTYPE_PUSHPULL(PB05_SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PB06_UART1_TX) | \
					 PIN_OTYPE_PUSHPULL(PB07_UART1_RX) | \
					 PIN_OTYPE_OPENDRAIN(PB08_I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PB09_I2C1_SDA) | \
					 PIN_OTYPE_PUSHPULL(PB10_IMU_HEATER) | \
					 PIN_OTYPE_OPENDRAIN(PB11_ETH_RMII_TX_EN) | \
					 PIN_OTYPE_PUSHPULL(PB12_CAN2_RX) | \
					 PIN_OTYPE_PUSHPULL(PB13_CAN2_TX) | \
					 PIN_OTYPE_PUSHPULL(PB14_SDIO_D0) | \
					 PIN_OTYPE_PUSHPULL(PB15_SDIO_D1))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PB00_ADC3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB01_ADC5) | \
					 PIN_OSPEED_SPEED_HIGH(PB02_SPI3_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PB03_SPI6_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PB04_SDIO_D3) | \
					 PIN_OSPEED_SPEED_HIGH(PB05_SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_UART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PB07_UART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB08_I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PB09_I2C1_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PB10_IMU_HEATER) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB11_ETH_RMII_TX_EN) | \
					 PIN_OSPEED_SPEED_HIGH(PB12_CAN2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB13_CAN2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PB14_SDIO_D0) | \
					 PIN_OSPEED_SPEED_HIGH(PB15_SDIO_D1))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PB00_ADC3) | \
					 PIN_PUPDR_FLOATING(PB01_ADC5) | \
					 PIN_PUPDR_FLOATING(PB02_SPI3_MOSI) | \
					 PIN_PUPDR_FLOATING(PB03_SPI6_SCK) | \
					 PIN_PUPDR_PULLUP(PB04_SDIO_D3) | \
					 PIN_PUPDR_FLOATING(PB05_SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(PB06_UART1_TX) | \
					 PIN_PUPDR_FLOATING(PB07_UART1_RX) | \
					 PIN_PUPDR_PULLUP(PB08_I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(PB09_I2C1_SDA) | \
					 PIN_PUPDR_FLOATING(PB10_IMU_HEATER) | \
					 PIN_PUPDR_PULLDOWN(PB11_ETH_RMII_TX_EN) | \
					 PIN_PUPDR_FLOATING(PB12_CAN2_RX) | \
					 PIN_PUPDR_FLOATING(PB13_CAN2_TX) | \
					 PIN_PUPDR_PULLUP(PB14_SDIO_D0) | \
					 PIN_PUPDR_PULLUP(PB15_SDIO_D1))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00_ADC3) | \
					 PIN_ODR_LEVEL_LOW(PB01_ADC5) | \
					 PIN_ODR_LEVEL_HIGH(PB02_SPI3_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PB03_SPI6_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PB04_SDIO_D3) | \
					 PIN_ODR_LEVEL_HIGH(PB05_SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PB06_UART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB07_UART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB08_I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PB09_I2C1_SDA) | \
					 PIN_ODR_LEVEL_LOW(PB10_IMU_HEATER) | \
					 PIN_ODR_LEVEL_HIGH(PB11_ETH_RMII_TX_EN) | \
					 PIN_ODR_LEVEL_HIGH(PB12_CAN2_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB13_CAN2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB14_SDIO_D0) | \
					 PIN_ODR_LEVEL_HIGH(PB15_SDIO_D1))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_ADC3, 0) | \
					 PIN_AFIO_AF(PB01_ADC5, 0) | \
					 PIN_AFIO_AF(PB02_SPI3_MOSI, 7) | \
					 PIN_AFIO_AF(PB03_SPI6_SCK, 8) | \
					 PIN_AFIO_AF(PB04_SDIO_D3, 9) | \
					 PIN_AFIO_AF(PB05_SPI1_MOSI, 5) | \
					 PIN_AFIO_AF(PB06_UART1_TX, 7) | \
					 PIN_AFIO_AF(PB07_UART1_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08_I2C1_SCL, 4) | \
					 PIN_AFIO_AF(PB09_I2C1_SDA, 4) | \
					 PIN_AFIO_AF(PB10_IMU_HEATER, 0) | \
					 PIN_AFIO_AF(PB11_ETH_RMII_TX_EN, 0) | \
					 PIN_AFIO_AF(PB12_CAN2_RX, 9) | \
					 PIN_AFIO_AF(PB13_CAN2_TX, 9) | \
					 PIN_AFIO_AF(PB14_SDIO_D0, 9) | \
					 PIN_AFIO_AF(PB15_SDIO_D1, 9))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(PC00_NFC_GPIO) | \
					 PIN_MODE_INPUT(PC01_ETH_MDC) | \
					 PIN_MODE_ANALOG(PC02_ADC6) | \
					 PIN_MODE_ANALOG(PC03_ADC7) | \
					 PIN_MODE_INPUT(PC04_ETH_RMII_RXD0) | \
					 PIN_MODE_INPUT(PC05_ETH_RMII_RXD1) | \
					 PIN_MODE_ALTERNATE(PC06_UART6_TX) | \
					 PIN_MODE_ALTERNATE(PC07_UART6_RX) | \
					 PIN_MODE_INPUT(PC08_UART5_RTS) | \
					 PIN_MODE_INPUT(PC09_UART5_CTS) | \
					 PIN_MODE_ALTERNATE(PC10_SPI3_SCK) | \
					 PIN_MODE_ALTERNATE(PC11_SPI3_MISO) | \
					 PIN_MODE_ALTERNATE(PC12_UART5_TX) | \
					 PIN_MODE_OUTPUT(PC13_VDD_3V3_SD_CARD_EN) | \
					 PIN_MODE_INPUT(PC14) | \
					 PIN_MODE_INPUT(PC15))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_OPENDRAIN(PC00_NFC_GPIO) | \
					 PIN_OTYPE_OPENDRAIN(PC01_ETH_MDC) | \
					 PIN_OTYPE_PUSHPULL(PC02_ADC6) | \
					 PIN_OTYPE_PUSHPULL(PC03_ADC7) | \
					 PIN_OTYPE_OPENDRAIN(PC04_ETH_RMII_RXD0) | \
					 PIN_OTYPE_OPENDRAIN(PC05_ETH_RMII_RXD1) | \
					 PIN_OTYPE_PUSHPULL(PC06_UART6_TX) | \
					 PIN_OTYPE_PUSHPULL(PC07_UART6_RX) | \
					 PIN_OTYPE_OPENDRAIN(PC08_UART5_RTS) | \
					 PIN_OTYPE_OPENDRAIN(PC09_UART5_CTS) | \
					 PIN_OTYPE_PUSHPULL(PC10_SPI3_SCK) | \
					 PIN_OTYPE_PUSHPULL(PC11_SPI3_MISO) | \
					 PIN_OTYPE_PUSHPULL(PC12_UART5_TX) | \
					 PIN_OTYPE_PUSHPULL(PC13_VDD_3V3_SD_CARD_EN) | \
					 PIN_OTYPE_PUSHPULL(PC14) | \
					 PIN_OTYPE_PUSHPULL(PC15))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00_NFC_GPIO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01_ETH_MDC) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02_ADC6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03_ADC7) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04_ETH_RMII_RXD0) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05_ETH_RMII_RXD1) | \
					 PIN_OSPEED_SPEED_HIGH(PC06_UART6_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PC07_UART6_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC08_UART5_RTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC09_UART5_CTS) | \
					 PIN_OSPEED_SPEED_HIGH(PC10_SPI3_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PC11_SPI3_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PC12_UART5_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13_VDD_3V3_SD_CARD_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC15))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(PC00_NFC_GPIO) | \
					 PIN_PUPDR_PULLDOWN(PC01_ETH_MDC) | \
					 PIN_PUPDR_FLOATING(PC02_ADC6) | \
					 PIN_PUPDR_FLOATING(PC03_ADC7) | \
					 PIN_PUPDR_PULLDOWN(PC04_ETH_RMII_RXD0) | \
					 PIN_PUPDR_PULLDOWN(PC05_ETH_RMII_RXD1) | \
					 PIN_PUPDR_FLOATING(PC06_UART6_TX) | \
					 PIN_PUPDR_FLOATING(PC07_UART6_RX) | \
					 PIN_PUPDR_PULLDOWN(PC08_UART5_RTS) | \
					 PIN_PUPDR_PULLDOWN(PC09_UART5_CTS) | \
					 PIN_PUPDR_FLOATING(PC10_SPI3_SCK) | \
					 PIN_PUPDR_FLOATING(PC11_SPI3_MISO) | \
					 PIN_PUPDR_FLOATING(PC12_UART5_TX) | \
					 PIN_PUPDR_FLOATING(PC13_VDD_3V3_SD_CARD_EN) | \
					 PIN_PUPDR_PULLDOWN(PC14) | \
					 PIN_PUPDR_PULLDOWN(PC15))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_HIGH(PC00_NFC_GPIO) | \
					 PIN_ODR_LEVEL_HIGH(PC01_ETH_MDC) | \
					 PIN_ODR_LEVEL_LOW(PC02_ADC6) | \
					 PIN_ODR_LEVEL_LOW(PC03_ADC7) | \
					 PIN_ODR_LEVEL_HIGH(PC04_ETH_RMII_RXD0) | \
					 PIN_ODR_LEVEL_HIGH(PC05_ETH_RMII_RXD1) | \
					 PIN_ODR_LEVEL_HIGH(PC06_UART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(PC07_UART6_RX) | \
					 PIN_ODR_LEVEL_HIGH(PC08_UART5_RTS) | \
					 PIN_ODR_LEVEL_HIGH(PC09_UART5_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PC10_SPI3_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PC11_SPI3_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PC12_UART5_TX) | \
					 PIN_ODR_LEVEL_HIGH(PC13_VDD_3V3_SD_CARD_EN) | \
					 PIN_ODR_LEVEL_LOW(PC14) | \
					 PIN_ODR_LEVEL_LOW(PC15))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00_NFC_GPIO, 0) | \
					 PIN_AFIO_AF(PC01_ETH_MDC, 0) | \
					 PIN_AFIO_AF(PC02_ADC6, 0) | \
					 PIN_AFIO_AF(PC03_ADC7, 0) | \
					 PIN_AFIO_AF(PC04_ETH_RMII_RXD0, 0) | \
					 PIN_AFIO_AF(PC05_ETH_RMII_RXD1, 0) | \
					 PIN_AFIO_AF(PC06_UART6_TX, 7) | \
					 PIN_AFIO_AF(PC07_UART6_RX, 7))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08_UART5_RTS, 0) | \
					 PIN_AFIO_AF(PC09_UART5_CTS, 0) | \
					 PIN_AFIO_AF(PC10_SPI3_SCK, 6) | \
					 PIN_AFIO_AF(PC11_SPI3_MISO, 6) | \
					 PIN_AFIO_AF(PC12_UART5_TX, 8) | \
					 PIN_AFIO_AF(PC13_VDD_3V3_SD_CARD_EN, 0) | \
					 PIN_AFIO_AF(PC14, 0) | \
					 PIN_AFIO_AF(PC15, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(PD00_CAN1_RX) | \
					 PIN_MODE_ALTERNATE(PD01_CAN1_TX) | \
					 PIN_MODE_ALTERNATE(PD02_UART5_RX) | \
					 PIN_MODE_INPUT(PD03_UART2_CTS) | \
					 PIN_MODE_INPUT(PD04_UART2_RTS) | \
					 PIN_MODE_ALTERNATE(PD05_UART2_TX) | \
					 PIN_MODE_ALTERNATE(PD06_SDIO_CK) | \
					 PIN_MODE_ALTERNATE(PD07_SDIO_CMD) | \
					 PIN_MODE_ALTERNATE(PD08_UART3_TX) | \
					 PIN_MODE_ALTERNATE(PD09_UART3_RX) | \
					 PIN_MODE_OUTPUT(PD10_LED4) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_INPUT(PD12) | \
					 PIN_MODE_ALTERNATE(PD13_SERVO5) | \
					 PIN_MODE_ALTERNATE(PD14_SERVO6) | \
					 PIN_MODE_INPUT(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00_CAN1_RX) | \
					 PIN_OTYPE_PUSHPULL(PD01_CAN1_TX) | \
					 PIN_OTYPE_PUSHPULL(PD02_UART5_RX) | \
					 PIN_OTYPE_OPENDRAIN(PD03_UART2_CTS) | \
					 PIN_OTYPE_OPENDRAIN(PD04_UART2_RTS) | \
					 PIN_OTYPE_PUSHPULL(PD05_UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(PD06_SDIO_CK) | \
					 PIN_OTYPE_PUSHPULL(PD07_SDIO_CMD) | \
					 PIN_OTYPE_PUSHPULL(PD08_UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(PD09_UART3_RX) | \
					 PIN_OTYPE_PUSHPULL(PD10_LED4) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_PUSHPULL(PD12) | \
					 PIN_OTYPE_PUSHPULL(PD13_SERVO5) | \
					 PIN_OTYPE_PUSHPULL(PD14_SERVO6) | \
					 PIN_OTYPE_PUSHPULL(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PD00_CAN1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PD01_CAN1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD02_UART5_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03_UART2_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04_UART2_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(PD05_UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD06_SDIO_CK) | \
					 PIN_OSPEED_SPEED_HIGH(PD07_SDIO_CMD) | \
					 PIN_OSPEED_SPEED_HIGH(PD08_UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD09_UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10_LED4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12) | \
					 PIN_OSPEED_SPEED_HIGH(PD13_SERVO5) | \
					 PIN_OSPEED_SPEED_HIGH(PD14_SERVO6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(PD00_CAN1_RX) | \
					 PIN_PUPDR_FLOATING(PD01_CAN1_TX) | \
					 PIN_PUPDR_FLOATING(PD02_UART5_RX) | \
					 PIN_PUPDR_PULLDOWN(PD03_UART2_CTS) | \
					 PIN_PUPDR_PULLDOWN(PD04_UART2_RTS) | \
					 PIN_PUPDR_FLOATING(PD05_UART2_TX) | \
					 PIN_PUPDR_PULLUP(PD06_SDIO_CK) | \
					 PIN_PUPDR_PULLUP(PD07_SDIO_CMD) | \
					 PIN_PUPDR_FLOATING(PD08_UART3_TX) | \
					 PIN_PUPDR_FLOATING(PD09_UART3_RX) | \
					 PIN_PUPDR_FLOATING(PD10_LED4) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_PULLDOWN(PD12) | \
					 PIN_PUPDR_FLOATING(PD13_SERVO5) | \
					 PIN_PUPDR_FLOATING(PD14_SERVO6) | \
					 PIN_PUPDR_PULLDOWN(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(PD00_CAN1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD01_CAN1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD02_UART5_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD03_UART2_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PD04_UART2_RTS) | \
					 PIN_ODR_LEVEL_HIGH(PD05_UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD06_SDIO_CK) | \
					 PIN_ODR_LEVEL_HIGH(PD07_SDIO_CMD) | \
					 PIN_ODR_LEVEL_HIGH(PD08_UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD09_UART3_RX) | \
					 PIN_ODR_LEVEL_LOW(PD10_LED4) | \
					 PIN_ODR_LEVEL_LOW(PD11) | \
					 PIN_ODR_LEVEL_LOW(PD12) | \
					 PIN_ODR_LEVEL_LOW(PD13_SERVO5) | \
					 PIN_ODR_LEVEL_LOW(PD14_SERVO6) | \
					 PIN_ODR_LEVEL_LOW(PD15))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00_CAN1_RX, 9) | \
					 PIN_AFIO_AF(PD01_CAN1_TX, 9) | \
					 PIN_AFIO_AF(PD02_UART5_RX, 8) | \
					 PIN_AFIO_AF(PD03_UART2_CTS, 0) | \
					 PIN_AFIO_AF(PD04_UART2_RTS, 0) | \
					 PIN_AFIO_AF(PD05_UART2_TX, 7) | \
					 PIN_AFIO_AF(PD06_SDIO_CK, 11) | \
					 PIN_AFIO_AF(PD07_SDIO_CMD, 11))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08_UART3_TX, 7) | \
					 PIN_AFIO_AF(PD09_UART3_RX, 7) | \
					 PIN_AFIO_AF(PD10_LED4, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12, 0) | \
					 PIN_AFIO_AF(PD13_SERVO5, 2) | \
					 PIN_AFIO_AF(PD14_SERVO6, 2) | \
					 PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(PE00_UART8_RX) | \
					 PIN_MODE_ALTERNATE(PE01_UART8_TX) | \
					 PIN_MODE_INPUT(PE02) | \
					 PIN_MODE_OUTPUT(PE03_LED1) | \
					 PIN_MODE_OUTPUT(PE04_LED2) | \
					 PIN_MODE_OUTPUT(PE05_LED3) | \
					 PIN_MODE_INPUT(PE06_NARMED) | \
					 PIN_MODE_OUTPUT(PE07_VDD_3V3_SENSORS3_EN) | \
					 PIN_MODE_ALTERNATE(PE08_UART7_TX) | \
					 PIN_MODE_INPUT(PE09) | \
					 PIN_MODE_INPUT(PE10_UART7_CTS) | \
					 PIN_MODE_INPUT(PE11_FMU_CAP1) | \
					 PIN_MODE_ALTERNATE(PE12_SPI4_SCK) | \
					 PIN_MODE_ALTERNATE(PE13_SPI4_MISO) | \
					 PIN_MODE_ALTERNATE(PE14_SPI4_MOSI) | \
					 PIN_MODE_INPUT(PE15_VDD_5V_PERIPH_OC))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00_UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(PE01_UART8_TX) | \
					 PIN_OTYPE_PUSHPULL(PE02) | \
					 PIN_OTYPE_PUSHPULL(PE03_LED1) | \
					 PIN_OTYPE_PUSHPULL(PE04_LED2) | \
					 PIN_OTYPE_PUSHPULL(PE05_LED3) | \
					 PIN_OTYPE_OPENDRAIN(PE06_NARMED) | \
					 PIN_OTYPE_PUSHPULL(PE07_VDD_3V3_SENSORS3_EN) | \
					 PIN_OTYPE_PUSHPULL(PE08_UART7_TX) | \
					 PIN_OTYPE_PUSHPULL(PE09) | \
					 PIN_OTYPE_OPENDRAIN(PE10_UART7_CTS) | \
					 PIN_OTYPE_OPENDRAIN(PE11_FMU_CAP1) | \
					 PIN_OTYPE_PUSHPULL(PE12_SPI4_SCK) | \
					 PIN_OTYPE_PUSHPULL(PE13_SPI4_MISO) | \
					 PIN_OTYPE_PUSHPULL(PE14_SPI4_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(PE15_VDD_5V_PERIPH_OC))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PE00_UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PE01_UART8_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03_LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04_LED2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05_LED3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE06_NARMED) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07_VDD_3V3_SENSORS3_EN) | \
					 PIN_OSPEED_SPEED_HIGH(PE08_UART7_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10_UART7_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE11_FMU_CAP1) | \
					 PIN_OSPEED_SPEED_HIGH(PE12_SPI4_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PE13_SPI4_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PE14_SPI4_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15_VDD_5V_PERIPH_OC))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(PE00_UART8_RX) | \
					 PIN_PUPDR_FLOATING(PE01_UART8_TX) | \
					 PIN_PUPDR_PULLDOWN(PE02) | \
					 PIN_PUPDR_FLOATING(PE03_LED1) | \
					 PIN_PUPDR_FLOATING(PE04_LED2) | \
					 PIN_PUPDR_FLOATING(PE05_LED3) | \
					 PIN_PUPDR_PULLDOWN(PE06_NARMED) | \
					 PIN_PUPDR_FLOATING(PE07_VDD_3V3_SENSORS3_EN) | \
					 PIN_PUPDR_FLOATING(PE08_UART7_TX) | \
					 PIN_PUPDR_PULLDOWN(PE09) | \
					 PIN_PUPDR_PULLDOWN(PE10_UART7_CTS) | \
					 PIN_PUPDR_PULLDOWN(PE11_FMU_CAP1) | \
					 PIN_PUPDR_FLOATING(PE12_SPI4_SCK) | \
					 PIN_PUPDR_FLOATING(PE13_SPI4_MISO) | \
					 PIN_PUPDR_FLOATING(PE14_SPI4_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PE15_VDD_5V_PERIPH_OC))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(PE00_UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(PE01_UART8_TX) | \
					 PIN_ODR_LEVEL_LOW(PE02) | \
					 PIN_ODR_LEVEL_LOW(PE03_LED1) | \
					 PIN_ODR_LEVEL_LOW(PE04_LED2) | \
					 PIN_ODR_LEVEL_LOW(PE05_LED3) | \
					 PIN_ODR_LEVEL_HIGH(PE06_NARMED) | \
					 PIN_ODR_LEVEL_HIGH(PE07_VDD_3V3_SENSORS3_EN) | \
					 PIN_ODR_LEVEL_HIGH(PE08_UART7_TX) | \
					 PIN_ODR_LEVEL_LOW(PE09) | \
					 PIN_ODR_LEVEL_HIGH(PE10_UART7_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PE11_FMU_CAP1) | \
					 PIN_ODR_LEVEL_HIGH(PE12_SPI4_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PE13_SPI4_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PE14_SPI4_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PE15_VDD_5V_PERIPH_OC))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00_UART8_RX, 8) | \
					 PIN_AFIO_AF(PE01_UART8_TX, 8) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03_LED1, 0) | \
					 PIN_AFIO_AF(PE04_LED2, 0) | \
					 PIN_AFIO_AF(PE05_LED3, 0) | \
					 PIN_AFIO_AF(PE06_NARMED, 0) | \
					 PIN_AFIO_AF(PE07_VDD_3V3_SENSORS3_EN, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08_UART7_TX, 7) | \
					 PIN_AFIO_AF(PE09, 0) | \
					 PIN_AFIO_AF(PE10_UART7_CTS, 0) | \
					 PIN_AFIO_AF(PE11_FMU_CAP1, 0) | \
					 PIN_AFIO_AF(PE12_SPI4_SCK, 5) | \
					 PIN_AFIO_AF(PE13_SPI4_MISO, 5) | \
					 PIN_AFIO_AF(PE14_SPI4_MOSI, 5) | \
					 PIN_AFIO_AF(PE15_VDD_5V_PERIPH_OC, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_ALTERNATE(PF00_I2C2_SDA) | \
					 PIN_MODE_ALTERNATE(PF01_I2C2_SCL) | \
					 PIN_MODE_INPUT(PF02) | \
					 PIN_MODE_INPUT(PF03_SPI4_DRDY1) | \
					 PIN_MODE_OUTPUT(PF04_VDD_3V3_SENSORS2_EN) | \
					 PIN_MODE_INPUT(PF05_SAFETY_IN) | \
					 PIN_MODE_ALTERNATE(PF06_UART7_RX) | \
					 PIN_MODE_ALTERNATE(PF07_SPI5_SCK) | \
					 PIN_MODE_INPUT(PF08_UART7_RTS) | \
					 PIN_MODE_ALTERNATE(PF09_ALARM) | \
					 PIN_MODE_INPUT(PF10) | \
					 PIN_MODE_ALTERNATE(PF11_SPI5_MOSI) | \
					 PIN_MODE_ANALOG(PF12_ADC4) | \
					 PIN_MODE_INPUT(PF13_VDD_5V_HIPOWER_OC) | \
					 PIN_MODE_ALTERNATE(PF14_I2C4_SCL) | \
					 PIN_MODE_ALTERNATE(PF15_I2C4_SDA))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_OPENDRAIN(PF00_I2C2_SDA) | \
					 PIN_OTYPE_OPENDRAIN(PF01_I2C2_SCL) | \
					 PIN_OTYPE_PUSHPULL(PF02) | \
					 PIN_OTYPE_OPENDRAIN(PF03_SPI4_DRDY1) | \
					 PIN_OTYPE_PUSHPULL(PF04_VDD_3V3_SENSORS2_EN) | \
					 PIN_OTYPE_OPENDRAIN(PF05_SAFETY_IN) | \
					 PIN_OTYPE_PUSHPULL(PF06_UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(PF07_SPI5_SCK) | \
					 PIN_OTYPE_OPENDRAIN(PF08_UART7_RTS) | \
					 PIN_OTYPE_PUSHPULL(PF09_ALARM) | \
					 PIN_OTYPE_PUSHPULL(PF10) | \
					 PIN_OTYPE_PUSHPULL(PF11_SPI5_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PF12_ADC4) | \
					 PIN_OTYPE_OPENDRAIN(PF13_VDD_5V_HIPOWER_OC) | \
					 PIN_OTYPE_OPENDRAIN(PF14_I2C4_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PF15_I2C4_SDA))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PF00_I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PF01_I2C2_SCL) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF03_SPI4_DRDY1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF04_VDD_3V3_SENSORS2_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF05_SAFETY_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PF06_UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PF07_SPI5_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF08_UART7_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(PF09_ALARM) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF10) | \
					 PIN_OSPEED_SPEED_HIGH(PF11_SPI5_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12_ADC4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13_VDD_5V_HIPOWER_OC) | \
					 PIN_OSPEED_SPEED_HIGH(PF14_I2C4_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PF15_I2C4_SDA))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLUP(PF00_I2C2_SDA) | \
					 PIN_PUPDR_PULLUP(PF01_I2C2_SCL) | \
					 PIN_PUPDR_PULLDOWN(PF02) | \
					 PIN_PUPDR_PULLDOWN(PF03_SPI4_DRDY1) | \
					 PIN_PUPDR_FLOATING(PF04_VDD_3V3_SENSORS2_EN) | \
					 PIN_PUPDR_PULLDOWN(PF05_SAFETY_IN) | \
					 PIN_PUPDR_FLOATING(PF06_UART7_RX) | \
					 PIN_PUPDR_FLOATING(PF07_SPI5_SCK) | \
					 PIN_PUPDR_PULLDOWN(PF08_UART7_RTS) | \
					 PIN_PUPDR_FLOATING(PF09_ALARM) | \
					 PIN_PUPDR_PULLDOWN(PF10) | \
					 PIN_PUPDR_FLOATING(PF11_SPI5_MOSI) | \
					 PIN_PUPDR_FLOATING(PF12_ADC4) | \
					 PIN_PUPDR_PULLDOWN(PF13_VDD_5V_HIPOWER_OC) | \
					 PIN_PUPDR_PULLUP(PF14_I2C4_SCL) | \
					 PIN_PUPDR_PULLUP(PF15_I2C4_SDA))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_HIGH(PF00_I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PF01_I2C2_SCL) | \
					 PIN_ODR_LEVEL_LOW(PF02) | \
					 PIN_ODR_LEVEL_HIGH(PF03_SPI4_DRDY1) | \
					 PIN_ODR_LEVEL_HIGH(PF04_VDD_3V3_SENSORS2_EN) | \
					 PIN_ODR_LEVEL_LOW(PF05_SAFETY_IN) | \
					 PIN_ODR_LEVEL_HIGH(PF06_UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(PF07_SPI5_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PF08_UART7_RTS) | \
					 PIN_ODR_LEVEL_LOW(PF09_ALARM) | \
					 PIN_ODR_LEVEL_LOW(PF10) | \
					 PIN_ODR_LEVEL_HIGH(PF11_SPI5_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PF12_ADC4) | \
					 PIN_ODR_LEVEL_HIGH(PF13_VDD_5V_HIPOWER_OC) | \
					 PIN_ODR_LEVEL_HIGH(PF14_I2C4_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PF15_I2C4_SDA))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00_I2C2_SDA, 4) | \
					 PIN_AFIO_AF(PF01_I2C2_SCL, 4) | \
					 PIN_AFIO_AF(PF02, 0) | \
					 PIN_AFIO_AF(PF03_SPI4_DRDY1, 0) | \
					 PIN_AFIO_AF(PF04_VDD_3V3_SENSORS2_EN, 0) | \
					 PIN_AFIO_AF(PF05_SAFETY_IN, 0) | \
					 PIN_AFIO_AF(PF06_UART7_RX, 7) | \
					 PIN_AFIO_AF(PF07_SPI5_SCK, 5))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08_UART7_RTS, 0) | \
					 PIN_AFIO_AF(PF09_ALARM, 1) | \
					 PIN_AFIO_AF(PF10, 0) | \
					 PIN_AFIO_AF(PF11_SPI5_MOSI, 5) | \
					 PIN_AFIO_AF(PF12_ADC4, 0) | \
					 PIN_AFIO_AF(PF13_VDD_5V_HIPOWER_OC, 0) | \
					 PIN_AFIO_AF(PF14_I2C4_SCL, 4) | \
					 PIN_AFIO_AF(PF15_I2C4_SDA, 4))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00_HW_VER_REV_DRIVE) | \
					 PIN_MODE_INPUT(PG01_VDD_BRICK_VALID) | \
					 PIN_MODE_INPUT(PG02_VDD_BRICK2_VALID) | \
					 PIN_MODE_INPUT(PG03_VDD_BRICK3_VALID) | \
					 PIN_MODE_OUTPUT(PG04_VDD_5V_PERIPH_EN) | \
					 PIN_MODE_INPUT(PG05_DRDY1_BMP388) | \
					 PIN_MODE_INPUT(PG06) | \
					 PIN_MODE_INPUT(PG07_SPI_SLAVE6) | \
					 PIN_MODE_OUTPUT(PG08_VDD_3V3_SENSORS4_EN) | \
					 PIN_MODE_ALTERNATE(PG09_SPI1_MISO) | \
					 PIN_MODE_OUTPUT(PG10_VDD_5V_HIPOWER_EN) | \
					 PIN_MODE_ALTERNATE(PG11_SDIO_D2) | \
					 PIN_MODE_INPUT(PG12_ETH_RMII_TXD1) | \
					 PIN_MODE_INPUT(PG13_ETH_RMII_TXD0) | \
					 PIN_MODE_ALTERNATE(PG14_SPI6_MOSI) | \
					 PIN_MODE_OUTPUT(PG15_ETH_POWER_EN))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_OPENDRAIN(PG00_HW_VER_REV_DRIVE) | \
					 PIN_OTYPE_OPENDRAIN(PG01_VDD_BRICK_VALID) | \
					 PIN_OTYPE_OPENDRAIN(PG02_VDD_BRICK2_VALID) | \
					 PIN_OTYPE_OPENDRAIN(PG03_VDD_BRICK3_VALID) | \
					 PIN_OTYPE_PUSHPULL(PG04_VDD_5V_PERIPH_EN) | \
					 PIN_OTYPE_OPENDRAIN(PG05_DRDY1_BMP388) | \
					 PIN_OTYPE_PUSHPULL(PG06) | \
					 PIN_OTYPE_OPENDRAIN(PG07_SPI_SLAVE6) | \
					 PIN_OTYPE_PUSHPULL(PG08_VDD_3V3_SENSORS4_EN) | \
					 PIN_OTYPE_PUSHPULL(PG09_SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(PG10_VDD_5V_HIPOWER_EN) | \
					 PIN_OTYPE_PUSHPULL(PG11_SDIO_D2) | \
					 PIN_OTYPE_OPENDRAIN(PG12_ETH_RMII_TXD1) | \
					 PIN_OTYPE_OPENDRAIN(PG13_ETH_RMII_TXD0) | \
					 PIN_OTYPE_PUSHPULL(PG14_SPI6_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PG15_ETH_POWER_EN))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00_HW_VER_REV_DRIVE) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01_VDD_BRICK_VALID) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02_VDD_BRICK2_VALID) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03_VDD_BRICK3_VALID) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04_VDD_5V_PERIPH_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05_DRDY1_BMP388) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07_SPI_SLAVE6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08_VDD_3V3_SENSORS4_EN) | \
					 PIN_OSPEED_SPEED_HIGH(PG09_SPI1_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG10_VDD_5V_HIPOWER_EN) | \
					 PIN_OSPEED_SPEED_HIGH(PG11_SDIO_D2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG12_ETH_RMII_TXD1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG13_ETH_RMII_TXD0) | \
					 PIN_OSPEED_SPEED_HIGH(PG14_SPI6_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15_ETH_POWER_EN))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00_HW_VER_REV_DRIVE) | \
					 PIN_PUPDR_PULLDOWN(PG01_VDD_BRICK_VALID) | \
					 PIN_PUPDR_PULLDOWN(PG02_VDD_BRICK2_VALID) | \
					 PIN_PUPDR_PULLDOWN(PG03_VDD_BRICK3_VALID) | \
					 PIN_PUPDR_FLOATING(PG04_VDD_5V_PERIPH_EN) | \
					 PIN_PUPDR_PULLDOWN(PG05_DRDY1_BMP388) | \
					 PIN_PUPDR_PULLDOWN(PG06) | \
					 PIN_PUPDR_PULLDOWN(PG07_SPI_SLAVE6) | \
					 PIN_PUPDR_FLOATING(PG08_VDD_3V3_SENSORS4_EN) | \
					 PIN_PUPDR_FLOATING(PG09_SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(PG10_VDD_5V_HIPOWER_EN) | \
					 PIN_PUPDR_PULLUP(PG11_SDIO_D2) | \
					 PIN_PUPDR_PULLDOWN(PG12_ETH_RMII_TXD1) | \
					 PIN_PUPDR_PULLDOWN(PG13_ETH_RMII_TXD0) | \
					 PIN_PUPDR_FLOATING(PG14_SPI6_MOSI) | \
					 PIN_PUPDR_FLOATING(PG15_ETH_POWER_EN))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_HIGH(PG00_HW_VER_REV_DRIVE) | \
					 PIN_ODR_LEVEL_HIGH(PG01_VDD_BRICK_VALID) | \
					 PIN_ODR_LEVEL_HIGH(PG02_VDD_BRICK2_VALID) | \
					 PIN_ODR_LEVEL_HIGH(PG03_VDD_BRICK3_VALID) | \
					 PIN_ODR_LEVEL_LOW(PG04_VDD_5V_PERIPH_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG05_DRDY1_BMP388) | \
					 PIN_ODR_LEVEL_LOW(PG06) | \
					 PIN_ODR_LEVEL_HIGH(PG07_SPI_SLAVE6) | \
					 PIN_ODR_LEVEL_HIGH(PG08_VDD_3V3_SENSORS4_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG09_SPI1_MISO) | \
					 PIN_ODR_LEVEL_LOW(PG10_VDD_5V_HIPOWER_EN) | \
					 PIN_ODR_LEVEL_HIGH(PG11_SDIO_D2) | \
					 PIN_ODR_LEVEL_HIGH(PG12_ETH_RMII_TXD1) | \
					 PIN_ODR_LEVEL_HIGH(PG13_ETH_RMII_TXD0) | \
					 PIN_ODR_LEVEL_HIGH(PG14_SPI6_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PG15_ETH_POWER_EN))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00_HW_VER_REV_DRIVE, 0) | \
					 PIN_AFIO_AF(PG01_VDD_BRICK_VALID, 0) | \
					 PIN_AFIO_AF(PG02_VDD_BRICK2_VALID, 0) | \
					 PIN_AFIO_AF(PG03_VDD_BRICK3_VALID, 0) | \
					 PIN_AFIO_AF(PG04_VDD_5V_PERIPH_EN, 0) | \
					 PIN_AFIO_AF(PG05_DRDY1_BMP388, 0) | \
					 PIN_AFIO_AF(PG06, 0) | \
					 PIN_AFIO_AF(PG07_SPI_SLAVE6, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08_VDD_3V3_SENSORS4_EN, 0) | \
					 PIN_AFIO_AF(PG09_SPI1_MISO, 5) | \
					 PIN_AFIO_AF(PG10_VDD_5V_HIPOWER_EN, 0) | \
					 PIN_AFIO_AF(PG11_SDIO_D2, 10) | \
					 PIN_AFIO_AF(PG12_ETH_RMII_TXD1, 0) | \
					 PIN_AFIO_AF(PG13_ETH_RMII_TXD0, 0) | \
					 PIN_AFIO_AF(PG14_SPI6_MOSI, 5) | \
					 PIN_AFIO_AF(PG15_ETH_POWER_EN, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(PH00_OSC_IN) | \
					 PIN_MODE_ALTERNATE(PH01_OSC_OUT) | \
					 PIN_MODE_OUTPUT(PH02_SPEKTRUM_PWR_EN) | \
					 PIN_MODE_INPUT(PH03_HW_VER_SENS) | \
					 PIN_MODE_INPUT(PH04_HW_REV_SENS) | \
					 PIN_MODE_INPUT(PH05_SPI_SLAVE2) | \
					 PIN_MODE_INPUT(PH06) | \
					 PIN_MODE_ALTERNATE(PH07_SPI5_MISO) | \
					 PIN_MODE_ALTERNATE(PH08_I2C3_SDA) | \
					 PIN_MODE_INPUT(PH09) | \
					 PIN_MODE_ALTERNATE(PH10_SERVO4) | \
					 PIN_MODE_ALTERNATE(PH11_SERVO3) | \
					 PIN_MODE_ALTERNATE(PH12_SERVO2) | \
					 PIN_MODE_ALTERNATE(PH13_UART4_TX) | \
					 PIN_MODE_ALTERNATE(PH14_UART4_RX) | \
					 PIN_MODE_INPUT(PH15_SPI_SLAVE5))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(PH00_OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(PH01_OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(PH02_SPEKTRUM_PWR_EN) | \
					 PIN_OTYPE_OPENDRAIN(PH03_HW_VER_SENS) | \
					 PIN_OTYPE_OPENDRAIN(PH04_HW_REV_SENS) | \
					 PIN_OTYPE_OPENDRAIN(PH05_SPI_SLAVE2) | \
					 PIN_OTYPE_PUSHPULL(PH06) | \
					 PIN_OTYPE_PUSHPULL(PH07_SPI5_MISO) | \
					 PIN_OTYPE_OPENDRAIN(PH08_I2C3_SDA) | \
					 PIN_OTYPE_PUSHPULL(PH09) | \
					 PIN_OTYPE_PUSHPULL(PH10_SERVO4) | \
					 PIN_OTYPE_PUSHPULL(PH11_SERVO3) | \
					 PIN_OTYPE_PUSHPULL(PH12_SERVO2) | \
					 PIN_OTYPE_PUSHPULL(PH13_UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(PH14_UART4_RX) | \
					 PIN_OTYPE_OPENDRAIN(PH15_SPI_SLAVE5))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PH00_OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PH01_OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02_SPEKTRUM_PWR_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03_HW_VER_SENS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04_HW_REV_SENS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH05_SPI_SLAVE2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH06) | \
					 PIN_OSPEED_SPEED_HIGH(PH07_SPI5_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PH08_I2C3_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH09) | \
					 PIN_OSPEED_SPEED_HIGH(PH10_SERVO4) | \
					 PIN_OSPEED_SPEED_HIGH(PH11_SERVO3) | \
					 PIN_OSPEED_SPEED_HIGH(PH12_SERVO2) | \
					 PIN_OSPEED_SPEED_HIGH(PH13_UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PH14_UART4_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15_SPI_SLAVE5))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(PH00_OSC_IN) | \
					 PIN_PUPDR_FLOATING(PH01_OSC_OUT) | \
					 PIN_PUPDR_FLOATING(PH02_SPEKTRUM_PWR_EN) | \
					 PIN_PUPDR_PULLDOWN(PH03_HW_VER_SENS) | \
					 PIN_PUPDR_PULLDOWN(PH04_HW_REV_SENS) | \
					 PIN_PUPDR_PULLDOWN(PH05_SPI_SLAVE2) | \
					 PIN_PUPDR_PULLDOWN(PH06) | \
					 PIN_PUPDR_FLOATING(PH07_SPI5_MISO) | \
					 PIN_PUPDR_PULLUP(PH08_I2C3_SDA) | \
					 PIN_PUPDR_PULLDOWN(PH09) | \
					 PIN_PUPDR_FLOATING(PH10_SERVO4) | \
					 PIN_PUPDR_FLOATING(PH11_SERVO3) | \
					 PIN_PUPDR_FLOATING(PH12_SERVO2) | \
					 PIN_PUPDR_FLOATING(PH13_UART4_TX) | \
					 PIN_PUPDR_FLOATING(PH14_UART4_RX) | \
					 PIN_PUPDR_PULLDOWN(PH15_SPI_SLAVE5))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(PH00_OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(PH01_OSC_OUT) | \
					 PIN_ODR_LEVEL_HIGH(PH02_SPEKTRUM_PWR_EN) | \
					 PIN_ODR_LEVEL_HIGH(PH03_HW_VER_SENS) | \
					 PIN_ODR_LEVEL_HIGH(PH04_HW_REV_SENS) | \
					 PIN_ODR_LEVEL_HIGH(PH05_SPI_SLAVE2) | \
					 PIN_ODR_LEVEL_LOW(PH06) | \
					 PIN_ODR_LEVEL_HIGH(PH07_SPI5_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PH08_I2C3_SDA) | \
					 PIN_ODR_LEVEL_LOW(PH09) | \
					 PIN_ODR_LEVEL_LOW(PH10_SERVO4) | \
					 PIN_ODR_LEVEL_LOW(PH11_SERVO3) | \
					 PIN_ODR_LEVEL_LOW(PH12_SERVO2) | \
					 PIN_ODR_LEVEL_HIGH(PH13_UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(PH14_UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(PH15_SPI_SLAVE5))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(PH00_OSC_IN, 0) | \
					 PIN_AFIO_AF(PH01_OSC_OUT, 0) | \
					 PIN_AFIO_AF(PH02_SPEKTRUM_PWR_EN, 0) | \
					 PIN_AFIO_AF(PH03_HW_VER_SENS, 0) | \
					 PIN_AFIO_AF(PH04_HW_REV_SENS, 0) | \
					 PIN_AFIO_AF(PH05_SPI_SLAVE2, 0) | \
					 PIN_AFIO_AF(PH06, 0) | \
					 PIN_AFIO_AF(PH07_SPI5_MISO, 5))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08_I2C3_SDA, 4) | \
					 PIN_AFIO_AF(PH09, 0) | \
					 PIN_AFIO_AF(PH10_SERVO4, 2) | \
					 PIN_AFIO_AF(PH11_SERVO3, 2) | \
					 PIN_AFIO_AF(PH12_SERVO2, 2) | \
					 PIN_AFIO_AF(PH13_UART4_TX, 8) | \
					 PIN_AFIO_AF(PH14_UART4_RX, 8) | \
					 PIN_AFIO_AF(PH15_SPI_SLAVE5, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_ALTERNATE(PI00_SERVO1) | \
					 PIN_MODE_ALTERNATE(PI01_SPI2_SCK) | \
					 PIN_MODE_ALTERNATE(PI02_SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(PI03_SPI2_MOSI) | \
					 PIN_MODE_INPUT(PI04_SPI_SLAVE3) | \
					 PIN_MODE_ALTERNATE(PI05_PWM_INPUT1) | \
					 PIN_MODE_INPUT(PI06_SPI3_DRDY1) | \
					 PIN_MODE_INPUT(PI07_SPI3_DRDY2) | \
					 PIN_MODE_INPUT(PI08_SPI_SLAVE4) | \
					 PIN_MODE_INPUT(PI09_SPI_SLAVE1) | \
					 PIN_MODE_INPUT(PI10_SPI_SLAVE7) | \
					 PIN_MODE_OUTPUT(PI11_VDD_3V3_SENSORS1_EN) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00_SERVO1) | \
					 PIN_OTYPE_PUSHPULL(PI01_SPI2_SCK) | \
					 PIN_OTYPE_PUSHPULL(PI02_SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(PI03_SPI2_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(PI04_SPI_SLAVE3) | \
					 PIN_OTYPE_PUSHPULL(PI05_PWM_INPUT1) | \
					 PIN_OTYPE_OPENDRAIN(PI06_SPI3_DRDY1) | \
					 PIN_OTYPE_OPENDRAIN(PI07_SPI3_DRDY2) | \
					 PIN_OTYPE_OPENDRAIN(PI08_SPI_SLAVE4) | \
					 PIN_OTYPE_OPENDRAIN(PI09_SPI_SLAVE1) | \
					 PIN_OTYPE_OPENDRAIN(PI10_SPI_SLAVE7) | \
					 PIN_OTYPE_PUSHPULL(PI11_VDD_3V3_SENSORS1_EN) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PI00_SERVO1) | \
					 PIN_OSPEED_SPEED_HIGH(PI01_SPI2_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PI02_SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PI03_SPI2_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI04_SPI_SLAVE3) | \
					 PIN_OSPEED_SPEED_HIGH(PI05_PWM_INPUT1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI06_SPI3_DRDY1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI07_SPI3_DRDY2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI08_SPI_SLAVE4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI09_SPI_SLAVE1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI10_SPI_SLAVE7) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI11_VDD_3V3_SENSORS1_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_FLOATING(PI00_SERVO1) | \
					 PIN_PUPDR_FLOATING(PI01_SPI2_SCK) | \
					 PIN_PUPDR_FLOATING(PI02_SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(PI03_SPI2_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PI04_SPI_SLAVE3) | \
					 PIN_PUPDR_FLOATING(PI05_PWM_INPUT1) | \
					 PIN_PUPDR_PULLDOWN(PI06_SPI3_DRDY1) | \
					 PIN_PUPDR_PULLDOWN(PI07_SPI3_DRDY2) | \
					 PIN_PUPDR_PULLDOWN(PI08_SPI_SLAVE4) | \
					 PIN_PUPDR_PULLDOWN(PI09_SPI_SLAVE1) | \
					 PIN_PUPDR_PULLDOWN(PI10_SPI_SLAVE7) | \
					 PIN_PUPDR_FLOATING(PI11_VDD_3V3_SENSORS1_EN) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00_SERVO1) | \
					 PIN_ODR_LEVEL_HIGH(PI01_SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PI02_SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PI03_SPI2_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PI04_SPI_SLAVE3) | \
					 PIN_ODR_LEVEL_HIGH(PI05_PWM_INPUT1) | \
					 PIN_ODR_LEVEL_HIGH(PI06_SPI3_DRDY1) | \
					 PIN_ODR_LEVEL_HIGH(PI07_SPI3_DRDY2) | \
					 PIN_ODR_LEVEL_HIGH(PI08_SPI_SLAVE4) | \
					 PIN_ODR_LEVEL_HIGH(PI09_SPI_SLAVE1) | \
					 PIN_ODR_LEVEL_HIGH(PI10_SPI_SLAVE7) | \
					 PIN_ODR_LEVEL_HIGH(PI11_VDD_3V3_SENSORS1_EN) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(PI00_SERVO1, 2) | \
					 PIN_AFIO_AF(PI01_SPI2_SCK, 5) | \
					 PIN_AFIO_AF(PI02_SPI2_MISO, 5) | \
					 PIN_AFIO_AF(PI03_SPI2_MOSI, 5) | \
					 PIN_AFIO_AF(PI04_SPI_SLAVE3, 0) | \
					 PIN_AFIO_AF(PI05_PWM_INPUT1, 3) | \
					 PIN_AFIO_AF(PI06_SPI3_DRDY1, 0) | \
					 PIN_AFIO_AF(PI07_SPI3_DRDY2, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(PI08_SPI_SLAVE4, 0) | \
					 PIN_AFIO_AF(PI09_SPI_SLAVE1, 0) | \
					 PIN_AFIO_AF(PI10_SPI_SLAVE7, 0) | \
					 PIN_AFIO_AF(PI11_VDD_3V3_SENSORS1_EN, 0) | \
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

#define AF_PA03_UART2                    7U
#define AF_LINE_UART2                    7U
#define AF_PA05_SPI1_SCK                 5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_PA06_SPI6_MISO                8U
#define AF_LINE_SPI6_MISO                8U
#define AF_PA08_I2C3_SCL                 4U
#define AF_LINE_I2C3_SCL                 4U
#define AF_PA11_USB_DM                   10U
#define AF_LINE_USB_DM                   10U
#define AF_PA12_USB_DP                   10U
#define AF_LINE_USB_DP                   10U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PB02_SPI3_MOSI                7U
#define AF_LINE_SPI3_MOSI                7U
#define AF_PB03_SPI6_SCK                 8U
#define AF_LINE_SPI6_SCK                 8U
#define AF_PB04_SDIO_D3                  9U
#define AF_LINE_SDIO_D3                  9U
#define AF_PB05_SPI1_MOSI                5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_PB06_UART1_TX                 7U
#define AF_LINE_UART1_TX                 7U
#define AF_PB07_UART1_RX                 7U
#define AF_LINE_UART1_RX                 7U
#define AF_PB08_I2C1_SCL                 4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_PB09_I2C1_SDA                 4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_PB12_CAN2_RX                  9U
#define AF_LINE_CAN2_RX                  9U
#define AF_PB13_CAN2_TX                  9U
#define AF_LINE_CAN2_TX                  9U
#define AF_PB14_SDIO_D0                  9U
#define AF_LINE_SDIO_D0                  9U
#define AF_PB15_SDIO_D1                  9U
#define AF_LINE_SDIO_D1                  9U
#define AF_PC06_UART6_TX                 7U
#define AF_LINE_UART6_TX                 7U
#define AF_PC07_UART6_RX                 7U
#define AF_LINE_UART6_RX                 7U
#define AF_PC10_SPI3_SCK                 6U
#define AF_LINE_SPI3_SCK                 6U
#define AF_PC11_SPI3_MISO                6U
#define AF_LINE_SPI3_MISO                6U
#define AF_PC12_UART5_TX                 8U
#define AF_LINE_UART5_TX                 8U
#define AF_PD00_CAN1_RX                  9U
#define AF_LINE_CAN1_RX                  9U
#define AF_PD01_CAN1_TX                  9U
#define AF_LINE_CAN1_TX                  9U
#define AF_PD02_UART5_RX                 8U
#define AF_LINE_UART5_RX                 8U
#define AF_PD05_UART2_TX                 7U
#define AF_LINE_UART2_TX                 7U
#define AF_PD06_SDIO_CK                  11U
#define AF_LINE_SDIO_CK                  11U
#define AF_PD07_SDIO_CMD                 11U
#define AF_LINE_SDIO_CMD                 11U
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
#define AF_PE08_UART7_TX                 7U
#define AF_LINE_UART7_TX                 7U
#define AF_PE12_SPI4_SCK                 5U
#define AF_LINE_SPI4_SCK                 5U
#define AF_PE13_SPI4_MISO                5U
#define AF_LINE_SPI4_MISO                5U
#define AF_PE14_SPI4_MOSI                5U
#define AF_LINE_SPI4_MOSI                5U
#define AF_PF00_I2C2_SDA                 4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_PF01_I2C2_SCL                 4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_PF06_UART7_RX                 7U
#define AF_LINE_UART7_RX                 7U
#define AF_PF07_SPI5_SCK                 5U
#define AF_LINE_SPI5_SCK                 5U
#define AF_PF09_ALARM                    1U
#define AF_LINE_ALARM                    1U
#define AF_PF11_SPI5_MOSI                5U
#define AF_LINE_SPI5_MOSI                5U
#define AF_PF14_I2C4_SCL                 4U
#define AF_LINE_I2C4_SCL                 4U
#define AF_PF15_I2C4_SDA                 4U
#define AF_LINE_I2C4_SDA                 4U
#define AF_PG09_SPI1_MISO                5U
#define AF_LINE_SPI1_MISO                5U
#define AF_PG11_SDIO_D2                  10U
#define AF_LINE_SDIO_D2                  10U
#define AF_PG14_SPI6_MOSI                5U
#define AF_LINE_SPI6_MOSI                5U
#define AF_PH00_OSC_IN                   0U
#define AF_LINE_OSC_IN                   0U
#define AF_PH01_OSC_OUT                  0U
#define AF_LINE_OSC_OUT                  0U
#define AF_PH07_SPI5_MISO                5U
#define AF_LINE_SPI5_MISO                5U
#define AF_PH08_I2C3_SDA                 4U
#define AF_LINE_I2C3_SDA                 4U
#define AF_PH10_SERVO4                   2U
#define AF_LINE_SERVO4                   2U
#define AF_PH11_SERVO3                   2U
#define AF_LINE_SERVO3                   2U
#define AF_PH12_SERVO2                   2U
#define AF_LINE_SERVO2                   2U
#define AF_PH13_UART4_TX                 8U
#define AF_LINE_UART4_TX                 8U
#define AF_PH14_UART4_RX                 8U
#define AF_LINE_UART4_RX                 8U
#define AF_PI00_SERVO1                   2U
#define AF_LINE_SERVO1                   2U
#define AF_PI01_SPI2_SCK                 5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_PI02_SPI2_MISO                5U
#define AF_LINE_SPI2_MISO                5U
#define AF_PI03_SPI2_MOSI                5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_PI05_PWM_INPUT1               3U
#define AF_LINE_PWM_INPUT1               3U


#define ADC1_ADC	 1
#define ADC1_ADC_FN	 INP
#define ADC1_ADC_INP	 16
#define UART2_USART	 2
#define UART2_USART_FN	 RX
#define UART2_USART_AF	 7
#define ADC2_ADC	 1
#define ADC2_ADC_FN	 INP
#define ADC2_ADC_INP	 18
#define ADC3_ADC	 1
#define ADC3_ADC_FN	 INP
#define ADC3_ADC_INP	 9
#define ADC5_ADC	 1
#define ADC5_ADC_FN	 INP
#define ADC5_ADC_INP	 5
#define ADC6_ADC	 3
#define ADC6_ADC_FN	 INP
#define ADC6_ADC_INP	 0
#define ADC7_ADC	 3
#define ADC7_ADC_FN	 INP
#define ADC7_ADC_INP	 1
#define SERVO5_TIM	 4
#define SERVO5_TIM_FN	 CH
#define SERVO5_TIM_CH	 2
#define SERVO5_TIM_AF	 2
#define SERVO6_TIM	 4
#define SERVO6_TIM_FN	 CH
#define SERVO6_TIM_CH	 3
#define SERVO6_TIM_AF	 2
#define ALARM_TIM	 17
#define ALARM_TIM_FN	 CH
#define ALARM_TIM_CH	 1
#define ALARM_TIM_AF	 1
#define ADC4_ADC	 1
#define ADC4_ADC_FN	 INP
#define ADC4_ADC_INP	 6
#define SERVO4_TIM	 5
#define SERVO4_TIM_FN	 CH
#define SERVO4_TIM_CH	 1
#define SERVO4_TIM_AF	 2
#define SERVO3_TIM	 5
#define SERVO3_TIM_FN	 CH
#define SERVO3_TIM_CH	 2
#define SERVO3_TIM_AF	 2
#define SERVO2_TIM	 5
#define SERVO2_TIM_FN	 CH
#define SERVO2_TIM_CH	 3
#define SERVO2_TIM_AF	 2
#define SERVO1_TIM	 5
#define SERVO1_TIM_FN	 CH
#define SERVO1_TIM_CH	 4
#define SERVO1_TIM_AF	 2
#define PWM_INPUT1_TIM	 8
#define PWM_INPUT1_TIM_FN	 CH
#define PWM_INPUT1_TIM_CH	 1
#define PWM_INPUT1_TIM_AF	 3

#define BOARD_GROUP_DECLFOREACH(line, group) \
  static const ioline_t group ## _ARRAY[] = {group}; \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)

#define BOARD_GROUP_FOREACH(line, group) \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)


#define BOARD_GROUP_DECLFOR(array, index, group)  \
  static const ioline_t group ## _ARRAY[] = {group};    \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define BOARD_GROUP_FOR(array, index, group)  \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define ENERGY_SAVE_INPUTS \
	LINE_LED4, \
	LINE_SERVO5, \
	LINE_SERVO6, \
	LINE_LED1, \
	LINE_LED2, \
	LINE_LED3, \
	LINE_SPI_SLAVE6, \
	LINE_SPI_SLAVE2, \
	LINE_SERVO4, \
	LINE_SERVO3, \
	LINE_SERVO2, \
	LINE_SPI_SLAVE5, \
	LINE_SERVO1, \
	LINE_SPI_SLAVE3, \
	LINE_SPI_SLAVE4, \
	LINE_SPI_SLAVE1, \
	LINE_SPI_SLAVE7
#define ENERGY_SAVE_INPUTS_SIZE 	 17

#define ENERGY_SAVE_LOWS \
	LINE_VDD_3V3_SENSORS3_EN, \
	LINE_VDD_3V3_SENSORS2_EN, \
	LINE_ALARM, \
	LINE_VDD_3V3_SENSORS4_EN, \
	LINE_ETH_POWER_EN, \
	LINE_VDD_3V3_SENSORS1_EN
#define ENERGY_SAVE_LOWS_SIZE 	 6

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

