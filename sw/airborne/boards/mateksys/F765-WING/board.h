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
#define BOARD_MATEKF765_WING
#define BOARD_NAME                  "MATEKSYS F765-WING"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F767xx

/*
 * IO pins assignments.
 */
#define	S1                             0U
#define	S2                             1U
#define	S3                             2U
#define	S4                             3U
#define	PA04                           4U
#define	SPI1_SCK                       5U
#define	SPI1_MISO                      6U
#define	SPI1_MOSI                      7U
#define	LED_WS2812                     8U
#define	UART1_TX                       9U
#define	UART1_RX                       10U
#define	OTG_FS_DM                      11U
#define	OTG_FS_DP                      12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	PA15                           15U

#define	S5                             0U
#define	S6                             1U
#define	PB02                           2U
#define	SPI3_SCK                       3U
#define	SPI3_MISO                      4U
#define	SPI3_MOSI                      5U
#define	I2C1_SCL                       6U
#define	I2C1_SDA                       7U
#define	UART5_RX                       8U
#define	ALARM                          9U
#define	I2C2_SCL                       10U
#define	I2C2_SDA                       11U
#define	OSD_CS                         12U
#define	SPI2_SCK                       13U
#define	SPI2_MISO                      14U
#define	SPI2_MOSI                      15U

#define	AIRSPEED                       0U
#define	RSSI                           1U
#define	VBAT_MEAS                      2U
#define	CURRENT_MEAS                   3U
#define	IMU1_CS                        4U
#define	PC05                           5U
#define	RC2                            6U
#define	RC1                            7U
#define	SDMMC1_D0                      8U
#define	SDMMC1_D1                      9U
#define	SDMMC1_D2                      10U
#define	SDMMC1_D3                      11U
#define	SDMMC1_CK                      12U
#define	PC13                           13U
#define	OSC32_IN                       14U
#define	OSC32_OUT                      15U

#define	UART4_RX                       0U
#define	UART4_TX                       1U
#define	SDMMC1_CMD                     2U
#define	PD03                           3U
#define	PD04                           4U
#define	UART2_TX                       5U
#define	UART2_RX                       6U
#define	IMU2_CS                        7U
#define	UART3_TX                       8U
#define	UART3_RX                       9U
#define	LED1                           10U
#define	LED2                           11U
#define	S7                             12U
#define	S8                             13U
#define	S9                             14U
#define	S10                            15U

#define	UART8_RX                       0U
#define	UART8_TX                       1U
#define	PE02                           2U
#define	PE03                           3U
#define	PE04                           4U
#define	S11                            5U
#define	S12                            6U
#define	UART7_RX                       7U
#define	UART7_TX                       8U
#define	UART7_RTS                      9U
#define	UART7_CTS                      10U
#define	SPI4_CS                        11U
#define	SPI4_CLK                       12U
#define	SPI4_MISO                      13U
#define	SPI4_MOSI                      14U
#define	PE15                           15U

#define	PF00                           0U
#define	PF01                           1U
#define	PF02                           2U
#define	PF03                           3U
#define	PF04                           4U
#define	PF05                           5U
#define	PF06                           6U
#define	PF07                           7U
#define	PF08                           8U
#define	PF09                           9U
#define	PF10                           10U
#define	PF11                           11U
#define	PF12                           12U
#define	PF13                           13U
#define	PF14                           14U
#define	PF15                           15U

#define	PG00                           0U
#define	PG01                           1U
#define	PG02                           2U
#define	PG03                           3U
#define	PG04                           4U
#define	PG05                           5U
#define	PG06                           6U
#define	PG07                           7U
#define	PG08                           8U
#define	PG09                           9U
#define	PG10                           10U
#define	PG11                           11U
#define	PG12                           12U
#define	PG13                           13U
#define	PG14                           14U
#define	PG15                           15U

#define	OSC_IN                         0U
#define	OSC_OUT                        1U
#define	PH02                           2U
#define	PH03                           3U
#define	PH04                           4U
#define	PH05                           5U
#define	PH06                           6U
#define	PH07                           7U
#define	PH08                           8U
#define	PH09                           9U
#define	PH10                           10U
#define	PH11                           11U
#define	PH12                           12U
#define	PH13                           13U
#define	PH14                           14U
#define	PH15                           15U

#define	PI00                           0U
#define	PI01                           1U
#define	PI02                           2U
#define	PI03                           3U
#define	PI04                           4U
#define	PI05                           5U
#define	PI06                           6U
#define	PI07                           7U
#define	PI08                           8U
#define	PI09                           9U
#define	PI10                           10U
#define	PI11                           11U
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
#define	LINE_S1                        PAL_LINE(GPIOA, 0U)
#define	LINE_S2                        PAL_LINE(GPIOA, 1U)
#define	LINE_S3                        PAL_LINE(GPIOA, 2U)
#define	LINE_S4                        PAL_LINE(GPIOA, 3U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOA, 7U)
#define	LINE_LED_WS2812                PAL_LINE(GPIOA, 8U)
#define	LINE_UART1_TX                  PAL_LINE(GPIOA, 9U)
#define	LINE_UART1_RX                  PAL_LINE(GPIOA, 10U)
#define	LINE_OTG_FS_DM                 PAL_LINE(GPIOA, 11U)
#define	LINE_OTG_FS_DP                 PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)

#define	LINE_S5                        PAL_LINE(GPIOB, 0U)
#define	LINE_S6                        PAL_LINE(GPIOB, 1U)
#define	LINE_SPI3_SCK                  PAL_LINE(GPIOB, 3U)
#define	LINE_SPI3_MISO                 PAL_LINE(GPIOB, 4U)
#define	LINE_SPI3_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 6U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 7U)
#define	LINE_UART5_RX                  PAL_LINE(GPIOB, 8U)
#define	LINE_ALARM                     PAL_LINE(GPIOB, 9U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define	LINE_OSD_CS                    PAL_LINE(GPIOB, 12U)
#define	LINE_SPI2_SCK                  PAL_LINE(GPIOB, 13U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOB, 14U)
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOB, 15U)

#define	LINE_AIRSPEED                  PAL_LINE(GPIOC, 0U)
#define	LINE_RSSI                      PAL_LINE(GPIOC, 1U)
#define	LINE_VBAT_MEAS                 PAL_LINE(GPIOC, 2U)
#define	LINE_CURRENT_MEAS              PAL_LINE(GPIOC, 3U)
#define	LINE_IMU1_CS                   PAL_LINE(GPIOC, 4U)
#define	LINE_RC2                       PAL_LINE(GPIOC, 6U)
#define	LINE_RC1                       PAL_LINE(GPIOC, 7U)
#define	LINE_SDMMC1_D0                 PAL_LINE(GPIOC, 8U)
#define	LINE_SDMMC1_D1                 PAL_LINE(GPIOC, 9U)
#define	LINE_SDMMC1_D2                 PAL_LINE(GPIOC, 10U)
#define	LINE_SDMMC1_D3                 PAL_LINE(GPIOC, 11U)
#define	LINE_SDMMC1_CK                 PAL_LINE(GPIOC, 12U)
#define	LINE_OSC32_IN                  PAL_LINE(GPIOC, 14U)
#define	LINE_OSC32_OUT                 PAL_LINE(GPIOC, 15U)

#define	LINE_UART4_RX                  PAL_LINE(GPIOD, 0U)
#define	LINE_UART4_TX                  PAL_LINE(GPIOD, 1U)
#define	LINE_SDMMC1_CMD                PAL_LINE(GPIOD, 2U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOD, 5U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOD, 6U)
#define	LINE_IMU2_CS                   PAL_LINE(GPIOD, 7U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOD, 8U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOD, 9U)
#define	LINE_LED1                      PAL_LINE(GPIOD, 10U)
#define	LINE_LED2                      PAL_LINE(GPIOD, 11U)
#define	LINE_S7                        PAL_LINE(GPIOD, 12U)
#define	LINE_S8                        PAL_LINE(GPIOD, 13U)
#define	LINE_S9                        PAL_LINE(GPIOD, 14U)
#define	LINE_S10                       PAL_LINE(GPIOD, 15U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE_S11                       PAL_LINE(GPIOE, 5U)
#define	LINE_S12                       PAL_LINE(GPIOE, 6U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOE, 7U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOE, 8U)
#define	LINE_UART7_RTS                 PAL_LINE(GPIOE, 9U)
#define	LINE_UART7_CTS                 PAL_LINE(GPIOE, 10U)
#define	LINE_SPI4_CS                   PAL_LINE(GPIOE, 11U)
#define	LINE_SPI4_CLK                  PAL_LINE(GPIOE, 12U)
#define	LINE_SPI4_MISO                 PAL_LINE(GPIOE, 13U)
#define	LINE_SPI4_MOSI                 PAL_LINE(GPIOE, 14U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)


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

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(S1) | \
					 PIN_MODE_ALTERNATE(S2) | \
					 PIN_MODE_ALTERNATE(S3) | \
					 PIN_MODE_ALTERNATE(S4) | \
					 PIN_MODE_INPUT(PA04) | \
					 PIN_MODE_ALTERNATE(SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(SPI1_MISO) | \
					 PIN_MODE_ALTERNATE(SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(LED_WS2812) | \
					 PIN_MODE_ALTERNATE(UART1_TX) | \
					 PIN_MODE_ALTERNATE(UART1_RX) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DM) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_INPUT(PA15))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(S1) | \
					 PIN_OTYPE_PUSHPULL(S2) | \
					 PIN_OTYPE_PUSHPULL(S3) | \
					 PIN_OTYPE_PUSHPULL(S4) | \
					 PIN_OTYPE_PUSHPULL(PA04) | \
					 PIN_OTYPE_PUSHPULL(SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(LED_WS2812) | \
					 PIN_OTYPE_PUSHPULL(UART1_TX) | \
					 PIN_OTYPE_PUSHPULL(UART1_RX) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DM) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(S1) | \
					 PIN_OSPEED_SPEED_HIGH(S2) | \
					 PIN_OSPEED_SPEED_HIGH(S3) | \
					 PIN_OSPEED_SPEED_HIGH(S4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(LED_WS2812) | \
					 PIN_OSPEED_SPEED_HIGH(UART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DM) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA15))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(S1) | \
					 PIN_PUPDR_FLOATING(S2) | \
					 PIN_PUPDR_FLOATING(S3) | \
					 PIN_PUPDR_FLOATING(S4) | \
					 PIN_PUPDR_PULLDOWN(PA04) | \
					 PIN_PUPDR_FLOATING(SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(LED_WS2812) | \
					 PIN_PUPDR_FLOATING(UART1_TX) | \
					 PIN_PUPDR_FLOATING(UART1_RX) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DM) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_PULLDOWN(PA15))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(S1) | \
					 PIN_ODR_LEVEL_LOW(S2) | \
					 PIN_ODR_LEVEL_LOW(S3) | \
					 PIN_ODR_LEVEL_LOW(S4) | \
					 PIN_ODR_LEVEL_LOW(PA04) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MOSI) | \
					 PIN_ODR_LEVEL_LOW(LED_WS2812) | \
					 PIN_ODR_LEVEL_HIGH(UART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DM) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PA15))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(S1, 1) | \
					 PIN_AFIO_AF(S2, 1) | \
					 PIN_AFIO_AF(S3, 2) | \
					 PIN_AFIO_AF(S4, 2) | \
					 PIN_AFIO_AF(PA04, 0) | \
					 PIN_AFIO_AF(SPI1_SCK, 5) | \
					 PIN_AFIO_AF(SPI1_MISO, 5) | \
					 PIN_AFIO_AF(SPI1_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(LED_WS2812, 1) | \
					 PIN_AFIO_AF(UART1_TX, 7) | \
					 PIN_AFIO_AF(UART1_RX, 7) | \
					 PIN_AFIO_AF(OTG_FS_DM, 10) | \
					 PIN_AFIO_AF(OTG_FS_DP, 10) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(PA15, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_ALTERNATE(S5) | \
					 PIN_MODE_ALTERNATE(S6) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_ALTERNATE(SPI3_SCK) | \
					 PIN_MODE_ALTERNATE(SPI3_MISO) | \
					 PIN_MODE_ALTERNATE(SPI3_MOSI) | \
					 PIN_MODE_ALTERNATE(I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(I2C1_SDA) | \
					 PIN_MODE_ALTERNATE(UART5_RX) | \
					 PIN_MODE_ALTERNATE(ALARM) | \
					 PIN_MODE_ALTERNATE(I2C2_SCL) | \
					 PIN_MODE_ALTERNATE(I2C2_SDA) | \
					 PIN_MODE_OUTPUT(OSD_CS) | \
					 PIN_MODE_ALTERNATE(SPI2_SCK) | \
					 PIN_MODE_ALTERNATE(SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(SPI2_MOSI))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(S5) | \
					 PIN_OTYPE_PUSHPULL(S6) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_PUSHPULL(SPI3_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI3_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI3_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SDA) | \
					 PIN_OTYPE_PUSHPULL(UART5_RX) | \
					 PIN_OTYPE_PUSHPULL(ALARM) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SDA) | \
					 PIN_OTYPE_PUSHPULL(OSD_CS) | \
					 PIN_OTYPE_PUSHPULL(SPI2_SCK) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MOSI))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(S5) | \
					 PIN_OSPEED_SPEED_HIGH(S6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_HIGH(SPI3_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI3_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI3_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(UART5_RX) | \
					 PIN_OSPEED_SPEED_HIGH(ALARM) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(OSD_CS) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MOSI))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(S5) | \
					 PIN_PUPDR_FLOATING(S6) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_FLOATING(SPI3_SCK) | \
					 PIN_PUPDR_FLOATING(SPI3_MISO) | \
					 PIN_PUPDR_FLOATING(SPI3_MOSI) | \
					 PIN_PUPDR_PULLUP(I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(I2C1_SDA) | \
					 PIN_PUPDR_FLOATING(UART5_RX) | \
					 PIN_PUPDR_FLOATING(ALARM) | \
					 PIN_PUPDR_PULLUP(I2C2_SCL) | \
					 PIN_PUPDR_PULLUP(I2C2_SDA) | \
					 PIN_PUPDR_FLOATING(OSD_CS) | \
					 PIN_PUPDR_FLOATING(SPI2_SCK) | \
					 PIN_PUPDR_FLOATING(SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(SPI2_MOSI))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(S5) | \
					 PIN_ODR_LEVEL_LOW(S6) | \
					 PIN_ODR_LEVEL_LOW(PB02) | \
					 PIN_ODR_LEVEL_HIGH(SPI3_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI3_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI3_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(UART5_RX) | \
					 PIN_ODR_LEVEL_LOW(ALARM) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(OSD_CS) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MOSI))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(S5, 3) | \
					 PIN_AFIO_AF(S6, 3) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(SPI3_SCK, 6) | \
					 PIN_AFIO_AF(SPI3_MISO, 6) | \
					 PIN_AFIO_AF(SPI3_MOSI, 6) | \
					 PIN_AFIO_AF(I2C1_SCL, 4) | \
					 PIN_AFIO_AF(I2C1_SDA, 4))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(UART5_RX, 7) | \
					 PIN_AFIO_AF(ALARM, 3) | \
					 PIN_AFIO_AF(I2C2_SCL, 4) | \
					 PIN_AFIO_AF(I2C2_SDA, 4) | \
					 PIN_AFIO_AF(OSD_CS, 0) | \
					 PIN_AFIO_AF(SPI2_SCK, 5) | \
					 PIN_AFIO_AF(SPI2_MISO, 5) | \
					 PIN_AFIO_AF(SPI2_MOSI, 5))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(AIRSPEED) | \
					 PIN_MODE_ANALOG(RSSI) | \
					 PIN_MODE_ANALOG(VBAT_MEAS) | \
					 PIN_MODE_ANALOG(CURRENT_MEAS) | \
					 PIN_MODE_OUTPUT(IMU1_CS) | \
					 PIN_MODE_INPUT(PC05) | \
					 PIN_MODE_INPUT(RC2) | \
					 PIN_MODE_INPUT(RC1) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D0) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D1) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D2) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D3) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CK) | \
					 PIN_MODE_INPUT(PC13) | \
					 PIN_MODE_ALTERNATE(OSC32_IN) | \
					 PIN_MODE_ALTERNATE(OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(AIRSPEED) | \
					 PIN_OTYPE_PUSHPULL(RSSI) | \
					 PIN_OTYPE_PUSHPULL(VBAT_MEAS) | \
					 PIN_OTYPE_PUSHPULL(CURRENT_MEAS) | \
					 PIN_OTYPE_PUSHPULL(IMU1_CS) | \
					 PIN_OTYPE_PUSHPULL(PC05) | \
					 PIN_OTYPE_OPENDRAIN(RC2) | \
					 PIN_OTYPE_OPENDRAIN(RC1) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D0) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D1) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D2) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D3) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CK) | \
					 PIN_OTYPE_PUSHPULL(PC13) | \
					 PIN_OTYPE_PUSHPULL(OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(AIRSPEED) | \
					 PIN_OSPEED_SPEED_VERYLOW(RSSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(VBAT_MEAS) | \
					 PIN_OSPEED_SPEED_VERYLOW(CURRENT_MEAS) | \
					 PIN_OSPEED_SPEED_HIGH(IMU1_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_VERYLOW(RC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(RC1) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(AIRSPEED) | \
					 PIN_PUPDR_FLOATING(RSSI) | \
					 PIN_PUPDR_FLOATING(VBAT_MEAS) | \
					 PIN_PUPDR_FLOATING(CURRENT_MEAS) | \
					 PIN_PUPDR_FLOATING(IMU1_CS) | \
					 PIN_PUPDR_PULLDOWN(PC05) | \
					 PIN_PUPDR_PULLDOWN(RC2) | \
					 PIN_PUPDR_PULLDOWN(RC1) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D0) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D1) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D2) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D3) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CK) | \
					 PIN_PUPDR_PULLDOWN(PC13) | \
					 PIN_PUPDR_FLOATING(OSC32_IN) | \
					 PIN_PUPDR_FLOATING(OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(AIRSPEED) | \
					 PIN_ODR_LEVEL_LOW(RSSI) | \
					 PIN_ODR_LEVEL_LOW(VBAT_MEAS) | \
					 PIN_ODR_LEVEL_LOW(CURRENT_MEAS) | \
					 PIN_ODR_LEVEL_HIGH(IMU1_CS) | \
					 PIN_ODR_LEVEL_LOW(PC05) | \
					 PIN_ODR_LEVEL_HIGH(RC2) | \
					 PIN_ODR_LEVEL_HIGH(RC1) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CK) | \
					 PIN_ODR_LEVEL_LOW(PC13) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(AIRSPEED, 0) | \
					 PIN_AFIO_AF(RSSI, 0) | \
					 PIN_AFIO_AF(VBAT_MEAS, 0) | \
					 PIN_AFIO_AF(CURRENT_MEAS, 0) | \
					 PIN_AFIO_AF(IMU1_CS, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(RC2, 0) | \
					 PIN_AFIO_AF(RC1, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDMMC1_D0, 12) | \
					 PIN_AFIO_AF(SDMMC1_D1, 12) | \
					 PIN_AFIO_AF(SDMMC1_D2, 12) | \
					 PIN_AFIO_AF(SDMMC1_D3, 12) | \
					 PIN_AFIO_AF(SDMMC1_CK, 12) | \
					 PIN_AFIO_AF(PC13, 0) | \
					 PIN_AFIO_AF(OSC32_IN, 0) | \
					 PIN_AFIO_AF(OSC32_OUT, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(UART4_RX) | \
					 PIN_MODE_ALTERNATE(UART4_TX) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CMD) | \
					 PIN_MODE_INPUT(PD03) | \
					 PIN_MODE_INPUT(PD04) | \
					 PIN_MODE_ALTERNATE(UART2_TX) | \
					 PIN_MODE_ALTERNATE(UART2_RX) | \
					 PIN_MODE_OUTPUT(IMU2_CS) | \
					 PIN_MODE_ALTERNATE(UART3_TX) | \
					 PIN_MODE_ALTERNATE(UART3_RX) | \
					 PIN_MODE_OUTPUT(LED1) | \
					 PIN_MODE_OUTPUT(LED2) | \
					 PIN_MODE_ALTERNATE(S7) | \
					 PIN_MODE_ALTERNATE(S8) | \
					 PIN_MODE_ALTERNATE(S9) | \
					 PIN_MODE_ALTERNATE(S10))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CMD) | \
					 PIN_OTYPE_PUSHPULL(PD03) | \
					 PIN_OTYPE_PUSHPULL(PD04) | \
					 PIN_OTYPE_PUSHPULL(UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(IMU2_CS) | \
					 PIN_OTYPE_PUSHPULL(UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(UART3_RX) | \
					 PIN_OTYPE_PUSHPULL(LED1) | \
					 PIN_OTYPE_PUSHPULL(LED2) | \
					 PIN_OTYPE_PUSHPULL(S7) | \
					 PIN_OTYPE_PUSHPULL(S8) | \
					 PIN_OTYPE_PUSHPULL(S9) | \
					 PIN_OTYPE_PUSHPULL(S10))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART4_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(IMU2_CS) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED2) | \
					 PIN_OSPEED_SPEED_HIGH(S7) | \
					 PIN_OSPEED_SPEED_HIGH(S8) | \
					 PIN_OSPEED_SPEED_HIGH(S9) | \
					 PIN_OSPEED_SPEED_HIGH(S10))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(UART4_RX) | \
					 PIN_PUPDR_FLOATING(UART4_TX) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CMD) | \
					 PIN_PUPDR_PULLDOWN(PD03) | \
					 PIN_PUPDR_PULLDOWN(PD04) | \
					 PIN_PUPDR_FLOATING(UART2_TX) | \
					 PIN_PUPDR_FLOATING(UART2_RX) | \
					 PIN_PUPDR_FLOATING(IMU2_CS) | \
					 PIN_PUPDR_FLOATING(UART3_TX) | \
					 PIN_PUPDR_FLOATING(UART3_RX) | \
					 PIN_PUPDR_FLOATING(LED1) | \
					 PIN_PUPDR_FLOATING(LED2) | \
					 PIN_PUPDR_FLOATING(S7) | \
					 PIN_PUPDR_FLOATING(S8) | \
					 PIN_PUPDR_FLOATING(S9) | \
					 PIN_PUPDR_FLOATING(S10))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CMD) | \
					 PIN_ODR_LEVEL_LOW(PD03) | \
					 PIN_ODR_LEVEL_LOW(PD04) | \
					 PIN_ODR_LEVEL_HIGH(UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(IMU2_CS) | \
					 PIN_ODR_LEVEL_HIGH(UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART3_RX) | \
					 PIN_ODR_LEVEL_LOW(LED1) | \
					 PIN_ODR_LEVEL_LOW(LED2) | \
					 PIN_ODR_LEVEL_LOW(S7) | \
					 PIN_ODR_LEVEL_LOW(S8) | \
					 PIN_ODR_LEVEL_LOW(S9) | \
					 PIN_ODR_LEVEL_LOW(S10))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(UART4_RX, 8) | \
					 PIN_AFIO_AF(UART4_TX, 8) | \
					 PIN_AFIO_AF(SDMMC1_CMD, 12) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(UART2_TX, 7) | \
					 PIN_AFIO_AF(UART2_RX, 7) | \
					 PIN_AFIO_AF(IMU2_CS, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(UART3_TX, 7) | \
					 PIN_AFIO_AF(UART3_RX, 7) | \
					 PIN_AFIO_AF(LED1, 0) | \
					 PIN_AFIO_AF(LED2, 0) | \
					 PIN_AFIO_AF(S7, 2) | \
					 PIN_AFIO_AF(S8, 2) | \
					 PIN_AFIO_AF(S9, 2) | \
					 PIN_AFIO_AF(S10, 2))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(UART8_RX) | \
					 PIN_MODE_ALTERNATE(UART8_TX) | \
					 PIN_MODE_INPUT(PE02) | \
					 PIN_MODE_INPUT(PE03) | \
					 PIN_MODE_INPUT(PE04) | \
					 PIN_MODE_ALTERNATE(S11) | \
					 PIN_MODE_ALTERNATE(S12) | \
					 PIN_MODE_ALTERNATE(UART7_RX) | \
					 PIN_MODE_ALTERNATE(UART7_TX) | \
					 PIN_MODE_ALTERNATE(UART7_RTS) | \
					 PIN_MODE_ALTERNATE(UART7_CTS) | \
					 PIN_MODE_OUTPUT(SPI4_CS) | \
					 PIN_MODE_ALTERNATE(SPI4_CLK) | \
					 PIN_MODE_ALTERNATE(SPI4_MISO) | \
					 PIN_MODE_ALTERNATE(SPI4_MOSI) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(UART8_TX) | \
					 PIN_OTYPE_PUSHPULL(PE02) | \
					 PIN_OTYPE_PUSHPULL(PE03) | \
					 PIN_OTYPE_PUSHPULL(PE04) | \
					 PIN_OTYPE_PUSHPULL(S11) | \
					 PIN_OTYPE_PUSHPULL(S12) | \
					 PIN_OTYPE_PUSHPULL(UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(UART7_TX) | \
					 PIN_OTYPE_PUSHPULL(UART7_RTS) | \
					 PIN_OTYPE_PUSHPULL(UART7_CTS) | \
					 PIN_OTYPE_PUSHPULL(SPI4_CS) | \
					 PIN_OTYPE_PUSHPULL(SPI4_CLK) | \
					 PIN_OTYPE_PUSHPULL(SPI4_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI4_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART8_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04) | \
					 PIN_OSPEED_SPEED_HIGH(S11) | \
					 PIN_OSPEED_SPEED_HIGH(S12) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_CTS) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_CS) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(UART8_RX) | \
					 PIN_PUPDR_FLOATING(UART8_TX) | \
					 PIN_PUPDR_PULLDOWN(PE02) | \
					 PIN_PUPDR_PULLDOWN(PE03) | \
					 PIN_PUPDR_PULLDOWN(PE04) | \
					 PIN_PUPDR_FLOATING(S11) | \
					 PIN_PUPDR_FLOATING(S12) | \
					 PIN_PUPDR_FLOATING(UART7_RX) | \
					 PIN_PUPDR_FLOATING(UART7_TX) | \
					 PIN_PUPDR_FLOATING(UART7_RTS) | \
					 PIN_PUPDR_FLOATING(UART7_CTS) | \
					 PIN_PUPDR_FLOATING(SPI4_CS) | \
					 PIN_PUPDR_FLOATING(SPI4_CLK) | \
					 PIN_PUPDR_FLOATING(SPI4_MISO) | \
					 PIN_PUPDR_FLOATING(SPI4_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART8_TX) | \
					 PIN_ODR_LEVEL_LOW(PE02) | \
					 PIN_ODR_LEVEL_LOW(PE03) | \
					 PIN_ODR_LEVEL_LOW(PE04) | \
					 PIN_ODR_LEVEL_LOW(S11) | \
					 PIN_ODR_LEVEL_LOW(S12) | \
					 PIN_ODR_LEVEL_HIGH(UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART7_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART7_RTS) | \
					 PIN_ODR_LEVEL_HIGH(UART7_CTS) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_CS) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_CLK) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(UART8_RX, 8) | \
					 PIN_AFIO_AF(UART8_TX, 8) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(PE04, 0) | \
					 PIN_AFIO_AF(S11, 3) | \
					 PIN_AFIO_AF(S12, 3) | \
					 PIN_AFIO_AF(UART7_RX, 8))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(UART7_TX, 8) | \
					 PIN_AFIO_AF(UART7_RTS, 8) | \
					 PIN_AFIO_AF(UART7_CTS, 8) | \
					 PIN_AFIO_AF(SPI4_CS, 0) | \
					 PIN_AFIO_AF(SPI4_CLK, 5) | \
					 PIN_AFIO_AF(SPI4_MISO, 5) | \
					 PIN_AFIO_AF(SPI4_MOSI, 5) | \
					 PIN_AFIO_AF(PE15, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_INPUT(PF00) | \
					 PIN_MODE_INPUT(PF01) | \
					 PIN_MODE_INPUT(PF02) | \
					 PIN_MODE_INPUT(PF03) | \
					 PIN_MODE_INPUT(PF04) | \
					 PIN_MODE_INPUT(PF05) | \
					 PIN_MODE_INPUT(PF06) | \
					 PIN_MODE_INPUT(PF07) | \
					 PIN_MODE_INPUT(PF08) | \
					 PIN_MODE_INPUT(PF09) | \
					 PIN_MODE_INPUT(PF10) | \
					 PIN_MODE_INPUT(PF11) | \
					 PIN_MODE_INPUT(PF12) | \
					 PIN_MODE_INPUT(PF13) | \
					 PIN_MODE_INPUT(PF14) | \
					 PIN_MODE_INPUT(PF15))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_PUSHPULL(PF00) | \
					 PIN_OTYPE_PUSHPULL(PF01) | \
					 PIN_OTYPE_PUSHPULL(PF02) | \
					 PIN_OTYPE_PUSHPULL(PF03) | \
					 PIN_OTYPE_PUSHPULL(PF04) | \
					 PIN_OTYPE_PUSHPULL(PF05) | \
					 PIN_OTYPE_PUSHPULL(PF06) | \
					 PIN_OTYPE_PUSHPULL(PF07) | \
					 PIN_OTYPE_PUSHPULL(PF08) | \
					 PIN_OTYPE_PUSHPULL(PF09) | \
					 PIN_OTYPE_PUSHPULL(PF10) | \
					 PIN_OTYPE_PUSHPULL(PF11) | \
					 PIN_OTYPE_PUSHPULL(PF12) | \
					 PIN_OTYPE_PUSHPULL(PF13) | \
					 PIN_OTYPE_PUSHPULL(PF14) | \
					 PIN_OTYPE_PUSHPULL(PF15))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PF00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF15))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLDOWN(PF00) | \
					 PIN_PUPDR_PULLDOWN(PF01) | \
					 PIN_PUPDR_PULLDOWN(PF02) | \
					 PIN_PUPDR_PULLDOWN(PF03) | \
					 PIN_PUPDR_PULLDOWN(PF04) | \
					 PIN_PUPDR_PULLDOWN(PF05) | \
					 PIN_PUPDR_PULLDOWN(PF06) | \
					 PIN_PUPDR_PULLDOWN(PF07) | \
					 PIN_PUPDR_PULLDOWN(PF08) | \
					 PIN_PUPDR_PULLDOWN(PF09) | \
					 PIN_PUPDR_PULLDOWN(PF10) | \
					 PIN_PUPDR_PULLDOWN(PF11) | \
					 PIN_PUPDR_PULLDOWN(PF12) | \
					 PIN_PUPDR_PULLDOWN(PF13) | \
					 PIN_PUPDR_PULLDOWN(PF14) | \
					 PIN_PUPDR_PULLDOWN(PF15))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_LOW(PF00) | \
					 PIN_ODR_LEVEL_LOW(PF01) | \
					 PIN_ODR_LEVEL_LOW(PF02) | \
					 PIN_ODR_LEVEL_LOW(PF03) | \
					 PIN_ODR_LEVEL_LOW(PF04) | \
					 PIN_ODR_LEVEL_LOW(PF05) | \
					 PIN_ODR_LEVEL_LOW(PF06) | \
					 PIN_ODR_LEVEL_LOW(PF07) | \
					 PIN_ODR_LEVEL_LOW(PF08) | \
					 PIN_ODR_LEVEL_LOW(PF09) | \
					 PIN_ODR_LEVEL_LOW(PF10) | \
					 PIN_ODR_LEVEL_LOW(PF11) | \
					 PIN_ODR_LEVEL_LOW(PF12) | \
					 PIN_ODR_LEVEL_LOW(PF13) | \
					 PIN_ODR_LEVEL_LOW(PF14) | \
					 PIN_ODR_LEVEL_LOW(PF15))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00, 0) | \
					 PIN_AFIO_AF(PF01, 0) | \
					 PIN_AFIO_AF(PF02, 0) | \
					 PIN_AFIO_AF(PF03, 0) | \
					 PIN_AFIO_AF(PF04, 0) | \
					 PIN_AFIO_AF(PF05, 0) | \
					 PIN_AFIO_AF(PF06, 0) | \
					 PIN_AFIO_AF(PF07, 0))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08, 0) | \
					 PIN_AFIO_AF(PF09, 0) | \
					 PIN_AFIO_AF(PF10, 0) | \
					 PIN_AFIO_AF(PF11, 0) | \
					 PIN_AFIO_AF(PF12, 0) | \
					 PIN_AFIO_AF(PF13, 0) | \
					 PIN_AFIO_AF(PF14, 0) | \
					 PIN_AFIO_AF(PF15, 0))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00) | \
					 PIN_MODE_INPUT(PG01) | \
					 PIN_MODE_INPUT(PG02) | \
					 PIN_MODE_INPUT(PG03) | \
					 PIN_MODE_INPUT(PG04) | \
					 PIN_MODE_INPUT(PG05) | \
					 PIN_MODE_INPUT(PG06) | \
					 PIN_MODE_INPUT(PG07) | \
					 PIN_MODE_INPUT(PG08) | \
					 PIN_MODE_INPUT(PG09) | \
					 PIN_MODE_INPUT(PG10) | \
					 PIN_MODE_INPUT(PG11) | \
					 PIN_MODE_INPUT(PG12) | \
					 PIN_MODE_INPUT(PG13) | \
					 PIN_MODE_INPUT(PG14) | \
					 PIN_MODE_INPUT(PG15))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(PG00) | \
					 PIN_OTYPE_PUSHPULL(PG01) | \
					 PIN_OTYPE_PUSHPULL(PG02) | \
					 PIN_OTYPE_PUSHPULL(PG03) | \
					 PIN_OTYPE_PUSHPULL(PG04) | \
					 PIN_OTYPE_PUSHPULL(PG05) | \
					 PIN_OTYPE_PUSHPULL(PG06) | \
					 PIN_OTYPE_PUSHPULL(PG07) | \
					 PIN_OTYPE_PUSHPULL(PG08) | \
					 PIN_OTYPE_PUSHPULL(PG09) | \
					 PIN_OTYPE_PUSHPULL(PG10) | \
					 PIN_OTYPE_PUSHPULL(PG11) | \
					 PIN_OTYPE_PUSHPULL(PG12) | \
					 PIN_OTYPE_PUSHPULL(PG13) | \
					 PIN_OTYPE_PUSHPULL(PG14) | \
					 PIN_OTYPE_PUSHPULL(PG15))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00) | \
					 PIN_PUPDR_PULLDOWN(PG01) | \
					 PIN_PUPDR_PULLDOWN(PG02) | \
					 PIN_PUPDR_PULLDOWN(PG03) | \
					 PIN_PUPDR_PULLDOWN(PG04) | \
					 PIN_PUPDR_PULLDOWN(PG05) | \
					 PIN_PUPDR_PULLDOWN(PG06) | \
					 PIN_PUPDR_PULLDOWN(PG07) | \
					 PIN_PUPDR_PULLDOWN(PG08) | \
					 PIN_PUPDR_PULLDOWN(PG09) | \
					 PIN_PUPDR_PULLDOWN(PG10) | \
					 PIN_PUPDR_PULLDOWN(PG11) | \
					 PIN_PUPDR_PULLDOWN(PG12) | \
					 PIN_PUPDR_PULLDOWN(PG13) | \
					 PIN_PUPDR_PULLDOWN(PG14) | \
					 PIN_PUPDR_PULLDOWN(PG15))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(PG00) | \
					 PIN_ODR_LEVEL_LOW(PG01) | \
					 PIN_ODR_LEVEL_LOW(PG02) | \
					 PIN_ODR_LEVEL_LOW(PG03) | \
					 PIN_ODR_LEVEL_LOW(PG04) | \
					 PIN_ODR_LEVEL_LOW(PG05) | \
					 PIN_ODR_LEVEL_LOW(PG06) | \
					 PIN_ODR_LEVEL_LOW(PG07) | \
					 PIN_ODR_LEVEL_LOW(PG08) | \
					 PIN_ODR_LEVEL_LOW(PG09) | \
					 PIN_ODR_LEVEL_LOW(PG10) | \
					 PIN_ODR_LEVEL_LOW(PG11) | \
					 PIN_ODR_LEVEL_LOW(PG12) | \
					 PIN_ODR_LEVEL_LOW(PG13) | \
					 PIN_ODR_LEVEL_LOW(PG14) | \
					 PIN_ODR_LEVEL_LOW(PG15))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00, 0) | \
					 PIN_AFIO_AF(PG01, 0) | \
					 PIN_AFIO_AF(PG02, 0) | \
					 PIN_AFIO_AF(PG03, 0) | \
					 PIN_AFIO_AF(PG04, 0) | \
					 PIN_AFIO_AF(PG05, 0) | \
					 PIN_AFIO_AF(PG06, 0) | \
					 PIN_AFIO_AF(PG07, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08, 0) | \
					 PIN_AFIO_AF(PG09, 0) | \
					 PIN_AFIO_AF(PG10, 0) | \
					 PIN_AFIO_AF(PG11, 0) | \
					 PIN_AFIO_AF(PG12, 0) | \
					 PIN_AFIO_AF(PG13, 0) | \
					 PIN_AFIO_AF(PG14, 0) | \
					 PIN_AFIO_AF(PG15, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(OSC_IN) | \
					 PIN_MODE_ALTERNATE(OSC_OUT) | \
					 PIN_MODE_INPUT(PH02) | \
					 PIN_MODE_INPUT(PH03) | \
					 PIN_MODE_INPUT(PH04) | \
					 PIN_MODE_INPUT(PH05) | \
					 PIN_MODE_INPUT(PH06) | \
					 PIN_MODE_INPUT(PH07) | \
					 PIN_MODE_INPUT(PH08) | \
					 PIN_MODE_INPUT(PH09) | \
					 PIN_MODE_INPUT(PH10) | \
					 PIN_MODE_INPUT(PH11) | \
					 PIN_MODE_INPUT(PH12) | \
					 PIN_MODE_INPUT(PH13) | \
					 PIN_MODE_INPUT(PH14) | \
					 PIN_MODE_INPUT(PH15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(PH02) | \
					 PIN_OTYPE_PUSHPULL(PH03) | \
					 PIN_OTYPE_PUSHPULL(PH04) | \
					 PIN_OTYPE_PUSHPULL(PH05) | \
					 PIN_OTYPE_PUSHPULL(PH06) | \
					 PIN_OTYPE_PUSHPULL(PH07) | \
					 PIN_OTYPE_PUSHPULL(PH08) | \
					 PIN_OTYPE_PUSHPULL(PH09) | \
					 PIN_OTYPE_PUSHPULL(PH10) | \
					 PIN_OTYPE_PUSHPULL(PH11) | \
					 PIN_OTYPE_PUSHPULL(PH12) | \
					 PIN_OTYPE_PUSHPULL(PH13) | \
					 PIN_OTYPE_PUSHPULL(PH14) | \
					 PIN_OTYPE_PUSHPULL(PH15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(OSC_IN) | \
					 PIN_PUPDR_FLOATING(OSC_OUT) | \
					 PIN_PUPDR_PULLDOWN(PH02) | \
					 PIN_PUPDR_PULLDOWN(PH03) | \
					 PIN_PUPDR_PULLDOWN(PH04) | \
					 PIN_PUPDR_PULLDOWN(PH05) | \
					 PIN_PUPDR_PULLDOWN(PH06) | \
					 PIN_PUPDR_PULLDOWN(PH07) | \
					 PIN_PUPDR_PULLDOWN(PH08) | \
					 PIN_PUPDR_PULLDOWN(PH09) | \
					 PIN_PUPDR_PULLDOWN(PH10) | \
					 PIN_PUPDR_PULLDOWN(PH11) | \
					 PIN_PUPDR_PULLDOWN(PH12) | \
					 PIN_PUPDR_PULLDOWN(PH13) | \
					 PIN_PUPDR_PULLDOWN(PH14) | \
					 PIN_PUPDR_PULLDOWN(PH15))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC_OUT) | \
					 PIN_ODR_LEVEL_LOW(PH02) | \
					 PIN_ODR_LEVEL_LOW(PH03) | \
					 PIN_ODR_LEVEL_LOW(PH04) | \
					 PIN_ODR_LEVEL_LOW(PH05) | \
					 PIN_ODR_LEVEL_LOW(PH06) | \
					 PIN_ODR_LEVEL_LOW(PH07) | \
					 PIN_ODR_LEVEL_LOW(PH08) | \
					 PIN_ODR_LEVEL_LOW(PH09) | \
					 PIN_ODR_LEVEL_LOW(PH10) | \
					 PIN_ODR_LEVEL_LOW(PH11) | \
					 PIN_ODR_LEVEL_LOW(PH12) | \
					 PIN_ODR_LEVEL_LOW(PH13) | \
					 PIN_ODR_LEVEL_LOW(PH14) | \
					 PIN_ODR_LEVEL_LOW(PH15))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(OSC_IN, 0) | \
					 PIN_AFIO_AF(OSC_OUT, 0) | \
					 PIN_AFIO_AF(PH02, 0) | \
					 PIN_AFIO_AF(PH03, 0) | \
					 PIN_AFIO_AF(PH04, 0) | \
					 PIN_AFIO_AF(PH05, 0) | \
					 PIN_AFIO_AF(PH06, 0) | \
					 PIN_AFIO_AF(PH07, 0))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08, 0) | \
					 PIN_AFIO_AF(PH09, 0) | \
					 PIN_AFIO_AF(PH10, 0) | \
					 PIN_AFIO_AF(PH11, 0) | \
					 PIN_AFIO_AF(PH12, 0) | \
					 PIN_AFIO_AF(PH13, 0) | \
					 PIN_AFIO_AF(PH14, 0) | \
					 PIN_AFIO_AF(PH15, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_INPUT(PI00) | \
					 PIN_MODE_INPUT(PI01) | \
					 PIN_MODE_INPUT(PI02) | \
					 PIN_MODE_INPUT(PI03) | \
					 PIN_MODE_INPUT(PI04) | \
					 PIN_MODE_INPUT(PI05) | \
					 PIN_MODE_INPUT(PI06) | \
					 PIN_MODE_INPUT(PI07) | \
					 PIN_MODE_INPUT(PI08) | \
					 PIN_MODE_INPUT(PI09) | \
					 PIN_MODE_INPUT(PI10) | \
					 PIN_MODE_INPUT(PI11) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00) | \
					 PIN_OTYPE_PUSHPULL(PI01) | \
					 PIN_OTYPE_PUSHPULL(PI02) | \
					 PIN_OTYPE_PUSHPULL(PI03) | \
					 PIN_OTYPE_PUSHPULL(PI04) | \
					 PIN_OTYPE_PUSHPULL(PI05) | \
					 PIN_OTYPE_PUSHPULL(PI06) | \
					 PIN_OTYPE_PUSHPULL(PI07) | \
					 PIN_OTYPE_PUSHPULL(PI08) | \
					 PIN_OTYPE_PUSHPULL(PI09) | \
					 PIN_OTYPE_PUSHPULL(PI10) | \
					 PIN_OTYPE_PUSHPULL(PI11) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PI00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_PULLDOWN(PI00) | \
					 PIN_PUPDR_PULLDOWN(PI01) | \
					 PIN_PUPDR_PULLDOWN(PI02) | \
					 PIN_PUPDR_PULLDOWN(PI03) | \
					 PIN_PUPDR_PULLDOWN(PI04) | \
					 PIN_PUPDR_PULLDOWN(PI05) | \
					 PIN_PUPDR_PULLDOWN(PI06) | \
					 PIN_PUPDR_PULLDOWN(PI07) | \
					 PIN_PUPDR_PULLDOWN(PI08) | \
					 PIN_PUPDR_PULLDOWN(PI09) | \
					 PIN_PUPDR_PULLDOWN(PI10) | \
					 PIN_PUPDR_PULLDOWN(PI11) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00) | \
					 PIN_ODR_LEVEL_LOW(PI01) | \
					 PIN_ODR_LEVEL_LOW(PI02) | \
					 PIN_ODR_LEVEL_LOW(PI03) | \
					 PIN_ODR_LEVEL_LOW(PI04) | \
					 PIN_ODR_LEVEL_LOW(PI05) | \
					 PIN_ODR_LEVEL_LOW(PI06) | \
					 PIN_ODR_LEVEL_LOW(PI07) | \
					 PIN_ODR_LEVEL_LOW(PI08) | \
					 PIN_ODR_LEVEL_LOW(PI09) | \
					 PIN_ODR_LEVEL_LOW(PI10) | \
					 PIN_ODR_LEVEL_LOW(PI11) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(PI00, 0) | \
					 PIN_AFIO_AF(PI01, 0) | \
					 PIN_AFIO_AF(PI02, 0) | \
					 PIN_AFIO_AF(PI03, 0) | \
					 PIN_AFIO_AF(PI04, 0) | \
					 PIN_AFIO_AF(PI05, 0) | \
					 PIN_AFIO_AF(PI06, 0) | \
					 PIN_AFIO_AF(PI07, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(PI08, 0) | \
					 PIN_AFIO_AF(PI09, 0) | \
					 PIN_AFIO_AF(PI10, 0) | \
					 PIN_AFIO_AF(PI11, 0) | \
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

#define AF_S1                            1U
#define AF_LINE_S1                       1U
#define AF_S2                            1U
#define AF_LINE_S2                       1U
#define AF_S3                            2U
#define AF_LINE_S3                       2U
#define AF_S4                            2U
#define AF_LINE_S4                       2U
#define AF_SPI1_SCK                      5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_SPI1_MISO                     5U
#define AF_LINE_SPI1_MISO                5U
#define AF_SPI1_MOSI                     5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_LED_WS2812                    1U
#define AF_LINE_LED_WS2812               1U
#define AF_UART1_TX                      7U
#define AF_LINE_UART1_TX                 7U
#define AF_UART1_RX                      7U
#define AF_LINE_UART1_RX                 7U
#define AF_OTG_FS_DM                     10U
#define AF_LINE_OTG_FS_DM                10U
#define AF_OTG_FS_DP                     10U
#define AF_LINE_OTG_FS_DP                10U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_S5                            3U
#define AF_LINE_S5                       3U
#define AF_S6                            3U
#define AF_LINE_S6                       3U
#define AF_SPI3_SCK                      6U
#define AF_LINE_SPI3_SCK                 6U
#define AF_SPI3_MISO                     6U
#define AF_LINE_SPI3_MISO                6U
#define AF_SPI3_MOSI                     6U
#define AF_LINE_SPI3_MOSI                6U
#define AF_I2C1_SCL                      4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_I2C1_SDA                      4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_UART5_RX                      7U
#define AF_LINE_UART5_RX                 7U
#define AF_ALARM                         3U
#define AF_LINE_ALARM                    3U
#define AF_I2C2_SCL                      4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_I2C2_SDA                      4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_SPI2_SCK                      5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_SPI2_MISO                     5U
#define AF_LINE_SPI2_MISO                5U
#define AF_SPI2_MOSI                     5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_SDMMC1_D0                     12U
#define AF_LINE_SDMMC1_D0                12U
#define AF_SDMMC1_D1                     12U
#define AF_LINE_SDMMC1_D1                12U
#define AF_SDMMC1_D2                     12U
#define AF_LINE_SDMMC1_D2                12U
#define AF_SDMMC1_D3                     12U
#define AF_LINE_SDMMC1_D3                12U
#define AF_SDMMC1_CK                     12U
#define AF_LINE_SDMMC1_CK                12U
#define AF_OSC32_IN                      0U
#define AF_LINE_OSC32_IN                 0U
#define AF_OSC32_OUT                     0U
#define AF_LINE_OSC32_OUT                0U
#define AF_UART4_RX                      8U
#define AF_LINE_UART4_RX                 8U
#define AF_UART4_TX                      8U
#define AF_LINE_UART4_TX                 8U
#define AF_SDMMC1_CMD                    12U
#define AF_LINE_SDMMC1_CMD               12U
#define AF_UART2_TX                      7U
#define AF_LINE_UART2_TX                 7U
#define AF_UART2_RX                      7U
#define AF_LINE_UART2_RX                 7U
#define AF_UART3_TX                      7U
#define AF_LINE_UART3_TX                 7U
#define AF_UART3_RX                      7U
#define AF_LINE_UART3_RX                 7U
#define AF_S7                            2U
#define AF_LINE_S7                       2U
#define AF_S8                            2U
#define AF_LINE_S8                       2U
#define AF_S9                            2U
#define AF_LINE_S9                       2U
#define AF_S10                           2U
#define AF_LINE_S10                      2U
#define AF_UART8_RX                      8U
#define AF_LINE_UART8_RX                 8U
#define AF_UART8_TX                      8U
#define AF_LINE_UART8_TX                 8U
#define AF_S11                           3U
#define AF_LINE_S11                      3U
#define AF_S12                           3U
#define AF_LINE_S12                      3U
#define AF_UART7_RX                      8U
#define AF_LINE_UART7_RX                 8U
#define AF_UART7_TX                      8U
#define AF_LINE_UART7_TX                 8U
#define AF_UART7_RTS                     8U
#define AF_LINE_UART7_RTS                8U
#define AF_UART7_CTS                     8U
#define AF_LINE_UART7_CTS                8U
#define AF_SPI4_CLK                      5U
#define AF_LINE_SPI4_CLK                 5U
#define AF_SPI4_MISO                     5U
#define AF_LINE_SPI4_MISO                5U
#define AF_SPI4_MOSI                     5U
#define AF_LINE_SPI4_MOSI                5U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U


#define S1_TIM	 2
#define S1_TIM_FN	 CH
#define S1_TIM_CH	 1
#define S1_TIM_AF	 1
#define S2_TIM	 2
#define S2_TIM_FN	 CH
#define S2_TIM_CH	 2
#define S2_TIM_AF	 1
#define S3_TIM	 5
#define S3_TIM_FN	 CH
#define S3_TIM_CH	 3
#define S3_TIM_AF	 2
#define S4_TIM	 5
#define S4_TIM_FN	 CH
#define S4_TIM_CH	 4
#define S4_TIM_AF	 2
#define LED_WS2812_TIM	 1
#define LED_WS2812_TIM_FN	 CH
#define LED_WS2812_TIM_CH	 1
#define LED_WS2812_TIM_AF	 1
#define S5_TIM	 8
#define S5_TIM_FN	 CH
#define S5_TIM_CH	 2
#define S5_TIM_AF	 3
#define S6_TIM	 8
#define S6_TIM_FN	 CH
#define S6_TIM_CH	 3
#define S6_TIM_AF	 3
#define ALARM_TIM	 11
#define ALARM_TIM_FN	 CH
#define ALARM_TIM_CH	 1
#define ALARM_TIM_AF	 3
#define AIRSPEED_ADC	 1
#define AIRSPEED_ADC_FN	 IN
#define AIRSPEED_ADC_IN	 10
#define RSSI_ADC	 1
#define RSSI_ADC_FN	 IN
#define RSSI_ADC_IN	 11
#define VBAT_MEAS_ADC	 1
#define VBAT_MEAS_ADC_FN	 IN
#define VBAT_MEAS_ADC_IN	 12
#define CURRENT_MEAS_ADC	 1
#define CURRENT_MEAS_ADC_FN	 IN
#define CURRENT_MEAS_ADC_IN	 13
#define RC2_TIM	 8
#define RC2_TIM_FN	 CH
#define RC2_TIM_CH	 1
#define RC2_TIM_AF	 3
#define RC2_USART	 6
#define RC2_USART_FN	 TX
#define RC2_USART_AF	 8
#define RC1_TIM	 3
#define RC1_TIM_FN	 CH
#define RC1_TIM_CH	 2
#define RC1_TIM_AF	 2
#define RC1_USART	 6
#define RC1_USART_FN	 RX
#define RC1_USART_AF	 8
#define UART4_RX_CAN	 1
#define UART4_RX_CAN_FN	 RX
#define UART4_RX_CAN_AF	 9
#define UART4_TX_CAN	 1
#define UART4_TX_CAN_FN	 TX
#define UART4_TX_CAN_AF	 9
#define S7_TIM	 4
#define S7_TIM_FN	 CH
#define S7_TIM_CH	 1
#define S7_TIM_AF	 2
#define S8_TIM	 4
#define S8_TIM_FN	 CH
#define S8_TIM_CH	 2
#define S8_TIM_AF	 2
#define S9_TIM	 4
#define S9_TIM_FN	 CH
#define S9_TIM_CH	 3
#define S9_TIM_AF	 2
#define S10_TIM	 4
#define S10_TIM_FN	 CH
#define S10_TIM_CH	 4
#define S10_TIM_AF	 2
#define S11_TIM	 9
#define S11_TIM_FN	 CH
#define S11_TIM_CH	 1
#define S11_TIM_AF	 3
#define S12_TIM	 9
#define S12_TIM_FN	 CH
#define S12_TIM_CH	 2
#define S12_TIM_AF	 3

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

