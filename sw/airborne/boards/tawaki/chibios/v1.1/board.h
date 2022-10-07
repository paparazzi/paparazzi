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
#define BOARD_TAWAKI
#define BOARD_NAME                  "Tawaki Autopilot"

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
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F777xx

/*
 * IO pins assignments.
 */
#define	AUX_A1                         0U
#define	AUX_A2                         1U
#define	AUX_A3                         2U
#define	AUX_A4                         3U
#define	PA04                           4U
#define	PA05                           5U
#define	AUX_B1                         6U
#define	AUX_B2                         7U
#define	PA08                           8U
#define	USB_VBUS                       9U
#define	LED2                           10U
#define	OTG_FS_DM                      11U
#define	OTG_FS_DP                      12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	UART7_TX                       15U

#define	AUX_B3                         0U
#define	AUX_B4                         1U
#define	PB02                           2U
#define	UART7_RX                       3U
#define	PB04                           4U
#define	DSHOT_RX                       5U
#define	SRVB1                          6U
#define	SRVB2                          7U
#define	SRVB3                          8U
#define	SRVB4                          9U
#define	I2C2_SCL                       10U
#define	I2C2_SDA                       11U
#define	SPI2_EXTERNAL_CS               12U
#define	PB13                           13U
#define	SPI2_EXTERNAL_MISO             14U
#define	SPI2_EXTERNAL_MOSI             15U

#define	VBAT_MEAS                      0U
#define	PC01                           1U
#define	PC02                           2U
#define	PC03                           3U
#define	PC04                           4U
#define	PC05                           5U
#define	RC2                            6U
#define	LED3                           7U
#define	SDMMC1_D0                      8U
#define	SDMMC1_D1                      9U
#define	SDMMC1_D2                      10U
#define	SDMMC1_D3                      11U
#define	SDMMC1_CK                      12U
#define	APSW                           13U
#define	OSC32_IN                       14U
#define	OSC32_OUT                      15U

#define	CAN_RX                         0U
#define	CAN_TX                         1U
#define	SDMMC1_CMD                     2U
#define	SPI2_EXTERNAL_CLK              3U
#define	PD04                           4U
#define	UART2_TX                       5U
#define	UART2_RX                       6U
#define	PD07                           7U
#define	UART3_TX                       8U
#define	UART3_RX                       9U
#define	LED4                           10U
#define	PD11                           11U
#define	I2C4_SCL                       12U
#define	I2C4_SDA                       13U
#define	PD14                           14U
#define	LED1                           15U

#define	RC1                            0U
#define	PE01                           1U
#define	SPI4_INTERNAL_CLK              2U
#define	PE03                           3U
#define	SPI4_INTERNAL_CS               4U
#define	SPI4_INTERNAL_MISO             5U
#define	SPI4_INTERNAL_MOSI             6U
#define	PE07                           7U
#define	PE08                           8U
#define	SRVA1                          9U
#define	PE10                           10U
#define	SRVA2                          11U
#define	PE12                           12U
#define	SRVA3                          13U
#define	SRVA4                          14U
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
#define	LINE_AUX_A1                    PAL_LINE(GPIOA, 0U)
#define	LINE_AUX_A2                    PAL_LINE(GPIOA, 1U)
#define	LINE_AUX_A3                    PAL_LINE(GPIOA, 2U)
#define	LINE_AUX_A4                    PAL_LINE(GPIOA, 3U)
#define	LINE_AUX_B1                    PAL_LINE(GPIOA, 6U)
#define	LINE_AUX_B2                    PAL_LINE(GPIOA, 7U)
#define	LINE_USB_VBUS                  PAL_LINE(GPIOA, 9U)
#define	LINE_LED2                      PAL_LINE(GPIOA, 10U)
#define	LINE_OTG_FS_DM                 PAL_LINE(GPIOA, 11U)
#define	LINE_OTG_FS_DP                 PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOA, 15U)

#define	LINE_AUX_B3                    PAL_LINE(GPIOB, 0U)
#define	LINE_AUX_B4                    PAL_LINE(GPIOB, 1U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOB, 3U)
#define	LINE_DSHOT_RX                  PAL_LINE(GPIOB, 5U)
#define	LINE_SRVB1                     PAL_LINE(GPIOB, 6U)
#define	LINE_SRVB2                     PAL_LINE(GPIOB, 7U)
#define	LINE_SRVB3                     PAL_LINE(GPIOB, 8U)
#define	LINE_SRVB4                     PAL_LINE(GPIOB, 9U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define	LINE_SPI2_EXTERNAL_CS          PAL_LINE(GPIOB, 12U)
#define	LINE_SPI2_EXTERNAL_MISO        PAL_LINE(GPIOB, 14U)
#define	LINE_SPI2_EXTERNAL_MOSI        PAL_LINE(GPIOB, 15U)

#define	LINE_VBAT_MEAS                 PAL_LINE(GPIOC, 0U)
#define	LINE_RC2                       PAL_LINE(GPIOC, 6U)
#define	LINE_LED3                      PAL_LINE(GPIOC, 7U)
#define	LINE_SDMMC1_D0                 PAL_LINE(GPIOC, 8U)
#define	LINE_SDMMC1_D1                 PAL_LINE(GPIOC, 9U)
#define	LINE_SDMMC1_D2                 PAL_LINE(GPIOC, 10U)
#define	LINE_SDMMC1_D3                 PAL_LINE(GPIOC, 11U)
#define	LINE_SDMMC1_CK                 PAL_LINE(GPIOC, 12U)
#define	LINE_APSW                      PAL_LINE(GPIOC, 13U)
#define	LINE_OSC32_IN                  PAL_LINE(GPIOC, 14U)
#define	LINE_OSC32_OUT                 PAL_LINE(GPIOC, 15U)

#define	LINE_CAN_RX                    PAL_LINE(GPIOD, 0U)
#define	LINE_CAN_TX                    PAL_LINE(GPIOD, 1U)
#define	LINE_SDMMC1_CMD                PAL_LINE(GPIOD, 2U)
#define	LINE_SPI2_EXTERNAL_CLK         PAL_LINE(GPIOD, 3U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOD, 5U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOD, 6U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOD, 8U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOD, 9U)
#define	LINE_LED4                      PAL_LINE(GPIOD, 10U)
#define	LINE_I2C4_SCL                  PAL_LINE(GPIOD, 12U)
#define	LINE_I2C4_SDA                  PAL_LINE(GPIOD, 13U)
#define	LINE_LED1                      PAL_LINE(GPIOD, 15U)

#define	LINE_RC1                       PAL_LINE(GPIOE, 0U)
#define	LINE_SPI4_INTERNAL_CLK         PAL_LINE(GPIOE, 2U)
#define	LINE_SPI4_INTERNAL_CS          PAL_LINE(GPIOE, 4U)
#define	LINE_SPI4_INTERNAL_MISO        PAL_LINE(GPIOE, 5U)
#define	LINE_SPI4_INTERNAL_MOSI        PAL_LINE(GPIOE, 6U)
#define	LINE_SRVA1                     PAL_LINE(GPIOE, 9U)
#define	LINE_SRVA2                     PAL_LINE(GPIOE, 11U)
#define	LINE_SRVA3                     PAL_LINE(GPIOE, 13U)
#define	LINE_SRVA4                     PAL_LINE(GPIOE, 14U)

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

#define VAL_GPIOA_MODER                 (PIN_MODE_INPUT(AUX_A1) | \
					 PIN_MODE_INPUT(AUX_A2) | \
					 PIN_MODE_INPUT(AUX_A3) | \
					 PIN_MODE_INPUT(AUX_A4) | \
					 PIN_MODE_INPUT(PA04) | \
					 PIN_MODE_INPUT(PA05) | \
					 PIN_MODE_INPUT(AUX_B1) | \
					 PIN_MODE_INPUT(AUX_B2) | \
					 PIN_MODE_INPUT(PA08) | \
					 PIN_MODE_INPUT(USB_VBUS) | \
					 PIN_MODE_OUTPUT(LED2) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DM) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_ALTERNATE(UART7_TX))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_OPENDRAIN(AUX_A1) | \
					 PIN_OTYPE_OPENDRAIN(AUX_A2) | \
					 PIN_OTYPE_OPENDRAIN(AUX_A3) | \
					 PIN_OTYPE_OPENDRAIN(AUX_A4) | \
					 PIN_OTYPE_PUSHPULL(PA04) | \
					 PIN_OTYPE_PUSHPULL(PA05) | \
					 PIN_OTYPE_OPENDRAIN(AUX_B1) | \
					 PIN_OTYPE_OPENDRAIN(AUX_B2) | \
					 PIN_OTYPE_PUSHPULL(PA08) | \
					 PIN_OTYPE_OPENDRAIN(USB_VBUS) | \
					 PIN_OTYPE_PUSHPULL(LED2) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DM) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_PUSHPULL(UART7_TX))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(AUX_A1) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_A2) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_A3) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_A4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA05) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_B1) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_B2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA08) | \
					 PIN_OSPEED_SPEED_VERYLOW(USB_VBUS) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED2) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DM) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_TX))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_PULLDOWN(AUX_A1) | \
					 PIN_PUPDR_PULLDOWN(AUX_A2) | \
					 PIN_PUPDR_PULLDOWN(AUX_A3) | \
					 PIN_PUPDR_PULLDOWN(AUX_A4) | \
					 PIN_PUPDR_PULLDOWN(PA04) | \
					 PIN_PUPDR_PULLDOWN(PA05) | \
					 PIN_PUPDR_PULLDOWN(AUX_B1) | \
					 PIN_PUPDR_PULLDOWN(AUX_B2) | \
					 PIN_PUPDR_PULLDOWN(PA08) | \
					 PIN_PUPDR_PULLDOWN(USB_VBUS) | \
					 PIN_PUPDR_FLOATING(LED2) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DM) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_FLOATING(UART7_TX))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(AUX_A1) | \
					 PIN_ODR_LEVEL_HIGH(AUX_A2) | \
					 PIN_ODR_LEVEL_HIGH(AUX_A3) | \
					 PIN_ODR_LEVEL_HIGH(AUX_A4) | \
					 PIN_ODR_LEVEL_LOW(PA04) | \
					 PIN_ODR_LEVEL_LOW(PA05) | \
					 PIN_ODR_LEVEL_HIGH(AUX_B1) | \
					 PIN_ODR_LEVEL_HIGH(AUX_B2) | \
					 PIN_ODR_LEVEL_LOW(PA08) | \
					 PIN_ODR_LEVEL_LOW(USB_VBUS) | \
					 PIN_ODR_LEVEL_LOW(LED2) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DM) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(UART7_TX))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(AUX_A1, 0) | \
					 PIN_AFIO_AF(AUX_A2, 0) | \
					 PIN_AFIO_AF(AUX_A3, 0) | \
					 PIN_AFIO_AF(AUX_A4, 0) | \
					 PIN_AFIO_AF(PA04, 0) | \
					 PIN_AFIO_AF(PA05, 0) | \
					 PIN_AFIO_AF(AUX_B1, 0) | \
					 PIN_AFIO_AF(AUX_B2, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08, 0) | \
					 PIN_AFIO_AF(USB_VBUS, 0) | \
					 PIN_AFIO_AF(LED2, 0) | \
					 PIN_AFIO_AF(OTG_FS_DM, 10) | \
					 PIN_AFIO_AF(OTG_FS_DP, 10) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(UART7_TX, 12))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(AUX_B3) | \
					 PIN_MODE_INPUT(AUX_B4) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_ALTERNATE(UART7_RX) | \
					 PIN_MODE_INPUT(PB04) | \
					 PIN_MODE_ALTERNATE(DSHOT_RX) | \
					 PIN_MODE_ALTERNATE(SRVB1) | \
					 PIN_MODE_ALTERNATE(SRVB2) | \
					 PIN_MODE_ALTERNATE(SRVB3) | \
					 PIN_MODE_ALTERNATE(SRVB4) | \
					 PIN_MODE_ALTERNATE(I2C2_SCL) | \
					 PIN_MODE_ALTERNATE(I2C2_SDA) | \
					 PIN_MODE_OUTPUT(SPI2_EXTERNAL_CS) | \
					 PIN_MODE_INPUT(PB13) | \
					 PIN_MODE_ALTERNATE(SPI2_EXTERNAL_MISO) | \
					 PIN_MODE_ALTERNATE(SPI2_EXTERNAL_MOSI))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_OPENDRAIN(AUX_B3) | \
					 PIN_OTYPE_OPENDRAIN(AUX_B4) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_PUSHPULL(UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(PB04) | \
					 PIN_OTYPE_PUSHPULL(DSHOT_RX) | \
					 PIN_OTYPE_PUSHPULL(SRVB1) | \
					 PIN_OTYPE_PUSHPULL(SRVB2) | \
					 PIN_OTYPE_PUSHPULL(SRVB3) | \
					 PIN_OTYPE_PUSHPULL(SRVB4) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C2_SDA) | \
					 PIN_OTYPE_PUSHPULL(SPI2_EXTERNAL_CS) | \
					 PIN_OTYPE_PUSHPULL(PB13) | \
					 PIN_OTYPE_PUSHPULL(SPI2_EXTERNAL_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI2_EXTERNAL_MOSI))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(AUX_B3) | \
					 PIN_OSPEED_SPEED_VERYLOW(AUX_B4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB04) | \
					 PIN_OSPEED_SPEED_HIGH(DSHOT_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SRVB1) | \
					 PIN_OSPEED_SPEED_HIGH(SRVB2) | \
					 PIN_OSPEED_SPEED_HIGH(SRVB3) | \
					 PIN_OSPEED_SPEED_HIGH(SRVB4) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_EXTERNAL_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB13) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_EXTERNAL_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_EXTERNAL_MOSI))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLDOWN(AUX_B3) | \
					 PIN_PUPDR_PULLDOWN(AUX_B4) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_FLOATING(UART7_RX) | \
					 PIN_PUPDR_PULLDOWN(PB04) | \
					 PIN_PUPDR_FLOATING(DSHOT_RX) | \
					 PIN_PUPDR_FLOATING(SRVB1) | \
					 PIN_PUPDR_FLOATING(SRVB2) | \
					 PIN_PUPDR_FLOATING(SRVB3) | \
					 PIN_PUPDR_FLOATING(SRVB4) | \
					 PIN_PUPDR_PULLUP(I2C2_SCL) | \
					 PIN_PUPDR_PULLUP(I2C2_SDA) | \
					 PIN_PUPDR_FLOATING(SPI2_EXTERNAL_CS) | \
					 PIN_PUPDR_PULLDOWN(PB13) | \
					 PIN_PUPDR_FLOATING(SPI2_EXTERNAL_MISO) | \
					 PIN_PUPDR_FLOATING(SPI2_EXTERNAL_MOSI))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_HIGH(AUX_B3) | \
					 PIN_ODR_LEVEL_HIGH(AUX_B4) | \
					 PIN_ODR_LEVEL_LOW(PB02) | \
					 PIN_ODR_LEVEL_HIGH(UART7_RX) | \
					 PIN_ODR_LEVEL_LOW(PB04) | \
					 PIN_ODR_LEVEL_HIGH(DSHOT_RX) | \
					 PIN_ODR_LEVEL_LOW(SRVB1) | \
					 PIN_ODR_LEVEL_LOW(SRVB2) | \
					 PIN_ODR_LEVEL_LOW(SRVB3) | \
					 PIN_ODR_LEVEL_LOW(SRVB4) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_EXTERNAL_CS) | \
					 PIN_ODR_LEVEL_LOW(PB13) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_EXTERNAL_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_EXTERNAL_MOSI))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(AUX_B3, 0) | \
					 PIN_AFIO_AF(AUX_B4, 0) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(UART7_RX, 12) | \
					 PIN_AFIO_AF(PB04, 0) | \
					 PIN_AFIO_AF(DSHOT_RX, 1) | \
					 PIN_AFIO_AF(SRVB1, 2) | \
					 PIN_AFIO_AF(SRVB2, 2))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(SRVB3, 2) | \
					 PIN_AFIO_AF(SRVB4, 2) | \
					 PIN_AFIO_AF(I2C2_SCL, 4) | \
					 PIN_AFIO_AF(I2C2_SDA, 4) | \
					 PIN_AFIO_AF(SPI2_EXTERNAL_CS, 0) | \
					 PIN_AFIO_AF(PB13, 0) | \
					 PIN_AFIO_AF(SPI2_EXTERNAL_MISO, 5) | \
					 PIN_AFIO_AF(SPI2_EXTERNAL_MOSI, 5))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(VBAT_MEAS) | \
					 PIN_MODE_INPUT(PC01) | \
					 PIN_MODE_INPUT(PC02) | \
					 PIN_MODE_INPUT(PC03) | \
					 PIN_MODE_INPUT(PC04) | \
					 PIN_MODE_INPUT(PC05) | \
					 PIN_MODE_INPUT(RC2) | \
					 PIN_MODE_OUTPUT(LED3) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D0) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D1) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D2) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D3) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CK) | \
					 PIN_MODE_OUTPUT(APSW) | \
					 PIN_MODE_ALTERNATE(OSC32_IN) | \
					 PIN_MODE_ALTERNATE(OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(VBAT_MEAS) | \
					 PIN_OTYPE_PUSHPULL(PC01) | \
					 PIN_OTYPE_PUSHPULL(PC02) | \
					 PIN_OTYPE_PUSHPULL(PC03) | \
					 PIN_OTYPE_PUSHPULL(PC04) | \
					 PIN_OTYPE_PUSHPULL(PC05) | \
					 PIN_OTYPE_OPENDRAIN(RC2) | \
					 PIN_OTYPE_PUSHPULL(LED3) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D0) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D1) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D2) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D3) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CK) | \
					 PIN_OTYPE_PUSHPULL(APSW) | \
					 PIN_OTYPE_PUSHPULL(OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(VBAT_MEAS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_VERYLOW(RC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED3) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(APSW) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(VBAT_MEAS) | \
					 PIN_PUPDR_PULLDOWN(PC01) | \
					 PIN_PUPDR_PULLDOWN(PC02) | \
					 PIN_PUPDR_PULLDOWN(PC03) | \
					 PIN_PUPDR_PULLDOWN(PC04) | \
					 PIN_PUPDR_PULLDOWN(PC05) | \
					 PIN_PUPDR_PULLDOWN(RC2) | \
					 PIN_PUPDR_FLOATING(LED3) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D0) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D1) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D2) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D3) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CK) | \
					 PIN_PUPDR_FLOATING(APSW) | \
					 PIN_PUPDR_FLOATING(OSC32_IN) | \
					 PIN_PUPDR_FLOATING(OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(VBAT_MEAS) | \
					 PIN_ODR_LEVEL_LOW(PC01) | \
					 PIN_ODR_LEVEL_LOW(PC02) | \
					 PIN_ODR_LEVEL_LOW(PC03) | \
					 PIN_ODR_LEVEL_LOW(PC04) | \
					 PIN_ODR_LEVEL_LOW(PC05) | \
					 PIN_ODR_LEVEL_HIGH(RC2) | \
					 PIN_ODR_LEVEL_LOW(LED3) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CK) | \
					 PIN_ODR_LEVEL_HIGH(APSW) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(VBAT_MEAS, 0) | \
					 PIN_AFIO_AF(PC01, 0) | \
					 PIN_AFIO_AF(PC02, 0) | \
					 PIN_AFIO_AF(PC03, 0) | \
					 PIN_AFIO_AF(PC04, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(RC2, 0) | \
					 PIN_AFIO_AF(LED3, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDMMC1_D0, 12) | \
					 PIN_AFIO_AF(SDMMC1_D1, 12) | \
					 PIN_AFIO_AF(SDMMC1_D2, 12) | \
					 PIN_AFIO_AF(SDMMC1_D3, 12) | \
					 PIN_AFIO_AF(SDMMC1_CK, 12) | \
					 PIN_AFIO_AF(APSW, 0) | \
					 PIN_AFIO_AF(OSC32_IN, 0) | \
					 PIN_AFIO_AF(OSC32_OUT, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(CAN_RX) | \
					 PIN_MODE_ALTERNATE(CAN_TX) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CMD) | \
					 PIN_MODE_ALTERNATE(SPI2_EXTERNAL_CLK) | \
					 PIN_MODE_INPUT(PD04) | \
					 PIN_MODE_ALTERNATE(UART2_TX) | \
					 PIN_MODE_ALTERNATE(UART2_RX) | \
					 PIN_MODE_INPUT(PD07) | \
					 PIN_MODE_ALTERNATE(UART3_TX) | \
					 PIN_MODE_ALTERNATE(UART3_RX) | \
					 PIN_MODE_OUTPUT(LED4) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_ALTERNATE(I2C4_SCL) | \
					 PIN_MODE_ALTERNATE(I2C4_SDA) | \
					 PIN_MODE_INPUT(PD14) | \
					 PIN_MODE_OUTPUT(LED1))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(CAN_RX) | \
					 PIN_OTYPE_PUSHPULL(CAN_TX) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CMD) | \
					 PIN_OTYPE_PUSHPULL(SPI2_EXTERNAL_CLK) | \
					 PIN_OTYPE_PUSHPULL(PD04) | \
					 PIN_OTYPE_PUSHPULL(UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(PD07) | \
					 PIN_OTYPE_PUSHPULL(UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(UART3_RX) | \
					 PIN_OTYPE_PUSHPULL(LED4) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_OPENDRAIN(I2C4_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C4_SDA) | \
					 PIN_OTYPE_PUSHPULL(PD14) | \
					 PIN_OTYPE_PUSHPULL(LED1))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(CAN_RX) | \
					 PIN_OSPEED_SPEED_HIGH(CAN_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CMD) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_EXTERNAL_CLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART2_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD07) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_HIGH(I2C4_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C4_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD14) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED1))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(CAN_RX) | \
					 PIN_PUPDR_FLOATING(CAN_TX) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CMD) | \
					 PIN_PUPDR_FLOATING(SPI2_EXTERNAL_CLK) | \
					 PIN_PUPDR_PULLDOWN(PD04) | \
					 PIN_PUPDR_FLOATING(UART2_TX) | \
					 PIN_PUPDR_FLOATING(UART2_RX) | \
					 PIN_PUPDR_PULLDOWN(PD07) | \
					 PIN_PUPDR_FLOATING(UART3_TX) | \
					 PIN_PUPDR_FLOATING(UART3_RX) | \
					 PIN_PUPDR_FLOATING(LED4) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_PULLUP(I2C4_SCL) | \
					 PIN_PUPDR_PULLUP(I2C4_SDA) | \
					 PIN_PUPDR_PULLDOWN(PD14) | \
					 PIN_PUPDR_FLOATING(LED1))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(CAN_RX) | \
					 PIN_ODR_LEVEL_HIGH(CAN_TX) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CMD) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_EXTERNAL_CLK) | \
					 PIN_ODR_LEVEL_LOW(PD04) | \
					 PIN_ODR_LEVEL_HIGH(UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART2_RX) | \
					 PIN_ODR_LEVEL_LOW(PD07) | \
					 PIN_ODR_LEVEL_HIGH(UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART3_RX) | \
					 PIN_ODR_LEVEL_LOW(LED4) | \
					 PIN_ODR_LEVEL_LOW(PD11) | \
					 PIN_ODR_LEVEL_HIGH(I2C4_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C4_SDA) | \
					 PIN_ODR_LEVEL_LOW(PD14) | \
					 PIN_ODR_LEVEL_LOW(LED1))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(CAN_RX, 9) | \
					 PIN_AFIO_AF(CAN_TX, 9) | \
					 PIN_AFIO_AF(SDMMC1_CMD, 12) | \
					 PIN_AFIO_AF(SPI2_EXTERNAL_CLK, 5) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(UART2_TX, 7) | \
					 PIN_AFIO_AF(UART2_RX, 7) | \
					 PIN_AFIO_AF(PD07, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(UART3_TX, 7) | \
					 PIN_AFIO_AF(UART3_RX, 7) | \
					 PIN_AFIO_AF(LED4, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(I2C4_SCL, 4) | \
					 PIN_AFIO_AF(I2C4_SDA, 4) | \
					 PIN_AFIO_AF(PD14, 0) | \
					 PIN_AFIO_AF(LED1, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(RC1) | \
					 PIN_MODE_INPUT(PE01) | \
					 PIN_MODE_ALTERNATE(SPI4_INTERNAL_CLK) | \
					 PIN_MODE_INPUT(PE03) | \
					 PIN_MODE_OUTPUT(SPI4_INTERNAL_CS) | \
					 PIN_MODE_ALTERNATE(SPI4_INTERNAL_MISO) | \
					 PIN_MODE_ALTERNATE(SPI4_INTERNAL_MOSI) | \
					 PIN_MODE_INPUT(PE07) | \
					 PIN_MODE_INPUT(PE08) | \
					 PIN_MODE_ALTERNATE(SRVA1) | \
					 PIN_MODE_INPUT(PE10) | \
					 PIN_MODE_ALTERNATE(SRVA2) | \
					 PIN_MODE_INPUT(PE12) | \
					 PIN_MODE_ALTERNATE(SRVA3) | \
					 PIN_MODE_ALTERNATE(SRVA4) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(RC1) | \
					 PIN_OTYPE_PUSHPULL(PE01) | \
					 PIN_OTYPE_PUSHPULL(SPI4_INTERNAL_CLK) | \
					 PIN_OTYPE_PUSHPULL(PE03) | \
					 PIN_OTYPE_PUSHPULL(SPI4_INTERNAL_CS) | \
					 PIN_OTYPE_PUSHPULL(SPI4_INTERNAL_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI4_INTERNAL_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PE07) | \
					 PIN_OTYPE_PUSHPULL(PE08) | \
					 PIN_OTYPE_PUSHPULL(SRVA1) | \
					 PIN_OTYPE_PUSHPULL(PE10) | \
					 PIN_OTYPE_PUSHPULL(SRVA2) | \
					 PIN_OTYPE_PUSHPULL(PE12) | \
					 PIN_OTYPE_PUSHPULL(SRVA3) | \
					 PIN_OTYPE_PUSHPULL(SRVA4) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(RC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE01) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_INTERNAL_CLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_INTERNAL_CS) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_INTERNAL_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI4_INTERNAL_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE08) | \
					 PIN_OSPEED_SPEED_HIGH(SRVA1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10) | \
					 PIN_OSPEED_SPEED_HIGH(SRVA2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12) | \
					 PIN_OSPEED_SPEED_HIGH(SRVA3) | \
					 PIN_OSPEED_SPEED_HIGH(SRVA4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(RC1) | \
					 PIN_PUPDR_PULLDOWN(PE01) | \
					 PIN_PUPDR_FLOATING(SPI4_INTERNAL_CLK) | \
					 PIN_PUPDR_PULLDOWN(PE03) | \
					 PIN_PUPDR_FLOATING(SPI4_INTERNAL_CS) | \
					 PIN_PUPDR_FLOATING(SPI4_INTERNAL_MISO) | \
					 PIN_PUPDR_FLOATING(SPI4_INTERNAL_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PE07) | \
					 PIN_PUPDR_PULLDOWN(PE08) | \
					 PIN_PUPDR_FLOATING(SRVA1) | \
					 PIN_PUPDR_PULLDOWN(PE10) | \
					 PIN_PUPDR_FLOATING(SRVA2) | \
					 PIN_PUPDR_PULLDOWN(PE12) | \
					 PIN_PUPDR_FLOATING(SRVA3) | \
					 PIN_PUPDR_FLOATING(SRVA4) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(RC1) | \
					 PIN_ODR_LEVEL_LOW(PE01) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_INTERNAL_CLK) | \
					 PIN_ODR_LEVEL_LOW(PE03) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_INTERNAL_CS) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_INTERNAL_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI4_INTERNAL_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PE07) | \
					 PIN_ODR_LEVEL_LOW(PE08) | \
					 PIN_ODR_LEVEL_LOW(SRVA1) | \
					 PIN_ODR_LEVEL_LOW(PE10) | \
					 PIN_ODR_LEVEL_LOW(SRVA2) | \
					 PIN_ODR_LEVEL_LOW(PE12) | \
					 PIN_ODR_LEVEL_LOW(SRVA3) | \
					 PIN_ODR_LEVEL_LOW(SRVA4) | \
					 PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(RC1, 8) | \
					 PIN_AFIO_AF(PE01, 0) | \
					 PIN_AFIO_AF(SPI4_INTERNAL_CLK, 5) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(SPI4_INTERNAL_CS, 0) | \
					 PIN_AFIO_AF(SPI4_INTERNAL_MISO, 5) | \
					 PIN_AFIO_AF(SPI4_INTERNAL_MOSI, 5) | \
					 PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08, 0) | \
					 PIN_AFIO_AF(SRVA1, 1) | \
					 PIN_AFIO_AF(PE10, 0) | \
					 PIN_AFIO_AF(SRVA2, 1) | \
					 PIN_AFIO_AF(PE12, 0) | \
					 PIN_AFIO_AF(SRVA3, 1) | \
					 PIN_AFIO_AF(SRVA4, 1) | \
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

#define AF_OTG_FS_DM                     10U
#define AF_LINE_OTG_FS_DM                10U
#define AF_OTG_FS_DP                     10U
#define AF_LINE_OTG_FS_DP                10U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_UART7_TX                      12U
#define AF_LINE_UART7_TX                 12U
#define AF_UART7_RX                      12U
#define AF_LINE_UART7_RX                 12U
#define AF_DSHOT_RX                      1U
#define AF_LINE_DSHOT_RX                 1U
#define AF_SRVB1                         2U
#define AF_LINE_SRVB1                    2U
#define AF_SRVB2                         2U
#define AF_LINE_SRVB2                    2U
#define AF_SRVB3                         2U
#define AF_LINE_SRVB3                    2U
#define AF_SRVB4                         2U
#define AF_LINE_SRVB4                    2U
#define AF_I2C2_SCL                      4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_I2C2_SDA                      4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_SPI2_EXTERNAL_MISO            5U
#define AF_LINE_SPI2_EXTERNAL_MISO       5U
#define AF_SPI2_EXTERNAL_MOSI            5U
#define AF_LINE_SPI2_EXTERNAL_MOSI       5U
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
#define AF_CAN_RX                        9U
#define AF_LINE_CAN_RX                   9U
#define AF_CAN_TX                        9U
#define AF_LINE_CAN_TX                   9U
#define AF_SDMMC1_CMD                    12U
#define AF_LINE_SDMMC1_CMD               12U
#define AF_SPI2_EXTERNAL_CLK             5U
#define AF_LINE_SPI2_EXTERNAL_CLK        5U
#define AF_UART2_TX                      7U
#define AF_LINE_UART2_TX                 7U
#define AF_UART2_RX                      7U
#define AF_LINE_UART2_RX                 7U
#define AF_UART3_TX                      7U
#define AF_LINE_UART3_TX                 7U
#define AF_UART3_RX                      7U
#define AF_LINE_UART3_RX                 7U
#define AF_I2C4_SCL                      4U
#define AF_LINE_I2C4_SCL                 4U
#define AF_I2C4_SDA                      4U
#define AF_LINE_I2C4_SDA                 4U
#define AF_RC1                           8U
#define AF_LINE_RC1                      8U
#define AF_SPI4_INTERNAL_CLK             5U
#define AF_LINE_SPI4_INTERNAL_CLK        5U
#define AF_SPI4_INTERNAL_MISO            5U
#define AF_LINE_SPI4_INTERNAL_MISO       5U
#define AF_SPI4_INTERNAL_MOSI            5U
#define AF_LINE_SPI4_INTERNAL_MOSI       5U
#define AF_SRVA1                         1U
#define AF_LINE_SRVA1                    1U
#define AF_SRVA2                         1U
#define AF_LINE_SRVA2                    1U
#define AF_SRVA3                         1U
#define AF_LINE_SRVA3                    1U
#define AF_SRVA4                         1U
#define AF_LINE_SRVA4                    1U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U


#define AUX_A1_ADC	 1
#define AUX_A1_ADC_FN	 IN
#define AUX_A1_ADC_IN	 0
#define AUX_A1_TIM	 5
#define AUX_A1_TIM_FN	 CH
#define AUX_A1_TIM_CH	 1
#define AUX_A1_TIM_AF	 2
#define AUX_A1_UART	 4
#define AUX_A1_UART_FN	 TX
#define AUX_A1_UART_AF	 8
#define AUX_A1_USART	 2
#define AUX_A1_USART_FN	 CTS
#define AUX_A1_USART_AF	 7
#define AUX_A2_ADC	 1
#define AUX_A2_ADC_FN	 IN
#define AUX_A2_ADC_IN	 1
#define AUX_A2_TIM	 5
#define AUX_A2_TIM_FN	 CH
#define AUX_A2_TIM_CH	 2
#define AUX_A2_TIM_AF	 2
#define AUX_A2_UART	 4
#define AUX_A2_UART_FN	 RX
#define AUX_A2_UART_AF	 8
#define AUX_A2_USART	 2
#define AUX_A2_USART_FN	 RTS
#define AUX_A2_USART_AF	 7
#define AUX_A3_ADC	 1
#define AUX_A3_ADC_FN	 IN
#define AUX_A3_ADC_IN	 2
#define AUX_A3_TIM	 5
#define AUX_A3_TIM_FN	 CH
#define AUX_A3_TIM_CH	 3
#define AUX_A3_TIM_AF	 2
#define AUX_A4_ADC	 1
#define AUX_A4_ADC_FN	 IN
#define AUX_A4_ADC_IN	 3
#define AUX_A4_TIM	 5
#define AUX_A4_TIM_FN	 CH
#define AUX_A4_TIM_CH	 4
#define AUX_A4_TIM_AF	 2
#define AUX_B1_ADC	 1
#define AUX_B1_ADC_FN	 IN
#define AUX_B1_ADC_IN	 6
#define AUX_B1_TIM	 3
#define AUX_B1_TIM_FN	 CH
#define AUX_B1_TIM_CH	 1
#define AUX_B1_TIM_AF	 2
#define AUX_B2_ADC	 1
#define AUX_B2_ADC_FN	 IN
#define AUX_B2_ADC_IN	 7
#define AUX_B2_TIM	 3
#define AUX_B2_TIM_FN	 CH
#define AUX_B2_TIM_CH	 2
#define AUX_B2_TIM_AF	 2
#define AUX_B3_ADC	 1
#define AUX_B3_ADC_FN	 IN
#define AUX_B3_ADC_IN	 8
#define AUX_B3_TIM	 3
#define AUX_B3_TIM_FN	 CH
#define AUX_B3_TIM_CH	 3
#define AUX_B3_TIM_AF	 2
#define AUX_B4_ADC	 1
#define AUX_B4_ADC_FN	 IN
#define AUX_B4_ADC_IN	 9
#define AUX_B4_TIM	 3
#define AUX_B4_TIM_FN	 CH
#define AUX_B4_TIM_CH	 4
#define AUX_B4_TIM_AF	 2
#define SRVB1_TIM	 4
#define SRVB1_TIM_FN	 CH
#define SRVB1_TIM_CH	 1
#define SRVB1_TIM_AF	 2
#define SRVB2_TIM	 4
#define SRVB2_TIM_FN	 CH
#define SRVB2_TIM_CH	 2
#define SRVB2_TIM_AF	 2
#define SRVB3_TIM	 4
#define SRVB3_TIM_FN	 CH
#define SRVB3_TIM_CH	 3
#define SRVB3_TIM_AF	 2
#define SRVB4_TIM	 4
#define SRVB4_TIM_FN	 CH
#define SRVB4_TIM_CH	 4
#define SRVB4_TIM_AF	 2
#define VBAT_MEAS_ADC	 1
#define VBAT_MEAS_ADC_FN	 IN
#define VBAT_MEAS_ADC_IN	 10
#define RC2_TIM	 8
#define RC2_TIM_FN	 CH
#define RC2_TIM_CH	 1
#define RC2_TIM_AF	 3
#define RC2_USART	 6
#define RC2_USART_FN	 TX
#define RC2_USART_AF	 8
#define RC1_UART	 8
#define RC1_UART_FN	 RX
#define RC1_UART_AF	 8
#define SRVA1_TIM	 1
#define SRVA1_TIM_FN	 CH
#define SRVA1_TIM_CH	 1
#define SRVA1_TIM_AF	 1
#define SRVA2_TIM	 1
#define SRVA2_TIM_FN	 CH
#define SRVA2_TIM_CH	 2
#define SRVA2_TIM_AF	 1
#define SRVA3_TIM	 1
#define SRVA3_TIM_FN	 CH
#define SRVA3_TIM_CH	 3
#define SRVA3_TIM_AF	 1
#define SRVA4_TIM	 1
#define SRVA4_TIM_FN	 CH
#define SRVA4_TIM_CH	 4
#define SRVA4_TIM_AF	 1

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

#define ENERGY_SAVE_INPUT \
	LINE_AUX_A1, \
	LINE_AUX_A2, \
	LINE_AUX_A3, \
	LINE_AUX_A4, \
	LINE_AUX_B1, \
	LINE_AUX_B2, \
	LINE_LED2, \
	LINE_AUX_B3, \
	LINE_AUX_B4, \
	LINE_SPI2_EXTERNAL_CS, \
	LINE_LED3, \
	LINE_LED4, \
	LINE_LED1, \
	LINE_SPI4_INTERNAL_CS
#define ENERGY_SAVE_INPUT_SIZE 	 14

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

