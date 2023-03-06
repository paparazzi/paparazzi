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
#define BOARD_MATEKH743_SLIM
#define BOARD_NAME                  "MATEKSYS FC H743 SLIM"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32H743xx

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_5
#endif

/* Default powerbrick values */
#define DefaultVoltageOfAdc(adc) ((3.3f/65536.0f) * 13.38f * adc)
#define DefaultMilliAmpereOfAdc(adc) ((3.3f/65536.0f) * 39.877f * adc)

/* Battery monitoring for file closing */
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_5_CHANNEL

/*
 * IO pins assignments.
 */
#define	PA00_SERVO3                    0U
#define	PA01_SERVO4                    1U
#define	PA02_SERVO5                    2U
#define	PA03_SERVO6                    3U
#define	PA04_ADC3                      4U
#define	PA05_SPI1_SCK                  5U
#define	PA06_SPI1_MISO                 6U
#define	PA07_ADC4                      7U
#define	PA08_LED_WS2812                8U
#define	PA09_UART1_TX                  9U
#define	PA10_UART1_RX                  10U
#define	PA11_USB_DM                    11U
#define	PA12_USB_DP                    12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15_ALARM                     15U

#define	PB00_SERVO1                    0U
#define	PB01_SERVO2                    1U
#define	PB02                           2U
#define	PB03_SPI3_SCK                  3U
#define	PB04_SPI3_MISO                 4U
#define	PB05_SPI3_MOSI                 5U
#define	PB06_I2C1_SCL                  6U
#define	PB07_I2C1_SDA                  7U
#define	PB08_UART4_RX                  8U
#define	PB09_UART4_TX                  9U
#define	PB10_I2C2_SCL                  10U
#define	PB11_I2C2_SDA                  11U
#define	PB12_SPI_SLAVE4                12U
#define	PB13_SPI2_SCK                  13U
#define	PB14_SPI2_MISO                 14U
#define	PB15_SPI2_MOSI                 15U

#define	PC00_ADC5                      0U
#define	PC01_ADC6                      1U
#define	PC02                           2U
#define	PC03                           3U
#define	PC04_ADC2                      4U
#define	PC05_ADC1                      5U
#define	PC06_UART6_TX                  6U
#define	PC07_RC1                       7U
#define	PC08_SDIO_D0                   8U
#define	PC09_SDIO_D1                   9U
#define	PC10_SDIO_D2                   10U
#define	PC11_SDIO_D3                   11U
#define	PC12_SDIO_CK                   12U
#define	PC13_SPI_SLAVE6                13U
#define	PC14                           14U
#define	PC15_SPI_SLAVE5                15U

#define	PD00_CAN1_RX                   0U
#define	PD01_CAN1_TX                   1U
#define	PD02_SDIO_CMD                  2U
#define	PD03                           3U
#define	PD04_SPI_SLAVE1                4U
#define	PD05_UART2_TX                  5U
#define	PD06_UART2_RX                  6U
#define	PD07_SPI1_MOSI                 7U
#define	PD08_UART3_TX                  8U
#define	PD09_UART3_RX                  9U
#define	PD10                           10U
#define	PD11                           11U
#define	PD12_SERVO7                    12U
#define	PD13_SERVO8                    13U
#define	PD14_SERVO9                    14U
#define	PD15_SERVO10                   15U

#define	PE00_UART8_RX                  0U
#define	PE01_UART8_TX                  1U
#define	PE02_SPI_SLAVE2                2U
#define	PE03_LED1                      3U
#define	PE04_LED2                      4U
#define	PE05_SERVO11                   5U
#define	PE06_SERVO12                   6U
#define	PE07_UART7_RX                  7U
#define	PE08_UART7_TX                  8U
#define	PE09_UART7_RTS                 9U
#define	PE10_UART7_CTS                 10U
#define	PE11_SPI_SLAVE3                11U
#define	PE12_SPI4_CLK                  12U
#define	PE13_SPI4_MISO                 13U
#define	PE14_SPI4_MOSI                 14U
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

#define	PH00_OSC_IN                    0U
#define	PH01_OSC_OUT                   1U
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
#define	LINE_SERVO3                    PAL_LINE(GPIOA, 0U)
#define	LINE_SERVO4                    PAL_LINE(GPIOA, 1U)
#define	LINE_SERVO5                    PAL_LINE(GPIOA, 2U)
#define	LINE_SERVO6                    PAL_LINE(GPIOA, 3U)
#define	LINE_ADC3                      PAL_LINE(GPIOA, 4U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_ADC4                      PAL_LINE(GPIOA, 7U)
#define	LINE_LED_WS2812                PAL_LINE(GPIOA, 8U)
#define	LINE_UART1_TX                  PAL_LINE(GPIOA, 9U)
#define	LINE_UART1_RX                  PAL_LINE(GPIOA, 10U)
#define	LINE_USB_DM                    PAL_LINE(GPIOA, 11U)
#define	LINE_USB_DP                    PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_ALARM                     PAL_LINE(GPIOA, 15U)

#define	LINE_SERVO1                    PAL_LINE(GPIOB, 0U)
#define	LINE_SERVO2                    PAL_LINE(GPIOB, 1U)
#define	LINE_SPI3_SCK                  PAL_LINE(GPIOB, 3U)
#define	LINE_SPI3_MISO                 PAL_LINE(GPIOB, 4U)
#define	LINE_SPI3_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 6U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 7U)
#define	LINE_UART4_RX                  PAL_LINE(GPIOB, 8U)
#define	LINE_UART4_TX                  PAL_LINE(GPIOB, 9U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define	LINE_SPI_SLAVE4                PAL_LINE(GPIOB, 12U)
#define	LINE_SPI2_SCK                  PAL_LINE(GPIOB, 13U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOB, 14U)
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOB, 15U)

#define	LINE_ADC5                      PAL_LINE(GPIOC, 0U)
#define	LINE_ADC6                      PAL_LINE(GPIOC, 1U)
#define	LINE_ADC2                      PAL_LINE(GPIOC, 4U)
#define	LINE_ADC1                      PAL_LINE(GPIOC, 5U)
#define	LINE_UART6_TX                  PAL_LINE(GPIOC, 6U)
#define	LINE_RC1                       PAL_LINE(GPIOC, 7U)
#define	LINE_SDIO_D0                   PAL_LINE(GPIOC, 8U)
#define	LINE_SDIO_D1                   PAL_LINE(GPIOC, 9U)
#define	LINE_SDIO_D2                   PAL_LINE(GPIOC, 10U)
#define	LINE_SDIO_D3                   PAL_LINE(GPIOC, 11U)
#define	LINE_SDIO_CK                   PAL_LINE(GPIOC, 12U)
#define	LINE_SPI_SLAVE6                PAL_LINE(GPIOC, 13U)
#define	LINE_SPI_SLAVE5                PAL_LINE(GPIOC, 15U)

#define	LINE_CAN1_RX                   PAL_LINE(GPIOD, 0U)
#define	LINE_CAN1_TX                   PAL_LINE(GPIOD, 1U)
#define	LINE_SDIO_CMD                  PAL_LINE(GPIOD, 2U)
#define	LINE_SPI_SLAVE1                PAL_LINE(GPIOD, 4U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOD, 5U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOD, 6U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOD, 7U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOD, 8U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOD, 9U)
#define	LINE_SERVO7                    PAL_LINE(GPIOD, 12U)
#define	LINE_SERVO8                    PAL_LINE(GPIOD, 13U)
#define	LINE_SERVO9                    PAL_LINE(GPIOD, 14U)
#define	LINE_SERVO10                   PAL_LINE(GPIOD, 15U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE_SPI_SLAVE2                PAL_LINE(GPIOE, 2U)
#define	LINE_LED1                      PAL_LINE(GPIOE, 3U)
#define	LINE_LED2                      PAL_LINE(GPIOE, 4U)
#define	LINE_SERVO11                   PAL_LINE(GPIOE, 5U)
#define	LINE_SERVO12                   PAL_LINE(GPIOE, 6U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOE, 7U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOE, 8U)
#define	LINE_UART7_RTS                 PAL_LINE(GPIOE, 9U)
#define	LINE_UART7_CTS                 PAL_LINE(GPIOE, 10U)
#define	LINE_SPI_SLAVE3                PAL_LINE(GPIOE, 11U)
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

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(PA00_SERVO3) | \
					 PIN_MODE_ALTERNATE(PA01_SERVO4) | \
					 PIN_MODE_ALTERNATE(PA02_SERVO5) | \
					 PIN_MODE_ALTERNATE(PA03_SERVO6) | \
					 PIN_MODE_ANALOG(PA04_ADC3) | \
					 PIN_MODE_ALTERNATE(PA05_SPI1_SCK) | \
					 PIN_MODE_ALTERNATE(PA06_SPI1_MISO) | \
					 PIN_MODE_ANALOG(PA07_ADC4) | \
					 PIN_MODE_ALTERNATE(PA08_LED_WS2812) | \
					 PIN_MODE_ALTERNATE(PA09_UART1_TX) | \
					 PIN_MODE_ALTERNATE(PA10_UART1_RX) | \
					 PIN_MODE_ALTERNATE(PA11_USB_DM) | \
					 PIN_MODE_ALTERNATE(PA12_USB_DP) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_ALTERNATE(PA15_ALARM))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_SERVO3) | \
					 PIN_OTYPE_PUSHPULL(PA01_SERVO4) | \
					 PIN_OTYPE_PUSHPULL(PA02_SERVO5) | \
					 PIN_OTYPE_PUSHPULL(PA03_SERVO6) | \
					 PIN_OTYPE_PUSHPULL(PA04_ADC3) | \
					 PIN_OTYPE_PUSHPULL(PA05_SPI1_SCK) | \
					 PIN_OTYPE_PUSHPULL(PA06_SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(PA07_ADC4) | \
					 PIN_OTYPE_PUSHPULL(PA08_LED_WS2812) | \
					 PIN_OTYPE_PUSHPULL(PA09_UART1_TX) | \
					 PIN_OTYPE_PUSHPULL(PA10_UART1_RX) | \
					 PIN_OTYPE_PUSHPULL(PA11_USB_DM) | \
					 PIN_OTYPE_PUSHPULL(PA12_USB_DP) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15_ALARM))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PA00_SERVO3) | \
					 PIN_OSPEED_SPEED_HIGH(PA01_SERVO4) | \
					 PIN_OSPEED_SPEED_HIGH(PA02_SERVO5) | \
					 PIN_OSPEED_SPEED_HIGH(PA03_SERVO6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04_ADC3) | \
					 PIN_OSPEED_SPEED_HIGH(PA05_SPI1_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PA06_SPI1_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA07_ADC4) | \
					 PIN_OSPEED_SPEED_HIGH(PA08_LED_WS2812) | \
					 PIN_OSPEED_SPEED_HIGH(PA09_UART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA10_UART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PA11_USB_DM) | \
					 PIN_OSPEED_SPEED_HIGH(PA12_USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(PA15_ALARM))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_SERVO3) | \
					 PIN_PUPDR_FLOATING(PA01_SERVO4) | \
					 PIN_PUPDR_FLOATING(PA02_SERVO5) | \
					 PIN_PUPDR_FLOATING(PA03_SERVO6) | \
					 PIN_PUPDR_FLOATING(PA04_ADC3) | \
					 PIN_PUPDR_FLOATING(PA05_SPI1_SCK) | \
					 PIN_PUPDR_FLOATING(PA06_SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(PA07_ADC4) | \
					 PIN_PUPDR_FLOATING(PA08_LED_WS2812) | \
					 PIN_PUPDR_FLOATING(PA09_UART1_TX) | \
					 PIN_PUPDR_FLOATING(PA10_UART1_RX) | \
					 PIN_PUPDR_FLOATING(PA11_USB_DM) | \
					 PIN_PUPDR_FLOATING(PA12_USB_DP) | \
					 PIN_PUPDR_FLOATING(PA13_SWDIO) | \
					 PIN_PUPDR_FLOATING(PA14_SWCLK) | \
					 PIN_PUPDR_FLOATING(PA15_ALARM))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(PA00_SERVO3) | \
					 PIN_ODR_LEVEL_LOW(PA01_SERVO4) | \
					 PIN_ODR_LEVEL_LOW(PA02_SERVO5) | \
					 PIN_ODR_LEVEL_LOW(PA03_SERVO6) | \
					 PIN_ODR_LEVEL_LOW(PA04_ADC3) | \
					 PIN_ODR_LEVEL_HIGH(PA05_SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PA06_SPI1_MISO) | \
					 PIN_ODR_LEVEL_LOW(PA07_ADC4) | \
					 PIN_ODR_LEVEL_LOW(PA08_LED_WS2812) | \
					 PIN_ODR_LEVEL_HIGH(PA09_UART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA10_UART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PA11_USB_DM) | \
					 PIN_ODR_LEVEL_HIGH(PA12_USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PA15_ALARM))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_SERVO3, 2) | \
					 PIN_AFIO_AF(PA01_SERVO4, 2) | \
					 PIN_AFIO_AF(PA02_SERVO5, 2) | \
					 PIN_AFIO_AF(PA03_SERVO6, 2) | \
					 PIN_AFIO_AF(PA04_ADC3, 0) | \
					 PIN_AFIO_AF(PA05_SPI1_SCK, 5) | \
					 PIN_AFIO_AF(PA06_SPI1_MISO, 5) | \
					 PIN_AFIO_AF(PA07_ADC4, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_LED_WS2812, 1) | \
					 PIN_AFIO_AF(PA09_UART1_TX, 7) | \
					 PIN_AFIO_AF(PA10_UART1_RX, 7) | \
					 PIN_AFIO_AF(PA11_USB_DM, 10) | \
					 PIN_AFIO_AF(PA12_USB_DP, 10) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15_ALARM, 1))

#define VAL_GPIOB_MODER                 (PIN_MODE_ALTERNATE(PB00_SERVO1) | \
					 PIN_MODE_ALTERNATE(PB01_SERVO2) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_ALTERNATE(PB03_SPI3_SCK) | \
					 PIN_MODE_ALTERNATE(PB04_SPI3_MISO) | \
					 PIN_MODE_ALTERNATE(PB05_SPI3_MOSI) | \
					 PIN_MODE_ALTERNATE(PB06_I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(PB07_I2C1_SDA) | \
					 PIN_MODE_ALTERNATE(PB08_UART4_RX) | \
					 PIN_MODE_ALTERNATE(PB09_UART4_TX) | \
					 PIN_MODE_ALTERNATE(PB10_I2C2_SCL) | \
					 PIN_MODE_ALTERNATE(PB11_I2C2_SDA) | \
					 PIN_MODE_OUTPUT(PB12_SPI_SLAVE4) | \
					 PIN_MODE_ALTERNATE(PB13_SPI2_SCK) | \
					 PIN_MODE_ALTERNATE(PB14_SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(PB15_SPI2_MOSI))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00_SERVO1) | \
					 PIN_OTYPE_PUSHPULL(PB01_SERVO2) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_PUSHPULL(PB03_SPI3_SCK) | \
					 PIN_OTYPE_PUSHPULL(PB04_SPI3_MISO) | \
					 PIN_OTYPE_PUSHPULL(PB05_SPI3_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(PB06_I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PB07_I2C1_SDA) | \
					 PIN_OTYPE_PUSHPULL(PB08_UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(PB09_UART4_TX) | \
					 PIN_OTYPE_OPENDRAIN(PB10_I2C2_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PB11_I2C2_SDA) | \
					 PIN_OTYPE_PUSHPULL(PB12_SPI_SLAVE4) | \
					 PIN_OTYPE_PUSHPULL(PB13_SPI2_SCK) | \
					 PIN_OTYPE_PUSHPULL(PB14_SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(PB15_SPI2_MOSI))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PB00_SERVO1) | \
					 PIN_OSPEED_SPEED_HIGH(PB01_SERVO2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_HIGH(PB03_SPI3_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PB04_SPI3_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PB05_SPI3_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PB07_I2C1_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PB08_UART4_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB09_UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PB10_I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PB11_I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PB12_SPI_SLAVE4) | \
					 PIN_OSPEED_SPEED_HIGH(PB13_SPI2_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PB14_SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PB15_SPI2_MOSI))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PB00_SERVO1) | \
					 PIN_PUPDR_FLOATING(PB01_SERVO2) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_FLOATING(PB03_SPI3_SCK) | \
					 PIN_PUPDR_FLOATING(PB04_SPI3_MISO) | \
					 PIN_PUPDR_FLOATING(PB05_SPI3_MOSI) | \
					 PIN_PUPDR_PULLUP(PB06_I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(PB07_I2C1_SDA) | \
					 PIN_PUPDR_FLOATING(PB08_UART4_RX) | \
					 PIN_PUPDR_FLOATING(PB09_UART4_TX) | \
					 PIN_PUPDR_PULLUP(PB10_I2C2_SCL) | \
					 PIN_PUPDR_PULLUP(PB11_I2C2_SDA) | \
					 PIN_PUPDR_FLOATING(PB12_SPI_SLAVE4) | \
					 PIN_PUPDR_FLOATING(PB13_SPI2_SCK) | \
					 PIN_PUPDR_FLOATING(PB14_SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(PB15_SPI2_MOSI))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00_SERVO1) | \
					 PIN_ODR_LEVEL_LOW(PB01_SERVO2) | \
					 PIN_ODR_LEVEL_LOW(PB02) | \
					 PIN_ODR_LEVEL_HIGH(PB03_SPI3_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PB04_SPI3_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PB05_SPI3_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PB06_I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PB07_I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PB08_UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB09_UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB10_I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PB11_I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PB12_SPI_SLAVE4) | \
					 PIN_ODR_LEVEL_HIGH(PB13_SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PB14_SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PB15_SPI2_MOSI))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_SERVO1, 2) | \
					 PIN_AFIO_AF(PB01_SERVO2, 2) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(PB03_SPI3_SCK, 6) | \
					 PIN_AFIO_AF(PB04_SPI3_MISO, 6) | \
					 PIN_AFIO_AF(PB05_SPI3_MOSI, 7) | \
					 PIN_AFIO_AF(PB06_I2C1_SCL, 4) | \
					 PIN_AFIO_AF(PB07_I2C1_SDA, 4))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08_UART4_RX, 8) | \
					 PIN_AFIO_AF(PB09_UART4_TX, 8) | \
					 PIN_AFIO_AF(PB10_I2C2_SCL, 4) | \
					 PIN_AFIO_AF(PB11_I2C2_SDA, 4) | \
					 PIN_AFIO_AF(PB12_SPI_SLAVE4, 0) | \
					 PIN_AFIO_AF(PB13_SPI2_SCK, 5) | \
					 PIN_AFIO_AF(PB14_SPI2_MISO, 5) | \
					 PIN_AFIO_AF(PB15_SPI2_MOSI, 5))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(PC00_ADC5) | \
					 PIN_MODE_ANALOG(PC01_ADC6) | \
					 PIN_MODE_INPUT(PC02) | \
					 PIN_MODE_INPUT(PC03) | \
					 PIN_MODE_ANALOG(PC04_ADC2) | \
					 PIN_MODE_ANALOG(PC05_ADC1) | \
					 PIN_MODE_ALTERNATE(PC06_UART6_TX) | \
					 PIN_MODE_INPUT(PC07_RC1) | \
					 PIN_MODE_ALTERNATE(PC08_SDIO_D0) | \
					 PIN_MODE_ALTERNATE(PC09_SDIO_D1) | \
					 PIN_MODE_ALTERNATE(PC10_SDIO_D2) | \
					 PIN_MODE_ALTERNATE(PC11_SDIO_D3) | \
					 PIN_MODE_ALTERNATE(PC12_SDIO_CK) | \
					 PIN_MODE_OUTPUT(PC13_SPI_SLAVE6) | \
					 PIN_MODE_INPUT(PC14) | \
					 PIN_MODE_OUTPUT(PC15_SPI_SLAVE5))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(PC00_ADC5) | \
					 PIN_OTYPE_PUSHPULL(PC01_ADC6) | \
					 PIN_OTYPE_PUSHPULL(PC02) | \
					 PIN_OTYPE_PUSHPULL(PC03) | \
					 PIN_OTYPE_PUSHPULL(PC04_ADC2) | \
					 PIN_OTYPE_PUSHPULL(PC05_ADC1) | \
					 PIN_OTYPE_PUSHPULL(PC06_UART6_TX) | \
					 PIN_OTYPE_OPENDRAIN(PC07_RC1) | \
					 PIN_OTYPE_PUSHPULL(PC08_SDIO_D0) | \
					 PIN_OTYPE_PUSHPULL(PC09_SDIO_D1) | \
					 PIN_OTYPE_PUSHPULL(PC10_SDIO_D2) | \
					 PIN_OTYPE_PUSHPULL(PC11_SDIO_D3) | \
					 PIN_OTYPE_PUSHPULL(PC12_SDIO_CK) | \
					 PIN_OTYPE_PUSHPULL(PC13_SPI_SLAVE6) | \
					 PIN_OTYPE_PUSHPULL(PC14) | \
					 PIN_OTYPE_PUSHPULL(PC15_SPI_SLAVE5))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00_ADC5) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01_ADC6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04_ADC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05_ADC1) | \
					 PIN_OSPEED_SPEED_HIGH(PC06_UART6_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC07_RC1) | \
					 PIN_OSPEED_SPEED_HIGH(PC08_SDIO_D0) | \
					 PIN_OSPEED_SPEED_HIGH(PC09_SDIO_D1) | \
					 PIN_OSPEED_SPEED_HIGH(PC10_SDIO_D2) | \
					 PIN_OSPEED_SPEED_HIGH(PC11_SDIO_D3) | \
					 PIN_OSPEED_SPEED_HIGH(PC12_SDIO_CK) | \
					 PIN_OSPEED_SPEED_HIGH(PC13_SPI_SLAVE6) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC14) | \
					 PIN_OSPEED_SPEED_HIGH(PC15_SPI_SLAVE5))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(PC00_ADC5) | \
					 PIN_PUPDR_FLOATING(PC01_ADC6) | \
					 PIN_PUPDR_PULLDOWN(PC02) | \
					 PIN_PUPDR_PULLDOWN(PC03) | \
					 PIN_PUPDR_FLOATING(PC04_ADC2) | \
					 PIN_PUPDR_FLOATING(PC05_ADC1) | \
					 PIN_PUPDR_FLOATING(PC06_UART6_TX) | \
					 PIN_PUPDR_PULLDOWN(PC07_RC1) | \
					 PIN_PUPDR_PULLUP(PC08_SDIO_D0) | \
					 PIN_PUPDR_PULLUP(PC09_SDIO_D1) | \
					 PIN_PUPDR_PULLUP(PC10_SDIO_D2) | \
					 PIN_PUPDR_PULLUP(PC11_SDIO_D3) | \
					 PIN_PUPDR_FLOATING(PC12_SDIO_CK) | \
					 PIN_PUPDR_FLOATING(PC13_SPI_SLAVE6) | \
					 PIN_PUPDR_PULLDOWN(PC14) | \
					 PIN_PUPDR_FLOATING(PC15_SPI_SLAVE5))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(PC00_ADC5) | \
					 PIN_ODR_LEVEL_LOW(PC01_ADC6) | \
					 PIN_ODR_LEVEL_LOW(PC02) | \
					 PIN_ODR_LEVEL_LOW(PC03) | \
					 PIN_ODR_LEVEL_LOW(PC04_ADC2) | \
					 PIN_ODR_LEVEL_LOW(PC05_ADC1) | \
					 PIN_ODR_LEVEL_HIGH(PC06_UART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(PC07_RC1) | \
					 PIN_ODR_LEVEL_HIGH(PC08_SDIO_D0) | \
					 PIN_ODR_LEVEL_HIGH(PC09_SDIO_D1) | \
					 PIN_ODR_LEVEL_HIGH(PC10_SDIO_D2) | \
					 PIN_ODR_LEVEL_HIGH(PC11_SDIO_D3) | \
					 PIN_ODR_LEVEL_HIGH(PC12_SDIO_CK) | \
					 PIN_ODR_LEVEL_HIGH(PC13_SPI_SLAVE6) | \
					 PIN_ODR_LEVEL_LOW(PC14) | \
					 PIN_ODR_LEVEL_HIGH(PC15_SPI_SLAVE5))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00_ADC5, 0) | \
					 PIN_AFIO_AF(PC01_ADC6, 0) | \
					 PIN_AFIO_AF(PC02, 0) | \
					 PIN_AFIO_AF(PC03, 0) | \
					 PIN_AFIO_AF(PC04_ADC2, 0) | \
					 PIN_AFIO_AF(PC05_ADC1, 0) | \
					 PIN_AFIO_AF(PC06_UART6_TX, 7) | \
					 PIN_AFIO_AF(PC07_RC1, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08_SDIO_D0, 12) | \
					 PIN_AFIO_AF(PC09_SDIO_D1, 12) | \
					 PIN_AFIO_AF(PC10_SDIO_D2, 12) | \
					 PIN_AFIO_AF(PC11_SDIO_D3, 12) | \
					 PIN_AFIO_AF(PC12_SDIO_CK, 12) | \
					 PIN_AFIO_AF(PC13_SPI_SLAVE6, 0) | \
					 PIN_AFIO_AF(PC14, 0) | \
					 PIN_AFIO_AF(PC15_SPI_SLAVE5, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(PD00_CAN1_RX) | \
					 PIN_MODE_ALTERNATE(PD01_CAN1_TX) | \
					 PIN_MODE_ALTERNATE(PD02_SDIO_CMD) | \
					 PIN_MODE_INPUT(PD03) | \
					 PIN_MODE_OUTPUT(PD04_SPI_SLAVE1) | \
					 PIN_MODE_ALTERNATE(PD05_UART2_TX) | \
					 PIN_MODE_ALTERNATE(PD06_UART2_RX) | \
					 PIN_MODE_ALTERNATE(PD07_SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(PD08_UART3_TX) | \
					 PIN_MODE_ALTERNATE(PD09_UART3_RX) | \
					 PIN_MODE_INPUT(PD10) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_ALTERNATE(PD12_SERVO7) | \
					 PIN_MODE_ALTERNATE(PD13_SERVO8) | \
					 PIN_MODE_ALTERNATE(PD14_SERVO9) | \
					 PIN_MODE_ALTERNATE(PD15_SERVO10))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00_CAN1_RX) | \
					 PIN_OTYPE_PUSHPULL(PD01_CAN1_TX) | \
					 PIN_OTYPE_PUSHPULL(PD02_SDIO_CMD) | \
					 PIN_OTYPE_PUSHPULL(PD03) | \
					 PIN_OTYPE_PUSHPULL(PD04_SPI_SLAVE1) | \
					 PIN_OTYPE_PUSHPULL(PD05_UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(PD06_UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(PD07_SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PD08_UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(PD09_UART3_RX) | \
					 PIN_OTYPE_PUSHPULL(PD10) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_PUSHPULL(PD12_SERVO7) | \
					 PIN_OTYPE_PUSHPULL(PD13_SERVO8) | \
					 PIN_OTYPE_PUSHPULL(PD14_SERVO9) | \
					 PIN_OTYPE_PUSHPULL(PD15_SERVO10))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PD00_CAN1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PD01_CAN1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD02_SDIO_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_HIGH(PD04_SPI_SLAVE1) | \
					 PIN_OSPEED_SPEED_HIGH(PD05_UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD06_UART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PD07_SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PD08_UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PD09_UART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_HIGH(PD12_SERVO7) | \
					 PIN_OSPEED_SPEED_HIGH(PD13_SERVO8) | \
					 PIN_OSPEED_SPEED_HIGH(PD14_SERVO9) | \
					 PIN_OSPEED_SPEED_HIGH(PD15_SERVO10))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(PD00_CAN1_RX) | \
					 PIN_PUPDR_FLOATING(PD01_CAN1_TX) | \
					 PIN_PUPDR_PULLUP(PD02_SDIO_CMD) | \
					 PIN_PUPDR_PULLDOWN(PD03) | \
					 PIN_PUPDR_FLOATING(PD04_SPI_SLAVE1) | \
					 PIN_PUPDR_FLOATING(PD05_UART2_TX) | \
					 PIN_PUPDR_FLOATING(PD06_UART2_RX) | \
					 PIN_PUPDR_FLOATING(PD07_SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(PD08_UART3_TX) | \
					 PIN_PUPDR_FLOATING(PD09_UART3_RX) | \
					 PIN_PUPDR_PULLDOWN(PD10) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_FLOATING(PD12_SERVO7) | \
					 PIN_PUPDR_FLOATING(PD13_SERVO8) | \
					 PIN_PUPDR_FLOATING(PD14_SERVO9) | \
					 PIN_PUPDR_FLOATING(PD15_SERVO10))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(PD00_CAN1_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD01_CAN1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD02_SDIO_CMD) | \
					 PIN_ODR_LEVEL_LOW(PD03) | \
					 PIN_ODR_LEVEL_HIGH(PD04_SPI_SLAVE1) | \
					 PIN_ODR_LEVEL_HIGH(PD05_UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD06_UART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(PD07_SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(PD08_UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(PD09_UART3_RX) | \
					 PIN_ODR_LEVEL_LOW(PD10) | \
					 PIN_ODR_LEVEL_LOW(PD11) | \
					 PIN_ODR_LEVEL_LOW(PD12_SERVO7) | \
					 PIN_ODR_LEVEL_LOW(PD13_SERVO8) | \
					 PIN_ODR_LEVEL_LOW(PD14_SERVO9) | \
					 PIN_ODR_LEVEL_LOW(PD15_SERVO10))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00_CAN1_RX, 9) | \
					 PIN_AFIO_AF(PD01_CAN1_TX, 9) | \
					 PIN_AFIO_AF(PD02_SDIO_CMD, 12) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04_SPI_SLAVE1, 0) | \
					 PIN_AFIO_AF(PD05_UART2_TX, 7) | \
					 PIN_AFIO_AF(PD06_UART2_RX, 7) | \
					 PIN_AFIO_AF(PD07_SPI1_MOSI, 5))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08_UART3_TX, 7) | \
					 PIN_AFIO_AF(PD09_UART3_RX, 7) | \
					 PIN_AFIO_AF(PD10, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12_SERVO7, 2) | \
					 PIN_AFIO_AF(PD13_SERVO8, 2) | \
					 PIN_AFIO_AF(PD14_SERVO9, 2) | \
					 PIN_AFIO_AF(PD15_SERVO10, 2))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(PE00_UART8_RX) | \
					 PIN_MODE_ALTERNATE(PE01_UART8_TX) | \
					 PIN_MODE_OUTPUT(PE02_SPI_SLAVE2) | \
					 PIN_MODE_OUTPUT(PE03_LED1) | \
					 PIN_MODE_OUTPUT(PE04_LED2) | \
					 PIN_MODE_ALTERNATE(PE05_SERVO11) | \
					 PIN_MODE_ALTERNATE(PE06_SERVO12) | \
					 PIN_MODE_ALTERNATE(PE07_UART7_RX) | \
					 PIN_MODE_ALTERNATE(PE08_UART7_TX) | \
					 PIN_MODE_INPUT(PE09_UART7_RTS) | \
					 PIN_MODE_INPUT(PE10_UART7_CTS) | \
					 PIN_MODE_OUTPUT(PE11_SPI_SLAVE3) | \
					 PIN_MODE_ALTERNATE(PE12_SPI4_CLK) | \
					 PIN_MODE_ALTERNATE(PE13_SPI4_MISO) | \
					 PIN_MODE_ALTERNATE(PE14_SPI4_MOSI) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00_UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(PE01_UART8_TX) | \
					 PIN_OTYPE_PUSHPULL(PE02_SPI_SLAVE2) | \
					 PIN_OTYPE_PUSHPULL(PE03_LED1) | \
					 PIN_OTYPE_PUSHPULL(PE04_LED2) | \
					 PIN_OTYPE_PUSHPULL(PE05_SERVO11) | \
					 PIN_OTYPE_PUSHPULL(PE06_SERVO12) | \
					 PIN_OTYPE_PUSHPULL(PE07_UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(PE08_UART7_TX) | \
					 PIN_OTYPE_OPENDRAIN(PE09_UART7_RTS) | \
					 PIN_OTYPE_OPENDRAIN(PE10_UART7_CTS) | \
					 PIN_OTYPE_PUSHPULL(PE11_SPI_SLAVE3) | \
					 PIN_OTYPE_PUSHPULL(PE12_SPI4_CLK) | \
					 PIN_OTYPE_PUSHPULL(PE13_SPI4_MISO) | \
					 PIN_OTYPE_PUSHPULL(PE14_SPI4_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PE00_UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PE01_UART8_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PE02_SPI_SLAVE2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03_LED1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04_LED2) | \
					 PIN_OSPEED_SPEED_HIGH(PE05_SERVO11) | \
					 PIN_OSPEED_SPEED_HIGH(PE06_SERVO12) | \
					 PIN_OSPEED_SPEED_HIGH(PE07_UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PE08_UART7_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09_UART7_RTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10_UART7_CTS) | \
					 PIN_OSPEED_SPEED_HIGH(PE11_SPI_SLAVE3) | \
					 PIN_OSPEED_SPEED_HIGH(PE12_SPI4_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(PE13_SPI4_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PE14_SPI4_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(PE00_UART8_RX) | \
					 PIN_PUPDR_FLOATING(PE01_UART8_TX) | \
					 PIN_PUPDR_FLOATING(PE02_SPI_SLAVE2) | \
					 PIN_PUPDR_FLOATING(PE03_LED1) | \
					 PIN_PUPDR_FLOATING(PE04_LED2) | \
					 PIN_PUPDR_FLOATING(PE05_SERVO11) | \
					 PIN_PUPDR_FLOATING(PE06_SERVO12) | \
					 PIN_PUPDR_FLOATING(PE07_UART7_RX) | \
					 PIN_PUPDR_FLOATING(PE08_UART7_TX) | \
					 PIN_PUPDR_PULLDOWN(PE09_UART7_RTS) | \
					 PIN_PUPDR_PULLDOWN(PE10_UART7_CTS) | \
					 PIN_PUPDR_FLOATING(PE11_SPI_SLAVE3) | \
					 PIN_PUPDR_FLOATING(PE12_SPI4_CLK) | \
					 PIN_PUPDR_FLOATING(PE13_SPI4_MISO) | \
					 PIN_PUPDR_FLOATING(PE14_SPI4_MOSI) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(PE00_UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(PE01_UART8_TX) | \
					 PIN_ODR_LEVEL_HIGH(PE02_SPI_SLAVE2) | \
					 PIN_ODR_LEVEL_LOW(PE03_LED1) | \
					 PIN_ODR_LEVEL_LOW(PE04_LED2) | \
					 PIN_ODR_LEVEL_LOW(PE05_SERVO11) | \
					 PIN_ODR_LEVEL_LOW(PE06_SERVO12) | \
					 PIN_ODR_LEVEL_HIGH(PE07_UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(PE08_UART7_TX) | \
					 PIN_ODR_LEVEL_HIGH(PE09_UART7_RTS) | \
					 PIN_ODR_LEVEL_HIGH(PE10_UART7_CTS) | \
					 PIN_ODR_LEVEL_HIGH(PE11_SPI_SLAVE3) | \
					 PIN_ODR_LEVEL_HIGH(PE12_SPI4_CLK) | \
					 PIN_ODR_LEVEL_HIGH(PE13_SPI4_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PE14_SPI4_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00_UART8_RX, 8) | \
					 PIN_AFIO_AF(PE01_UART8_TX, 8) | \
					 PIN_AFIO_AF(PE02_SPI_SLAVE2, 0) | \
					 PIN_AFIO_AF(PE03_LED1, 0) | \
					 PIN_AFIO_AF(PE04_LED2, 0) | \
					 PIN_AFIO_AF(PE05_SERVO11, 4) | \
					 PIN_AFIO_AF(PE06_SERVO12, 4) | \
					 PIN_AFIO_AF(PE07_UART7_RX, 7))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08_UART7_TX, 7) | \
					 PIN_AFIO_AF(PE09_UART7_RTS, 0) | \
					 PIN_AFIO_AF(PE10_UART7_CTS, 0) | \
					 PIN_AFIO_AF(PE11_SPI_SLAVE3, 0) | \
					 PIN_AFIO_AF(PE12_SPI4_CLK, 5) | \
					 PIN_AFIO_AF(PE13_SPI4_MISO, 5) | \
					 PIN_AFIO_AF(PE14_SPI4_MOSI, 5) | \
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

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(PH00_OSC_IN) | \
					 PIN_MODE_ALTERNATE(PH01_OSC_OUT) | \
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

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(PH00_OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(PH01_OSC_OUT) | \
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

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PH00_OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PH01_OSC_OUT) | \
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

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(PH00_OSC_IN) | \
					 PIN_PUPDR_FLOATING(PH01_OSC_OUT) | \
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

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(PH00_OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(PH01_OSC_OUT) | \
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

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(PH00_OSC_IN, 0) | \
					 PIN_AFIO_AF(PH01_OSC_OUT, 0) | \
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

#define AF_PA00_SERVO3                   2U
#define AF_LINE_SERVO3                   2U
#define AF_PA01_SERVO4                   2U
#define AF_LINE_SERVO4                   2U
#define AF_PA02_SERVO5                   2U
#define AF_LINE_SERVO5                   2U
#define AF_PA03_SERVO6                   2U
#define AF_LINE_SERVO6                   2U
#define AF_PA05_SPI1_SCK                 5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_PA06_SPI1_MISO                5U
#define AF_LINE_SPI1_MISO                5U
#define AF_PA08_LED_WS2812               1U
#define AF_LINE_LED_WS2812               1U
#define AF_PA09_UART1_TX                 7U
#define AF_LINE_UART1_TX                 7U
#define AF_PA10_UART1_RX                 7U
#define AF_LINE_UART1_RX                 7U
#define AF_PA11_USB_DM                   10U
#define AF_LINE_USB_DM                   10U
#define AF_PA12_USB_DP                   10U
#define AF_LINE_USB_DP                   10U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PA15_ALARM                    1U
#define AF_LINE_ALARM                    1U
#define AF_PB00_SERVO1                   2U
#define AF_LINE_SERVO1                   2U
#define AF_PB01_SERVO2                   2U
#define AF_LINE_SERVO2                   2U
#define AF_PB03_SPI3_SCK                 6U
#define AF_LINE_SPI3_SCK                 6U
#define AF_PB04_SPI3_MISO                6U
#define AF_LINE_SPI3_MISO                6U
#define AF_PB05_SPI3_MOSI                7U
#define AF_LINE_SPI3_MOSI                7U
#define AF_PB06_I2C1_SCL                 4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_PB07_I2C1_SDA                 4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_PB08_UART4_RX                 8U
#define AF_LINE_UART4_RX                 8U
#define AF_PB09_UART4_TX                 8U
#define AF_LINE_UART4_TX                 8U
#define AF_PB10_I2C2_SCL                 4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_PB11_I2C2_SDA                 4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_PB13_SPI2_SCK                 5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_PB14_SPI2_MISO                5U
#define AF_LINE_SPI2_MISO                5U
#define AF_PB15_SPI2_MOSI                5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_PC06_UART6_TX                 7U
#define AF_LINE_UART6_TX                 7U
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
#define AF_PD00_CAN1_RX                  9U
#define AF_LINE_CAN1_RX                  9U
#define AF_PD01_CAN1_TX                  9U
#define AF_LINE_CAN1_TX                  9U
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
#define AF_PD12_SERVO7                   2U
#define AF_LINE_SERVO7                   2U
#define AF_PD13_SERVO8                   2U
#define AF_LINE_SERVO8                   2U
#define AF_PD14_SERVO9                   2U
#define AF_LINE_SERVO9                   2U
#define AF_PD15_SERVO10                  2U
#define AF_LINE_SERVO10                  2U
#define AF_PE00_UART8_RX                 8U
#define AF_LINE_UART8_RX                 8U
#define AF_PE01_UART8_TX                 8U
#define AF_LINE_UART8_TX                 8U
#define AF_PE05_SERVO11                  4U
#define AF_LINE_SERVO11                  4U
#define AF_PE06_SERVO12                  4U
#define AF_LINE_SERVO12                  4U
#define AF_PE07_UART7_RX                 7U
#define AF_LINE_UART7_RX                 7U
#define AF_PE08_UART7_TX                 7U
#define AF_LINE_UART7_TX                 7U
#define AF_PE12_SPI4_CLK                 5U
#define AF_LINE_SPI4_CLK                 5U
#define AF_PE13_SPI4_MISO                5U
#define AF_LINE_SPI4_MISO                5U
#define AF_PE14_SPI4_MOSI                5U
#define AF_LINE_SPI4_MOSI                5U
#define AF_PH00_OSC_IN                   0U
#define AF_LINE_OSC_IN                   0U
#define AF_PH01_OSC_OUT                  0U
#define AF_LINE_OSC_OUT                  0U


#define SERVO3_TIM	 5
#define SERVO3_TIM_FN	 CH
#define SERVO3_TIM_CH	 1
#define SERVO3_TIM_AF	 2
#define SERVO4_TIM	 5
#define SERVO4_TIM_FN	 CH
#define SERVO4_TIM_CH	 2
#define SERVO4_TIM_AF	 2
#define SERVO5_TIM	 5
#define SERVO5_TIM_FN	 CH
#define SERVO5_TIM_CH	 3
#define SERVO5_TIM_AF	 2
#define SERVO6_TIM	 5
#define SERVO6_TIM_FN	 CH
#define SERVO6_TIM_CH	 4
#define SERVO6_TIM_AF	 2
#define ADC3_ADC	 1
#define ADC3_ADC_FN	 INP
#define ADC3_ADC_INP	 18
#define ADC4_ADC	 1
#define ADC4_ADC_FN	 INP
#define ADC4_ADC_INP	 7
#define LED_WS2812_TIM	 1
#define LED_WS2812_TIM_FN	 CH
#define LED_WS2812_TIM_CH	 1
#define LED_WS2812_TIM_AF	 1
#define ALARM_TIM	 2
#define ALARM_TIM_FN	 CH
#define ALARM_TIM_CH	 1
#define ALARM_TIM_AF	 1
#define SERVO1_TIM	 3
#define SERVO1_TIM_FN	 CH
#define SERVO1_TIM_CH	 3
#define SERVO1_TIM_AF	 2
#define SERVO2_TIM	 3
#define SERVO2_TIM_FN	 CH
#define SERVO2_TIM_CH	 4
#define SERVO2_TIM_AF	 2
#define ADC5_ADC	 1
#define ADC5_ADC_FN	 INP
#define ADC5_ADC_INP	 10
#define ADC6_ADC	 1
#define ADC6_ADC_FN	 INP
#define ADC6_ADC_INP	 11
#define ADC1_ADC	 1
#define ADC1_ADC_FN	 INP
#define ADC1_ADC_INP	 8
#define ADC2_ADC	 1
#define ADC2_ADC_FN	 INP
#define ADC2_ADC_INP	 4
#define RC1_TIM	 3
#define RC1_TIM_FN	 CH
#define RC1_TIM_CH	 2
#define RC1_TIM_AF	 2
#define RC1_USART	 6
#define RC1_USART_FN	 RX
#define RC1_USART_AF	 7
#define SERVO7_TIM	 4
#define SERVO7_TIM_FN	 CH
#define SERVO7_TIM_CH	 1
#define SERVO7_TIM_AF	 2
#define SERVO8_TIM	 4
#define SERVO8_TIM_FN	 CH
#define SERVO8_TIM_CH	 2
#define SERVO8_TIM_AF	 2
#define SERVO9_TIM	 4
#define SERVO9_TIM_FN	 CH
#define SERVO9_TIM_CH	 3
#define SERVO9_TIM_AF	 2
#define SERVO10_TIM	 4
#define SERVO10_TIM_FN	 CH
#define SERVO10_TIM_CH	 4
#define SERVO10_TIM_AF	 2
#define SERVO11_TIM	 15
#define SERVO11_TIM_FN	 CH
#define SERVO11_TIM_CH	 1
#define SERVO11_TIM_AF	 4
#define SERVO12_TIM	 15
#define SERVO12_TIM_FN	 CH
#define SERVO12_TIM_CH	 2
#define SERVO12_TIM_AF	 4

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
	LINE_SERVO3, \
	LINE_SERVO4, \
	LINE_SERVO5, \
	LINE_SERVO6, \
	LINE_SERVO1, \
	LINE_SERVO2, \
	LINE_SPI_SLAVE4, \
	LINE_SPI_SLAVE6, \
	LINE_SPI_SLAVE5, \
	LINE_SPI_SLAVE1, \
	LINE_SERVO7, \
	LINE_SERVO8, \
	LINE_SERVO9, \
	LINE_SERVO10, \
	LINE_SPI_SLAVE2, \
	LINE_LED1, \
	LINE_LED2, \
	LINE_SERVO11, \
	LINE_SERVO12, \
	LINE_SPI_SLAVE3
#define ENERGY_SAVE_INPUTS_SIZE 	 20

#define ENERGY_SAVE_LOWS \
	LINE_ALARM
#define ENERGY_SAVE_LOWS_SIZE 	 1

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

