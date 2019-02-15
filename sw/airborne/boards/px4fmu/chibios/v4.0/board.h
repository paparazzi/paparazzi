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
#define BOARD_NAME                  "Pixhawk PX4 FMU v 4.0"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F427xx

/*
 * IO pins assignments.
 */
#define	USART4_TX                      0U
#define	USART4_RX                      1U
#define	VOLT_SENS                      2U
#define	CURRENT_SENS                   3U
#define	VDD_5V_SENS                    4U
#define	MPU9250_SCK                    5U
#define	MPU9250_MISO                   6U
#define	MPU9250_MOSI                   7U
#define	_8266_RTS                      8U
#define	USB_VBUS                       9U
#define	FRSKY_INV                      10U
#define	OTG_FS_DM                      11U
#define	OTG_FS_DP                      12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	ALARM                          15U

#define	_RC_INPUT                      0U
#define	LED_GREEN                      1U
#define	BOOT1                          2U
#define	LED_BLUE                       3U
#define	_8266_GPIO2                    4U
#define	VDD_BRICK_VALID                5U
#define	USART1_TX                      6U
#define	USART1_RX                      7U
#define	I2C1_SCL                       8U
#define	I2C1_SDA                       9U
#define	FRAM_SCK                       10U
#define	LED_RED                        11U
#define	CAN2_RX                        12U
#define	CAN2_TX                        13U
#define	FRAM_MISO                      14U
#define	FRAM_MOSI                      15U

#define	VBUS_VALID                     0U
#define	RSSI_IN                        1U
#define	MPU9250_CS                     2U
#define	LED_SAFETY                     3U
#define	SAFETY_SW_IN                   4U
#define	VDD_3V3_EN                     5U
#define	RC_OUTPUT                      6U
#define	RC_INPUT                       7U
#define	SDMMC1_D0                      8U
#define	SDMMC1_D1                      9U
#define	SDMMC1_D2                      10U
#define	SDMMC1_D3                      11U
#define	SDMMC1_CK                      12U
#define	SBUS_INV                       13U
#define	_20608_DRDY                    14U
#define	_20608_CS                      15U

#define	CAN1_RX                        0U
#define	CAN1_TX                        1U
#define	SDMMC1_CMD                     2U
#define	USART2_CTS                     3U
#define	USART2_RTS                     4U
#define	USART2_TX                      5U
#define	USART2_RX                      6U
#define	BARO_CS                        7U
#define	USART3_TX                      8U
#define	USART3_RX                      9U
#define	FRAM_CS                        10U
#define	USART3_CTS                     11U
#define	USART3_RTS                     12U
#define	SRV5_TIM4_CH2                  13U
#define	SRV6_TIM4_CH3                  14U
#define	MPU9250_DRDY                   15U

#define	UART8_RX                       0U
#define	UART8_TX                       1U
#define	_8266_GPIO0                    2U
#define	VDD_3V3_SENS_EN                3U
#define	SPEKTRUM_POWER                 4U
#define	_8266_PD                       5U
#define	_8266_RST                      6U
#define	UART7_RX                       7U
#define	UART7_TX                       8U
#define	SRV4_TIM1_CH1                  9U
#define	_8266_CTS                      10U
#define	SRV3_TIM1_CH2                  11U
#define	HMC5983_DRDY                   12U
#define	SRV2_TIM1_CH3                  13U
#define	SRV1_TIM1_CH4                  14U
#define	HMC_5983                       15U

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
#define	LINE_USART4_TX                 PAL_LINE(GPIOA, 0U)
#define	LINE_USART4_RX                 PAL_LINE(GPIOA, 1U)
#define	LINE_VOLT_SENS                 PAL_LINE(GPIOA, 2U)
#define	LINE_CURRENT_SENS              PAL_LINE(GPIOA, 3U)
#define	LINE_VDD_5V_SENS               PAL_LINE(GPIOA, 4U)
#define	LINE_MPU9250_SCK               PAL_LINE(GPIOA, 5U)
#define	LINE_MPU9250_MISO              PAL_LINE(GPIOA, 6U)
#define	LINE_MPU9250_MOSI              PAL_LINE(GPIOA, 7U)
#define	LINE__8266_RTS                 PAL_LINE(GPIOA, 8U)
#define	LINE_USB_VBUS                  PAL_LINE(GPIOA, 9U)
#define	LINE_FRSKY_INV                 PAL_LINE(GPIOA, 10U)
#define	LINE_OTG_FS_DM                 PAL_LINE(GPIOA, 11U)
#define	LINE_OTG_FS_DP                 PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_ALARM                     PAL_LINE(GPIOA, 15U)

#define	LINE__RC_INPUT                 PAL_LINE(GPIOB, 0U)
#define	LINE_LED_GREEN                 PAL_LINE(GPIOB, 1U)
#define	LINE_BOOT1                     PAL_LINE(GPIOB, 2U)
#define	LINE_LED_BLUE                  PAL_LINE(GPIOB, 3U)
#define	LINE__8266_GPIO2               PAL_LINE(GPIOB, 4U)
#define	LINE_VDD_BRICK_VALID           PAL_LINE(GPIOB, 5U)
#define	LINE_USART1_TX                 PAL_LINE(GPIOB, 6U)
#define	LINE_USART1_RX                 PAL_LINE(GPIOB, 7U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 9U)
#define	LINE_FRAM_SCK                  PAL_LINE(GPIOB, 10U)
#define	LINE_LED_RED                   PAL_LINE(GPIOB, 11U)
#define	LINE_CAN2_RX                   PAL_LINE(GPIOB, 12U)
#define	LINE_CAN2_TX                   PAL_LINE(GPIOB, 13U)
#define	LINE_FRAM_MISO                 PAL_LINE(GPIOB, 14U)
#define	LINE_FRAM_MOSI                 PAL_LINE(GPIOB, 15U)

#define	LINE_VBUS_VALID                PAL_LINE(GPIOC, 0U)
#define	LINE_RSSI_IN                   PAL_LINE(GPIOC, 1U)
#define	LINE_MPU9250_CS                PAL_LINE(GPIOC, 2U)
#define	LINE_LED_SAFETY                PAL_LINE(GPIOC, 3U)
#define	LINE_SAFETY_SW_IN              PAL_LINE(GPIOC, 4U)
#define	LINE_VDD_3V3_EN                PAL_LINE(GPIOC, 5U)
#define	LINE_RC_OUTPUT                 PAL_LINE(GPIOC, 6U)
#define	LINE_RC_INPUT                  PAL_LINE(GPIOC, 7U)
#define	LINE_SDMMC1_D0                 PAL_LINE(GPIOC, 8U)
#define	LINE_SDMMC1_D1                 PAL_LINE(GPIOC, 9U)
#define	LINE_SDMMC1_D2                 PAL_LINE(GPIOC, 10U)
#define	LINE_SDMMC1_D3                 PAL_LINE(GPIOC, 11U)
#define	LINE_SDMMC1_CK                 PAL_LINE(GPIOC, 12U)
#define	LINE_SBUS_INV                  PAL_LINE(GPIOC, 13U)
#define	LINE__20608_DRDY               PAL_LINE(GPIOC, 14U)
#define	LINE__20608_CS                 PAL_LINE(GPIOC, 15U)

#define	LINE_CAN1_RX                   PAL_LINE(GPIOD, 0U)
#define	LINE_CAN1_TX                   PAL_LINE(GPIOD, 1U)
#define	LINE_SDMMC1_CMD                PAL_LINE(GPIOD, 2U)
#define	LINE_USART2_CTS                PAL_LINE(GPIOD, 3U)
#define	LINE_USART2_RTS                PAL_LINE(GPIOD, 4U)
#define	LINE_USART2_TX                 PAL_LINE(GPIOD, 5U)
#define	LINE_USART2_RX                 PAL_LINE(GPIOD, 6U)
#define	LINE_BARO_CS                   PAL_LINE(GPIOD, 7U)
#define	LINE_USART3_TX                 PAL_LINE(GPIOD, 8U)
#define	LINE_USART3_RX                 PAL_LINE(GPIOD, 9U)
#define	LINE_FRAM_CS                   PAL_LINE(GPIOD, 10U)
#define	LINE_USART3_CTS                PAL_LINE(GPIOD, 11U)
#define	LINE_USART3_RTS                PAL_LINE(GPIOD, 12U)
#define	LINE_SRV5_TIM4_CH2             PAL_LINE(GPIOD, 13U)
#define	LINE_SRV6_TIM4_CH3             PAL_LINE(GPIOD, 14U)
#define	LINE_MPU9250_DRDY              PAL_LINE(GPIOD, 15U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE__8266_GPIO0               PAL_LINE(GPIOE, 2U)
#define	LINE_VDD_3V3_SENS_EN           PAL_LINE(GPIOE, 3U)
#define	LINE_SPEKTRUM_POWER            PAL_LINE(GPIOE, 4U)
#define	LINE__8266_PD                  PAL_LINE(GPIOE, 5U)
#define	LINE__8266_RST                 PAL_LINE(GPIOE, 6U)
#define	LINE_UART7_RX                  PAL_LINE(GPIOE, 7U)
#define	LINE_UART7_TX                  PAL_LINE(GPIOE, 8U)
#define	LINE_SRV4_TIM1_CH1             PAL_LINE(GPIOE, 9U)
#define	LINE__8266_CTS                 PAL_LINE(GPIOE, 10U)
#define	LINE_SRV3_TIM1_CH2             PAL_LINE(GPIOE, 11U)
#define	LINE_HMC5983_DRDY              PAL_LINE(GPIOE, 12U)
#define	LINE_SRV2_TIM1_CH3             PAL_LINE(GPIOE, 13U)
#define	LINE_SRV1_TIM1_CH4             PAL_LINE(GPIOE, 14U)
#define	LINE_HMC_5983                  PAL_LINE(GPIOE, 15U)

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

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(USART4_TX) | \
					 PIN_MODE_ALTERNATE(USART4_RX) | \
					 PIN_MODE_ANALOG(VOLT_SENS) | \
					 PIN_MODE_ANALOG(CURRENT_SENS) | \
					 PIN_MODE_ANALOG(VDD_5V_SENS) | \
					 PIN_MODE_ALTERNATE(MPU9250_SCK) | \
					 PIN_MODE_ALTERNATE(MPU9250_MISO) | \
					 PIN_MODE_ALTERNATE(MPU9250_MOSI) | \
					 PIN_MODE_INPUT(_8266_RTS) | \
					 PIN_MODE_INPUT(USB_VBUS) | \
					 PIN_MODE_OUTPUT(FRSKY_INV) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DM) | \
					 PIN_MODE_ALTERNATE(OTG_FS_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_OUTPUT(ALARM))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(USART4_TX) | \
					 PIN_OTYPE_PUSHPULL(USART4_RX) | \
					 PIN_OTYPE_PUSHPULL(VOLT_SENS) | \
					 PIN_OTYPE_PUSHPULL(CURRENT_SENS) | \
					 PIN_OTYPE_PUSHPULL(VDD_5V_SENS) | \
					 PIN_OTYPE_PUSHPULL(MPU9250_SCK) | \
					 PIN_OTYPE_PUSHPULL(MPU9250_MISO) | \
					 PIN_OTYPE_PUSHPULL(MPU9250_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(_8266_RTS) | \
					 PIN_OTYPE_OPENDRAIN(USB_VBUS) | \
					 PIN_OTYPE_PUSHPULL(FRSKY_INV) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DM) | \
					 PIN_OTYPE_PUSHPULL(OTG_FS_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_PUSHPULL(ALARM))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(USART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART4_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(VOLT_SENS) | \
					 PIN_OSPEED_SPEED_VERYLOW(CURRENT_SENS) | \
					 PIN_OSPEED_SPEED_VERYLOW(VDD_5V_SENS) | \
					 PIN_OSPEED_SPEED_HIGH(MPU9250_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(MPU9250_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(MPU9250_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_RTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(USB_VBUS) | \
					 PIN_OSPEED_SPEED_VERYLOW(FRSKY_INV) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DM) | \
					 PIN_OSPEED_SPEED_HIGH(OTG_FS_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(ALARM))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(USART4_TX) | \
					 PIN_PUPDR_FLOATING(USART4_RX) | \
					 PIN_PUPDR_FLOATING(VOLT_SENS) | \
					 PIN_PUPDR_FLOATING(CURRENT_SENS) | \
					 PIN_PUPDR_FLOATING(VDD_5V_SENS) | \
					 PIN_PUPDR_FLOATING(MPU9250_SCK) | \
					 PIN_PUPDR_FLOATING(MPU9250_MISO) | \
					 PIN_PUPDR_FLOATING(MPU9250_MOSI) | \
					 PIN_PUPDR_PULLDOWN(_8266_RTS) | \
					 PIN_PUPDR_PULLDOWN(USB_VBUS) | \
					 PIN_PUPDR_FLOATING(FRSKY_INV) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DM) | \
					 PIN_PUPDR_FLOATING(OTG_FS_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_FLOATING(ALARM))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(USART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART4_RX) | \
					 PIN_ODR_LEVEL_LOW(VOLT_SENS) | \
					 PIN_ODR_LEVEL_LOW(CURRENT_SENS) | \
					 PIN_ODR_LEVEL_LOW(VDD_5V_SENS) | \
					 PIN_ODR_LEVEL_HIGH(MPU9250_SCK) | \
					 PIN_ODR_LEVEL_HIGH(MPU9250_MISO) | \
					 PIN_ODR_LEVEL_HIGH(MPU9250_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(_8266_RTS) | \
					 PIN_ODR_LEVEL_LOW(USB_VBUS) | \
					 PIN_ODR_LEVEL_HIGH(FRSKY_INV) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DM) | \
					 PIN_ODR_LEVEL_HIGH(OTG_FS_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(ALARM))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(USART4_TX, 8) | \
					 PIN_AFIO_AF(USART4_RX, 8) | \
					 PIN_AFIO_AF(VOLT_SENS, 0) | \
					 PIN_AFIO_AF(CURRENT_SENS, 0) | \
					 PIN_AFIO_AF(VDD_5V_SENS, 0) | \
					 PIN_AFIO_AF(MPU9250_SCK, 5) | \
					 PIN_AFIO_AF(MPU9250_MISO, 5) | \
					 PIN_AFIO_AF(MPU9250_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(_8266_RTS, 0) | \
					 PIN_AFIO_AF(USB_VBUS, 0) | \
					 PIN_AFIO_AF(FRSKY_INV, 0) | \
					 PIN_AFIO_AF(OTG_FS_DM, 10) | \
					 PIN_AFIO_AF(OTG_FS_DP, 10) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(ALARM, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(_RC_INPUT) | \
					 PIN_MODE_OUTPUT(LED_GREEN) | \
					 PIN_MODE_INPUT(BOOT1) | \
					 PIN_MODE_OUTPUT(LED_BLUE) | \
					 PIN_MODE_INPUT(_8266_GPIO2) | \
					 PIN_MODE_INPUT(VDD_BRICK_VALID) | \
					 PIN_MODE_ALTERNATE(USART1_TX) | \
					 PIN_MODE_ALTERNATE(USART1_RX) | \
					 PIN_MODE_ALTERNATE(I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(I2C1_SDA) | \
					 PIN_MODE_ALTERNATE(FRAM_SCK) | \
					 PIN_MODE_OUTPUT(LED_RED) | \
					 PIN_MODE_ALTERNATE(CAN2_RX) | \
					 PIN_MODE_ALTERNATE(CAN2_TX) | \
					 PIN_MODE_ALTERNATE(FRAM_MISO) | \
					 PIN_MODE_ALTERNATE(FRAM_MOSI))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_OPENDRAIN(_RC_INPUT) | \
					 PIN_OTYPE_PUSHPULL(LED_GREEN) | \
					 PIN_OTYPE_OPENDRAIN(BOOT1) | \
					 PIN_OTYPE_PUSHPULL(LED_BLUE) | \
					 PIN_OTYPE_OPENDRAIN(_8266_GPIO2) | \
					 PIN_OTYPE_OPENDRAIN(VDD_BRICK_VALID) | \
					 PIN_OTYPE_PUSHPULL(USART1_TX) | \
					 PIN_OTYPE_PUSHPULL(USART1_RX) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SDA) | \
					 PIN_OTYPE_PUSHPULL(FRAM_SCK) | \
					 PIN_OTYPE_PUSHPULL(LED_RED) | \
					 PIN_OTYPE_PUSHPULL(CAN2_RX) | \
					 PIN_OTYPE_PUSHPULL(CAN2_TX) | \
					 PIN_OTYPE_PUSHPULL(FRAM_MISO) | \
					 PIN_OTYPE_PUSHPULL(FRAM_MOSI))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(_RC_INPUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_GREEN) | \
					 PIN_OSPEED_SPEED_VERYLOW(BOOT1) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_BLUE) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_GPIO2) | \
					 PIN_OSPEED_SPEED_VERYLOW(VDD_BRICK_VALID) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(FRAM_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_RED) | \
					 PIN_OSPEED_SPEED_HIGH(CAN2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(CAN2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(FRAM_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(FRAM_MOSI))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLDOWN(_RC_INPUT) | \
					 PIN_PUPDR_FLOATING(LED_GREEN) | \
					 PIN_PUPDR_PULLDOWN(BOOT1) | \
					 PIN_PUPDR_FLOATING(LED_BLUE) | \
					 PIN_PUPDR_PULLDOWN(_8266_GPIO2) | \
					 PIN_PUPDR_PULLDOWN(VDD_BRICK_VALID) | \
					 PIN_PUPDR_FLOATING(USART1_TX) | \
					 PIN_PUPDR_FLOATING(USART1_RX) | \
					 PIN_PUPDR_PULLUP(I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(I2C1_SDA) | \
					 PIN_PUPDR_FLOATING(FRAM_SCK) | \
					 PIN_PUPDR_FLOATING(LED_RED) | \
					 PIN_PUPDR_FLOATING(CAN2_RX) | \
					 PIN_PUPDR_FLOATING(CAN2_TX) | \
					 PIN_PUPDR_FLOATING(FRAM_MISO) | \
					 PIN_PUPDR_FLOATING(FRAM_MOSI))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_HIGH(_RC_INPUT) | \
					 PIN_ODR_LEVEL_LOW(LED_GREEN) | \
					 PIN_ODR_LEVEL_HIGH(BOOT1) | \
					 PIN_ODR_LEVEL_LOW(LED_BLUE) | \
					 PIN_ODR_LEVEL_HIGH(_8266_GPIO2) | \
					 PIN_ODR_LEVEL_HIGH(VDD_BRICK_VALID) | \
					 PIN_ODR_LEVEL_HIGH(USART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(FRAM_SCK) | \
					 PIN_ODR_LEVEL_LOW(LED_RED) | \
					 PIN_ODR_LEVEL_HIGH(CAN2_RX) | \
					 PIN_ODR_LEVEL_HIGH(CAN2_TX) | \
					 PIN_ODR_LEVEL_HIGH(FRAM_MISO) | \
					 PIN_ODR_LEVEL_HIGH(FRAM_MOSI))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(_RC_INPUT, 0) | \
					 PIN_AFIO_AF(LED_GREEN, 0) | \
					 PIN_AFIO_AF(BOOT1, 0) | \
					 PIN_AFIO_AF(LED_BLUE, 0) | \
					 PIN_AFIO_AF(_8266_GPIO2, 0) | \
					 PIN_AFIO_AF(VDD_BRICK_VALID, 0) | \
					 PIN_AFIO_AF(USART1_TX, 7) | \
					 PIN_AFIO_AF(USART1_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(I2C1_SCL, 4) | \
					 PIN_AFIO_AF(I2C1_SDA, 4) | \
					 PIN_AFIO_AF(FRAM_SCK, 5) | \
					 PIN_AFIO_AF(LED_RED, 0) | \
					 PIN_AFIO_AF(CAN2_RX, 9) | \
					 PIN_AFIO_AF(CAN2_TX, 9) | \
					 PIN_AFIO_AF(FRAM_MISO, 5) | \
					 PIN_AFIO_AF(FRAM_MOSI, 5))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(VBUS_VALID) | \
					 PIN_MODE_INPUT(RSSI_IN) | \
					 PIN_MODE_OUTPUT(MPU9250_CS) | \
					 PIN_MODE_OUTPUT(LED_SAFETY) | \
					 PIN_MODE_INPUT(SAFETY_SW_IN) | \
					 PIN_MODE_OUTPUT(VDD_3V3_EN) | \
					 PIN_MODE_INPUT(RC_OUTPUT) | \
					 PIN_MODE_ALTERNATE(RC_INPUT) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D0) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D1) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D2) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D3) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CK) | \
					 PIN_MODE_OUTPUT(SBUS_INV) | \
					 PIN_MODE_INPUT(_20608_DRDY) | \
					 PIN_MODE_OUTPUT(_20608_CS))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_OPENDRAIN(VBUS_VALID) | \
					 PIN_OTYPE_OPENDRAIN(RSSI_IN) | \
					 PIN_OTYPE_PUSHPULL(MPU9250_CS) | \
					 PIN_OTYPE_PUSHPULL(LED_SAFETY) | \
					 PIN_OTYPE_OPENDRAIN(SAFETY_SW_IN) | \
					 PIN_OTYPE_PUSHPULL(VDD_3V3_EN) | \
					 PIN_OTYPE_OPENDRAIN(RC_OUTPUT) | \
					 PIN_OTYPE_PUSHPULL(RC_INPUT) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D0) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D1) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D2) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D3) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CK) | \
					 PIN_OTYPE_PUSHPULL(SBUS_INV) | \
					 PIN_OTYPE_OPENDRAIN(_20608_DRDY) | \
					 PIN_OTYPE_PUSHPULL(_20608_CS))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(VBUS_VALID) | \
					 PIN_OSPEED_SPEED_VERYLOW(RSSI_IN) | \
					 PIN_OSPEED_SPEED_HIGH(MPU9250_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_SAFETY) | \
					 PIN_OSPEED_SPEED_VERYLOW(SAFETY_SW_IN) | \
					 PIN_OSPEED_SPEED_VERYLOW(VDD_3V3_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(RC_OUTPUT) | \
					 PIN_OSPEED_SPEED_HIGH(RC_INPUT) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(SBUS_INV) | \
					 PIN_OSPEED_SPEED_VERYLOW(_20608_DRDY) | \
					 PIN_OSPEED_SPEED_HIGH(_20608_CS))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(VBUS_VALID) | \
					 PIN_PUPDR_PULLDOWN(RSSI_IN) | \
					 PIN_PUPDR_FLOATING(MPU9250_CS) | \
					 PIN_PUPDR_FLOATING(LED_SAFETY) | \
					 PIN_PUPDR_FLOATING(SAFETY_SW_IN) | \
					 PIN_PUPDR_FLOATING(VDD_3V3_EN) | \
					 PIN_PUPDR_PULLDOWN(RC_OUTPUT) | \
					 PIN_PUPDR_FLOATING(RC_INPUT) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D0) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D1) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D2) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D3) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CK) | \
					 PIN_PUPDR_FLOATING(SBUS_INV) | \
					 PIN_PUPDR_PULLDOWN(_20608_DRDY) | \
					 PIN_PUPDR_FLOATING(_20608_CS))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_HIGH(VBUS_VALID) | \
					 PIN_ODR_LEVEL_HIGH(RSSI_IN) | \
					 PIN_ODR_LEVEL_HIGH(MPU9250_CS) | \
					 PIN_ODR_LEVEL_LOW(LED_SAFETY) | \
					 PIN_ODR_LEVEL_LOW(SAFETY_SW_IN) | \
					 PIN_ODR_LEVEL_HIGH(VDD_3V3_EN) | \
					 PIN_ODR_LEVEL_HIGH(RC_OUTPUT) | \
					 PIN_ODR_LEVEL_HIGH(RC_INPUT) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CK) | \
					 PIN_ODR_LEVEL_HIGH(SBUS_INV) | \
					 PIN_ODR_LEVEL_HIGH(_20608_DRDY) | \
					 PIN_ODR_LEVEL_HIGH(_20608_CS))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(VBUS_VALID, 0) | \
					 PIN_AFIO_AF(RSSI_IN, 0) | \
					 PIN_AFIO_AF(MPU9250_CS, 0) | \
					 PIN_AFIO_AF(LED_SAFETY, 0) | \
					 PIN_AFIO_AF(SAFETY_SW_IN, 0) | \
					 PIN_AFIO_AF(VDD_3V3_EN, 0) | \
					 PIN_AFIO_AF(RC_OUTPUT, 0) | \
					 PIN_AFIO_AF(RC_INPUT, 8))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDMMC1_D0, 12) | \
					 PIN_AFIO_AF(SDMMC1_D1, 12) | \
					 PIN_AFIO_AF(SDMMC1_D2, 12) | \
					 PIN_AFIO_AF(SDMMC1_D3, 12) | \
					 PIN_AFIO_AF(SDMMC1_CK, 12) | \
					 PIN_AFIO_AF(SBUS_INV, 0) | \
					 PIN_AFIO_AF(_20608_DRDY, 0) | \
					 PIN_AFIO_AF(_20608_CS, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_ALTERNATE(CAN1_RX) | \
					 PIN_MODE_ALTERNATE(CAN1_TX) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CMD) | \
					 PIN_MODE_INPUT(USART2_CTS) | \
					 PIN_MODE_INPUT(USART2_RTS) | \
					 PIN_MODE_ALTERNATE(USART2_TX) | \
					 PIN_MODE_ALTERNATE(USART2_RX) | \
					 PIN_MODE_OUTPUT(BARO_CS) | \
					 PIN_MODE_ALTERNATE(USART3_TX) | \
					 PIN_MODE_ALTERNATE(USART3_RX) | \
					 PIN_MODE_OUTPUT(FRAM_CS) | \
					 PIN_MODE_INPUT(USART3_CTS) | \
					 PIN_MODE_INPUT(USART3_RTS) | \
					 PIN_MODE_INPUT(SRV5_TIM4_CH2) | \
					 PIN_MODE_INPUT(SRV6_TIM4_CH3) | \
					 PIN_MODE_INPUT(MPU9250_DRDY))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(CAN1_RX) | \
					 PIN_OTYPE_PUSHPULL(CAN1_TX) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CMD) | \
					 PIN_OTYPE_OPENDRAIN(USART2_CTS) | \
					 PIN_OTYPE_OPENDRAIN(USART2_RTS) | \
					 PIN_OTYPE_PUSHPULL(USART2_TX) | \
					 PIN_OTYPE_PUSHPULL(USART2_RX) | \
					 PIN_OTYPE_PUSHPULL(BARO_CS) | \
					 PIN_OTYPE_PUSHPULL(USART3_TX) | \
					 PIN_OTYPE_PUSHPULL(USART3_RX) | \
					 PIN_OTYPE_PUSHPULL(FRAM_CS) | \
					 PIN_OTYPE_OPENDRAIN(USART3_CTS) | \
					 PIN_OTYPE_OPENDRAIN(USART3_RTS) | \
					 PIN_OTYPE_OPENDRAIN(SRV5_TIM4_CH2) | \
					 PIN_OTYPE_OPENDRAIN(SRV6_TIM4_CH3) | \
					 PIN_OTYPE_OPENDRAIN(MPU9250_DRDY))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(CAN1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(CAN1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CMD) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART2_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART2_RTS) | \
					 PIN_OSPEED_SPEED_HIGH(USART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(BARO_CS) | \
					 PIN_OSPEED_SPEED_HIGH(USART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART3_RX) | \
					 PIN_OSPEED_SPEED_HIGH(FRAM_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART3_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART3_RTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV5_TIM4_CH2) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV6_TIM4_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPU9250_DRDY))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(CAN1_RX) | \
					 PIN_PUPDR_FLOATING(CAN1_TX) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CMD) | \
					 PIN_PUPDR_PULLDOWN(USART2_CTS) | \
					 PIN_PUPDR_PULLDOWN(USART2_RTS) | \
					 PIN_PUPDR_FLOATING(USART2_TX) | \
					 PIN_PUPDR_FLOATING(USART2_RX) | \
					 PIN_PUPDR_FLOATING(BARO_CS) | \
					 PIN_PUPDR_FLOATING(USART3_TX) | \
					 PIN_PUPDR_FLOATING(USART3_RX) | \
					 PIN_PUPDR_FLOATING(FRAM_CS) | \
					 PIN_PUPDR_PULLDOWN(USART3_CTS) | \
					 PIN_PUPDR_PULLDOWN(USART3_RTS) | \
					 PIN_PUPDR_PULLDOWN(SRV5_TIM4_CH2) | \
					 PIN_PUPDR_PULLDOWN(SRV6_TIM4_CH3) | \
					 PIN_PUPDR_FLOATING(MPU9250_DRDY))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(CAN1_RX) | \
					 PIN_ODR_LEVEL_HIGH(CAN1_TX) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CMD) | \
					 PIN_ODR_LEVEL_HIGH(USART2_CTS) | \
					 PIN_ODR_LEVEL_HIGH(USART2_RTS) | \
					 PIN_ODR_LEVEL_HIGH(USART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(BARO_CS) | \
					 PIN_ODR_LEVEL_HIGH(USART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART3_RX) | \
					 PIN_ODR_LEVEL_HIGH(FRAM_CS) | \
					 PIN_ODR_LEVEL_HIGH(USART3_CTS) | \
					 PIN_ODR_LEVEL_HIGH(USART3_RTS) | \
					 PIN_ODR_LEVEL_HIGH(SRV5_TIM4_CH2) | \
					 PIN_ODR_LEVEL_HIGH(SRV6_TIM4_CH3) | \
					 PIN_ODR_LEVEL_LOW(MPU9250_DRDY))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(CAN1_RX, 9) | \
					 PIN_AFIO_AF(CAN1_TX, 9) | \
					 PIN_AFIO_AF(SDMMC1_CMD, 12) | \
					 PIN_AFIO_AF(USART2_CTS, 0) | \
					 PIN_AFIO_AF(USART2_RTS, 0) | \
					 PIN_AFIO_AF(USART2_TX, 7) | \
					 PIN_AFIO_AF(USART2_RX, 7) | \
					 PIN_AFIO_AF(BARO_CS, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(USART3_TX, 7) | \
					 PIN_AFIO_AF(USART3_RX, 7) | \
					 PIN_AFIO_AF(FRAM_CS, 0) | \
					 PIN_AFIO_AF(USART3_CTS, 0) | \
					 PIN_AFIO_AF(USART3_RTS, 0) | \
					 PIN_AFIO_AF(SRV5_TIM4_CH2, 0) | \
					 PIN_AFIO_AF(SRV6_TIM4_CH3, 0) | \
					 PIN_AFIO_AF(MPU9250_DRDY, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(UART8_RX) | \
					 PIN_MODE_ALTERNATE(UART8_TX) | \
					 PIN_MODE_INPUT(_8266_GPIO0) | \
					 PIN_MODE_OUTPUT(VDD_3V3_SENS_EN) | \
					 PIN_MODE_OUTPUT(SPEKTRUM_POWER) | \
					 PIN_MODE_INPUT(_8266_PD) | \
					 PIN_MODE_INPUT(_8266_RST) | \
					 PIN_MODE_ALTERNATE(UART7_RX) | \
					 PIN_MODE_ALTERNATE(UART7_TX) | \
					 PIN_MODE_INPUT(SRV4_TIM1_CH1) | \
					 PIN_MODE_INPUT(_8266_CTS) | \
					 PIN_MODE_INPUT(SRV3_TIM1_CH2) | \
					 PIN_MODE_INPUT(HMC5983_DRDY) | \
					 PIN_MODE_INPUT(SRV2_TIM1_CH3) | \
					 PIN_MODE_INPUT(SRV1_TIM1_CH4) | \
					 PIN_MODE_OUTPUT(HMC_5983))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(UART8_TX) | \
					 PIN_OTYPE_OPENDRAIN(_8266_GPIO0) | \
					 PIN_OTYPE_PUSHPULL(VDD_3V3_SENS_EN) | \
					 PIN_OTYPE_PUSHPULL(SPEKTRUM_POWER) | \
					 PIN_OTYPE_OPENDRAIN(_8266_PD) | \
					 PIN_OTYPE_OPENDRAIN(_8266_RST) | \
					 PIN_OTYPE_PUSHPULL(UART7_RX) | \
					 PIN_OTYPE_PUSHPULL(UART7_TX) | \
					 PIN_OTYPE_OPENDRAIN(SRV4_TIM1_CH1) | \
					 PIN_OTYPE_OPENDRAIN(_8266_CTS) | \
					 PIN_OTYPE_OPENDRAIN(SRV3_TIM1_CH2) | \
					 PIN_OTYPE_OPENDRAIN(HMC5983_DRDY) | \
					 PIN_OTYPE_OPENDRAIN(SRV2_TIM1_CH3) | \
					 PIN_OTYPE_OPENDRAIN(SRV1_TIM1_CH4) | \
					 PIN_OTYPE_PUSHPULL(HMC_5983))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART8_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_GPIO0) | \
					 PIN_OSPEED_SPEED_VERYLOW(VDD_3V3_SENS_EN) | \
					 PIN_OSPEED_SPEED_VERYLOW(SPEKTRUM_POWER) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_PD) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_RST) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART7_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV4_TIM1_CH1) | \
					 PIN_OSPEED_SPEED_VERYLOW(_8266_CTS) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV3_TIM1_CH2) | \
					 PIN_OSPEED_SPEED_VERYLOW(HMC5983_DRDY) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV2_TIM1_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(SRV1_TIM1_CH4) | \
					 PIN_OSPEED_SPEED_HIGH(HMC_5983))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(UART8_RX) | \
					 PIN_PUPDR_FLOATING(UART8_TX) | \
					 PIN_PUPDR_PULLDOWN(_8266_GPIO0) | \
					 PIN_PUPDR_FLOATING(VDD_3V3_SENS_EN) | \
					 PIN_PUPDR_FLOATING(SPEKTRUM_POWER) | \
					 PIN_PUPDR_PULLDOWN(_8266_PD) | \
					 PIN_PUPDR_PULLDOWN(_8266_RST) | \
					 PIN_PUPDR_FLOATING(UART7_RX) | \
					 PIN_PUPDR_FLOATING(UART7_TX) | \
					 PIN_PUPDR_PULLDOWN(SRV4_TIM1_CH1) | \
					 PIN_PUPDR_PULLDOWN(_8266_CTS) | \
					 PIN_PUPDR_PULLDOWN(SRV3_TIM1_CH2) | \
					 PIN_PUPDR_FLOATING(HMC5983_DRDY) | \
					 PIN_PUPDR_PULLDOWN(SRV2_TIM1_CH3) | \
					 PIN_PUPDR_PULLDOWN(SRV1_TIM1_CH4) | \
					 PIN_PUPDR_FLOATING(HMC_5983))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART8_TX) | \
					 PIN_ODR_LEVEL_HIGH(_8266_GPIO0) | \
					 PIN_ODR_LEVEL_HIGH(VDD_3V3_SENS_EN) | \
					 PIN_ODR_LEVEL_HIGH(SPEKTRUM_POWER) | \
					 PIN_ODR_LEVEL_HIGH(_8266_PD) | \
					 PIN_ODR_LEVEL_HIGH(_8266_RST) | \
					 PIN_ODR_LEVEL_HIGH(UART7_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART7_TX) | \
					 PIN_ODR_LEVEL_HIGH(SRV4_TIM1_CH1) | \
					 PIN_ODR_LEVEL_HIGH(_8266_CTS) | \
					 PIN_ODR_LEVEL_HIGH(SRV3_TIM1_CH2) | \
					 PIN_ODR_LEVEL_LOW(HMC5983_DRDY) | \
					 PIN_ODR_LEVEL_HIGH(SRV2_TIM1_CH3) | \
					 PIN_ODR_LEVEL_HIGH(SRV1_TIM1_CH4) | \
					 PIN_ODR_LEVEL_HIGH(HMC_5983))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(UART8_RX, 8) | \
					 PIN_AFIO_AF(UART8_TX, 8) | \
					 PIN_AFIO_AF(_8266_GPIO0, 0) | \
					 PIN_AFIO_AF(VDD_3V3_SENS_EN, 0) | \
					 PIN_AFIO_AF(SPEKTRUM_POWER, 0) | \
					 PIN_AFIO_AF(_8266_PD, 0) | \
					 PIN_AFIO_AF(_8266_RST, 0) | \
					 PIN_AFIO_AF(UART7_RX, 8))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(UART7_TX, 8) | \
					 PIN_AFIO_AF(SRV4_TIM1_CH1, 0) | \
					 PIN_AFIO_AF(_8266_CTS, 0) | \
					 PIN_AFIO_AF(SRV3_TIM1_CH2, 0) | \
					 PIN_AFIO_AF(HMC5983_DRDY, 0) | \
					 PIN_AFIO_AF(SRV2_TIM1_CH3, 0) | \
					 PIN_AFIO_AF(SRV1_TIM1_CH4, 0) | \
					 PIN_AFIO_AF(HMC_5983, 0))

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

#define AF_USART4_TX                     8U
#define AF_LINE_USART4_TX                8U
#define AF_USART4_RX                     8U
#define AF_LINE_USART4_RX                8U
#define AF_MPU9250_SCK                   5U
#define AF_LINE_MPU9250_SCK              5U
#define AF_MPU9250_MISO                  5U
#define AF_LINE_MPU9250_MISO             5U
#define AF_MPU9250_MOSI                  5U
#define AF_LINE_MPU9250_MOSI             5U
#define AF_OTG_FS_DM                     10U
#define AF_LINE_OTG_FS_DM                10U
#define AF_OTG_FS_DP                     10U
#define AF_LINE_OTG_FS_DP                10U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_USART1_TX                     7U
#define AF_LINE_USART1_TX                7U
#define AF_USART1_RX                     7U
#define AF_LINE_USART1_RX                7U
#define AF_I2C1_SCL                      4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_I2C1_SDA                      4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_FRAM_SCK                      5U
#define AF_LINE_FRAM_SCK                 5U
#define AF_CAN2_RX                       9U
#define AF_LINE_CAN2_RX                  9U
#define AF_CAN2_TX                       9U
#define AF_LINE_CAN2_TX                  9U
#define AF_FRAM_MISO                     5U
#define AF_LINE_FRAM_MISO                5U
#define AF_FRAM_MOSI                     5U
#define AF_LINE_FRAM_MOSI                5U
#define AF_RC_INPUT                      8U
#define AF_LINE_RC_INPUT                 8U
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
#define AF_CAN1_RX                       9U
#define AF_LINE_CAN1_RX                  9U
#define AF_CAN1_TX                       9U
#define AF_LINE_CAN1_TX                  9U
#define AF_SDMMC1_CMD                    12U
#define AF_LINE_SDMMC1_CMD               12U
#define AF_USART2_TX                     7U
#define AF_LINE_USART2_TX                7U
#define AF_USART2_RX                     7U
#define AF_LINE_USART2_RX                7U
#define AF_USART3_TX                     7U
#define AF_LINE_USART3_TX                7U
#define AF_USART3_RX                     7U
#define AF_LINE_USART3_RX                7U
#define AF_UART8_RX                      8U
#define AF_LINE_UART8_RX                 8U
#define AF_UART8_TX                      8U
#define AF_LINE_UART8_TX                 8U
#define AF_UART7_RX                      8U
#define AF_LINE_UART7_RX                 8U
#define AF_UART7_TX                      8U
#define AF_LINE_UART7_TX                 8U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

