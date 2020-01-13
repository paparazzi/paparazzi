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
#define BOARD_ST_CRAZYFLIE
#define BOARD_NAME  "Bitcraze Crazyflie 2.1"

/*
 * Board oscillators-related settings.
 * NOTE: LSE fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F405xx

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * IO pins assignments.
 */
#define	PA00                           0U
#define	PA00_PORT                      GPIOA
#define	MOTOR1                         1U
#define	MOTOR1_PORT                    GPIOA
#define	E_TX2                          2U
#define	E_TX2_PORT                     GPIOA
#define	E_RX2                          3U
#define	E_RX2_PORT                     GPIOA
#define	NRF_FLOW_CTRL                  4U
#define	NRF_FLOW_CTRL_PORT             GPIOA
#define	E_SCK                          5U
#define	E_SCK_PORT                     GPIOA
#define	E_MISO                         6U
#define	E_MISO_PORT                    GPIOA
#define	E_MOSI                         7U
#define	E_MOSI_PORT                    GPIOA
#define	IMU_SCL                        8U
#define	IMU_SCL_PORT                   GPIOA
#define	PA09                           9U
#define	PA09_PORT                      GPIOA
#define	USB_ID                         10U
#define	USB_ID_PORT                    GPIOA
#define	USB_DM                         11U
#define	USB_DM_PORT                    GPIOA
#define	USB_DP                         12U
#define	USB_DP_PORT                    GPIOA
#define	STM_SWIO                       13U
#define	STM_SWIO_PORT                  GPIOA
#define	STM_SWCLK                      14U
#define	STM_SWCLK_PORT                 GPIOA
#define	MOTOR3                         15U
#define	MOTOR3_PORT                    GPIOA

#define	PB00                           0U
#define	PB00_PORT                      GPIOB
#define	PB01                           1U
#define	PB01_PORT                      GPIOB
#define	BOOT1                          2U
#define	BOOT1_PORT                     GPIOB
#define	PB03                           3U
#define	PB03_PORT                      GPIOB
#define	E_CS1                          4U
#define	E_CS1_PORT                     GPIOB
#define	E_CS2                          5U
#define	E_CS2_PORT                     GPIOB
#define	E_SCL                          6U
#define	E_SCL_PORT                     GPIOB
#define	E_SDA                          7U
#define	E_SDA_PORT                     GPIOB
#define	E_CS3                          8U
#define	E_CS3_PORT                     GPIOB
#define	MOTOR4                         9U
#define	MOTOR4_PORT                    GPIOB
#define	PB10                           10U
#define	PB10_PORT                      GPIOB
#define	MOTOR2                         11U
#define	MOTOR2_PORT                    GPIOB
#define	PB12                           12U
#define	PB12_PORT                      GPIOB
#define	NRF_SWCLK                      13U
#define	NRF_SWCLK_PORT                 GPIOB
#define	PB14                           14U
#define	PB14_PORT                      GPIOB
#define	NRF_SWIO                       15U
#define	NRF_SWIO_PORT                  GPIOB

#define	LED_RED_L                      0U
#define	LED_RED_L_PORT                 GPIOC
#define	LED_GREEN_L                    1U
#define	LED_GREEN_L_PORT               GPIOC
#define	LED_GREEN_R                    2U
#define	LED_GREEN_R_PORT               GPIOC
#define	LED_RED_R                      3U
#define	LED_RED_R_PORT                 GPIOC
#define	PC04                           4U
#define	PC04_PORT                      GPIOC
#define	PC05                           5U
#define	PC05_PORT                      GPIOC
#define	NRF_TX                         6U
#define	NRF_TX_PORT                    GPIOC
#define	NRF_RX                         7U
#define	NRF_RX_PORT                    GPIOC
#define	PC08                           8U
#define	PC08_PORT                      GPIOC
#define	IMU_SDA                        9U
#define	IMU_SDA_PORT                   GPIOC
#define	E_TX1                          10U
#define	E_TX1_PORT                     GPIOC
#define	E_RX1                          11U
#define	E_RX1_PORT                     GPIOC
#define	E_CS0                          12U
#define	E_CS0_PORT                     GPIOC
#define	PC13                           13U
#define	PC13_PORT                      GPIOC
#define	PC14                           14U
#define	PC14_PORT                      GPIOC
#define	PC15                           15U
#define	PC15_PORT                      GPIOC

#define	PD00                           0U
#define	PD00_PORT                      GPIOD
#define	PD01                           1U
#define	PD01_PORT                      GPIOD
#define	LED_BLUE_L                     2U
#define	LED_BLUE_L_PORT                GPIOD
#define	PD03                           3U
#define	PD03_PORT                      GPIOD
#define	PD04                           4U
#define	PD04_PORT                      GPIOD
#define	PD05                           5U
#define	PD05_PORT                      GPIOD
#define	PD06                           6U
#define	PD06_PORT                      GPIOD
#define	PD07                           7U
#define	PD07_PORT                      GPIOD
#define	PD08                           8U
#define	PD08_PORT                      GPIOD
#define	PD09                           9U
#define	PD09_PORT                      GPIOD
#define	PD10                           10U
#define	PD10_PORT                      GPIOD
#define	PD11                           11U
#define	PD11_PORT                      GPIOD
#define	PD12                           12U
#define	PD12_PORT                      GPIOD
#define	PD13                           13U
#define	PD13_PORT                      GPIOD
#define	PD14                           14U
#define	PD14_PORT                      GPIOD
#define	PD15                           15U
#define	PD15_PORT                      GPIOD

#define	PE00                           0U
#define	PE00_PORT                      GPIOE
#define	PE01                           1U
#define	PE01_PORT                      GPIOE
#define	PE02                           2U
#define	PE02_PORT                      GPIOE
#define	PE03                           3U
#define	PE03_PORT                      GPIOE
#define	PE04                           4U
#define	PE04_PORT                      GPIOE
#define	PE05                           5U
#define	PE05_PORT                      GPIOE
#define	PE06                           6U
#define	PE06_PORT                      GPIOE
#define	PE07                           7U
#define	PE07_PORT                      GPIOE
#define	PE08                           8U
#define	PE08_PORT                      GPIOE
#define	PE09                           9U
#define	PE09_PORT                      GPIOE
#define	PE10                           10U
#define	PE10_PORT                      GPIOE
#define	PE11                           11U
#define	PE11_PORT                      GPIOE
#define	PE12                           12U
#define	PE12_PORT                      GPIOE
#define	PE13                           13U
#define	PE13_PORT                      GPIOE
#define	PE14                           14U
#define	PE14_PORT                      GPIOE
#define	PE15                           15U
#define	PE15_PORT                      GPIOE

#define	PF00                           0U
#define	PF00_PORT                      GPIOF
#define	PF01                           1U
#define	PF01_PORT                      GPIOF
#define	PF02                           2U
#define	PF02_PORT                      GPIOF
#define	PF03                           3U
#define	PF03_PORT                      GPIOF
#define	PF04                           4U
#define	PF04_PORT                      GPIOF
#define	PF05                           5U
#define	PF05_PORT                      GPIOF
#define	PF06                           6U
#define	PF06_PORT                      GPIOF
#define	PF07                           7U
#define	PF07_PORT                      GPIOF
#define	PF08                           8U
#define	PF08_PORT                      GPIOF
#define	PF09                           9U
#define	PF09_PORT                      GPIOF
#define	PF10                           10U
#define	PF10_PORT                      GPIOF
#define	PF11                           11U
#define	PF11_PORT                      GPIOF
#define	PF12                           12U
#define	PF12_PORT                      GPIOF
#define	PF13                           13U
#define	PF13_PORT                      GPIOF
#define	PF14                           14U
#define	PF14_PORT                      GPIOF
#define	PF15                           15U
#define	PF15_PORT                      GPIOF

#define	PG00                           0U
#define	PG00_PORT                      GPIOG
#define	PG01                           1U
#define	PG01_PORT                      GPIOG
#define	PG02                           2U
#define	PG02_PORT                      GPIOG
#define	PG03                           3U
#define	PG03_PORT                      GPIOG
#define	PG04                           4U
#define	PG04_PORT                      GPIOG
#define	PG05                           5U
#define	PG05_PORT                      GPIOG
#define	PG06                           6U
#define	PG06_PORT                      GPIOG
#define	PG07                           7U
#define	PG07_PORT                      GPIOG
#define	PG08                           8U
#define	PG08_PORT                      GPIOG
#define	PG09                           9U
#define	PG09_PORT                      GPIOG
#define	PG10                           10U
#define	PG10_PORT                      GPIOG
#define	PG11                           11U
#define	PG11_PORT                      GPIOG
#define	PG12                           12U
#define	PG12_PORT                      GPIOG
#define	PG13                           13U
#define	PG13_PORT                      GPIOG
#define	PG14                           14U
#define	PG14_PORT                      GPIOG
#define	PG15                           15U
#define	PG15_PORT                      GPIOG

#define	OSC_IN                         0U
#define	OSC_IN_PORT                    GPIOH
#define	OSC_OUT                        1U
#define	OSC_OUT_PORT                   GPIOH
#define	PH02                           2U
#define	PH02_PORT                      GPIOH
#define	PH03                           3U
#define	PH03_PORT                      GPIOH
#define	PH04                           4U
#define	PH04_PORT                      GPIOH
#define	PH05                           5U
#define	PH05_PORT                      GPIOH
#define	PH06                           6U
#define	PH06_PORT                      GPIOH
#define	PH07                           7U
#define	PH07_PORT                      GPIOH
#define	PH08                           8U
#define	PH08_PORT                      GPIOH
#define	PH09                           9U
#define	PH09_PORT                      GPIOH
#define	PH10                           10U
#define	PH10_PORT                      GPIOH
#define	PH11                           11U
#define	PH11_PORT                      GPIOH
#define	PH12                           12U
#define	PH12_PORT                      GPIOH
#define	PH13                           13U
#define	PH13_PORT                      GPIOH
#define	PH14                           14U
#define	PH14_PORT                      GPIOH
#define	PH15                           15U
#define	PH15_PORT                      GPIOH

#define	PI00                           0U
#define	PI00_PORT                      GPIOI
#define	PI01                           1U
#define	PI01_PORT                      GPIOI
#define	PI02                           2U
#define	PI02_PORT                      GPIOI
#define	PI03                           3U
#define	PI03_PORT                      GPIOI
#define	PI04                           4U
#define	PI04_PORT                      GPIOI
#define	PI05                           5U
#define	PI05_PORT                      GPIOI
#define	PI06                           6U
#define	PI06_PORT                      GPIOI
#define	PI07                           7U
#define	PI07_PORT                      GPIOI
#define	PI08                           8U
#define	PI08_PORT                      GPIOI
#define	PI09                           9U
#define	PI09_PORT                      GPIOI
#define	PI10                           10U
#define	PI10_PORT                      GPIOI
#define	PI11                           11U
#define	PI11_PORT                      GPIOI
#define	PI12                           12U
#define	PI12_PORT                      GPIOI
#define	PI13                           13U
#define	PI13_PORT                      GPIOI
#define	PI14                           14U
#define	PI14_PORT                      GPIOI
#define	PI15                           15U
#define	PI15_PORT                      GPIOI

#define	PJ00                           0U
#define	PJ00_PORT                      GPIOJ
#define	PJ01                           1U
#define	PJ01_PORT                      GPIOJ
#define	PJ02                           2U
#define	PJ02_PORT                      GPIOJ
#define	PJ03                           3U
#define	PJ03_PORT                      GPIOJ
#define	PJ04                           4U
#define	PJ04_PORT                      GPIOJ
#define	PJ05                           5U
#define	PJ05_PORT                      GPIOJ
#define	PJ06                           6U
#define	PJ06_PORT                      GPIOJ
#define	PJ07                           7U
#define	PJ07_PORT                      GPIOJ
#define	PJ08                           8U
#define	PJ08_PORT                      GPIOJ
#define	PJ09                           9U
#define	PJ09_PORT                      GPIOJ
#define	PJ10                           10U
#define	PJ10_PORT                      GPIOJ
#define	PJ11                           11U
#define	PJ11_PORT                      GPIOJ
#define	PJ12                           12U
#define	PJ12_PORT                      GPIOJ
#define	PJ13                           13U
#define	PJ13_PORT                      GPIOJ
#define	PJ14                           14U
#define	PJ14_PORT                      GPIOJ
#define	PJ15                           15U
#define	PJ15_PORT                      GPIOJ

#define	PK00                           0U
#define	PK00_PORT                      GPIOK
#define	PK01                           1U
#define	PK01_PORT                      GPIOK
#define	PK02                           2U
#define	PK02_PORT                      GPIOK
#define	PK03                           3U
#define	PK03_PORT                      GPIOK
#define	PK04                           4U
#define	PK04_PORT                      GPIOK
#define	PK05                           5U
#define	PK05_PORT                      GPIOK
#define	PK06                           6U
#define	PK06_PORT                      GPIOK
#define	PK07                           7U
#define	PK07_PORT                      GPIOK
#define	PK08                           8U
#define	PK08_PORT                      GPIOK
#define	PK09                           9U
#define	PK09_PORT                      GPIOK
#define	PK10                           10U
#define	PK10_PORT                      GPIOK
#define	PK11                           11U
#define	PK11_PORT                      GPIOK
#define	PK12                           12U
#define	PK12_PORT                      GPIOK
#define	PK13                           13U
#define	PK13_PORT                      GPIOK
#define	PK14                           14U
#define	PK14_PORT                      GPIOK
#define	PK15                           15U
#define	PK15_PORT                      GPIOK

/*
 * IO lines assignments.
 */
#define	LINE_MOTOR1                    PAL_LINE(GPIOA, 1U)
#define	LINE_E_TX2                     PAL_LINE(GPIOA, 2U)
#define	LINE_E_RX2                     PAL_LINE(GPIOA, 3U)
#define	LINE_NRF_FLOW_CTRL             PAL_LINE(GPIOA, 4U)
#define	LINE_E_SCK                     PAL_LINE(GPIOA, 5U)
#define	LINE_E_MISO                    PAL_LINE(GPIOA, 6U)
#define	LINE_E_MOSI                    PAL_LINE(GPIOA, 7U)
#define	LINE_IMU_SCL                   PAL_LINE(GPIOA, 8U)
#define	LINE_USB_ID                    PAL_LINE(GPIOA, 10U)
#define	LINE_USB_DM                    PAL_LINE(GPIOA, 11U)
#define	LINE_USB_DP                    PAL_LINE(GPIOA, 12U)
#define	LINE_STM_SWIO                  PAL_LINE(GPIOA, 13U)
#define	LINE_STM_SWCLK                 PAL_LINE(GPIOA, 14U)
#define	LINE_MOTOR3                    PAL_LINE(GPIOA, 15U)

#define	LINE_BOOT1                     PAL_LINE(GPIOB, 2U)
#define	LINE_E_CS1                     PAL_LINE(GPIOB, 4U)
#define	LINE_E_CS2                     PAL_LINE(GPIOB, 5U)
#define	LINE_E_SCL                     PAL_LINE(GPIOB, 6U)
#define	LINE_E_SDA                     PAL_LINE(GPIOB, 7U)
#define	LINE_E_CS3                     PAL_LINE(GPIOB, 8U)
#define	LINE_MOTOR4                    PAL_LINE(GPIOB, 9U)
#define	LINE_MOTOR2                    PAL_LINE(GPIOB, 11U)
#define	LINE_NRF_SWCLK                 PAL_LINE(GPIOB, 13U)
#define	LINE_NRF_SWIO                  PAL_LINE(GPIOB, 15U)

#define	LINE_LED_RED_L                 PAL_LINE(GPIOC, 0U)
#define	LINE_LED_GREEN_L               PAL_LINE(GPIOC, 1U)
#define	LINE_LED_GREEN_R               PAL_LINE(GPIOC, 2U)
#define	LINE_LED_RED_R                 PAL_LINE(GPIOC, 3U)
#define	LINE_NRF_TX                    PAL_LINE(GPIOC, 6U)
#define	LINE_NRF_RX                    PAL_LINE(GPIOC, 7U)
#define	LINE_IMU_SDA                   PAL_LINE(GPIOC, 9U)
#define	LINE_E_TX1                     PAL_LINE(GPIOC, 10U)
#define	LINE_E_RX1                     PAL_LINE(GPIOC, 11U)
#define	LINE_E_CS0                     PAL_LINE(GPIOC, 12U)

#define	LINE_LED_BLUE_L                PAL_LINE(GPIOD, 2U)

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

#define VAL_GPIOA_MODER                 (PIN_MODE_INPUT(PA00) | \
					 PIN_MODE_ALTERNATE(MOTOR1) | \
					 PIN_MODE_ALTERNATE(E_TX2) | \
					 PIN_MODE_ALTERNATE(E_RX2) | \
					 PIN_MODE_INPUT(NRF_FLOW_CTRL) | \
					 PIN_MODE_ALTERNATE(E_SCK) | \
					 PIN_MODE_ALTERNATE(E_MISO) | \
					 PIN_MODE_ALTERNATE(E_MOSI) | \
					 PIN_MODE_ALTERNATE(IMU_SCL) | \
					 PIN_MODE_INPUT(PA09) | \
					 PIN_MODE_ALTERNATE(USB_ID) | \
					 PIN_MODE_ALTERNATE(USB_DM) | \
					 PIN_MODE_ALTERNATE(USB_DP) | \
					 PIN_MODE_ALTERNATE(STM_SWIO) | \
					 PIN_MODE_ALTERNATE(STM_SWCLK) | \
					 PIN_MODE_ALTERNATE(MOTOR3))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00) | \
					 PIN_OTYPE_PUSHPULL(MOTOR1) | \
					 PIN_OTYPE_PUSHPULL(E_TX2) | \
					 PIN_OTYPE_PUSHPULL(E_RX2) | \
					 PIN_OTYPE_OPENDRAIN(NRF_FLOW_CTRL) | \
					 PIN_OTYPE_PUSHPULL(E_SCK) | \
					 PIN_OTYPE_PUSHPULL(E_MISO) | \
					 PIN_OTYPE_PUSHPULL(E_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(IMU_SCL) | \
					 PIN_OTYPE_PUSHPULL(PA09) | \
					 PIN_OTYPE_PUSHPULL(USB_ID) | \
					 PIN_OTYPE_PUSHPULL(USB_DM) | \
					 PIN_OTYPE_PUSHPULL(USB_DP) | \
					 PIN_OTYPE_PUSHPULL(STM_SWIO) | \
					 PIN_OTYPE_PUSHPULL(STM_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(MOTOR3))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PA00) | \
					 PIN_OSPEED_SPEED_HIGH(MOTOR1) | \
					 PIN_OSPEED_SPEED_HIGH(E_TX2) | \
					 PIN_OSPEED_SPEED_HIGH(E_RX2) | \
					 PIN_OSPEED_SPEED_VERYLOW(NRF_FLOW_CTRL) | \
					 PIN_OSPEED_SPEED_HIGH(E_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(E_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(E_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_SCL) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA09) | \
					 PIN_OSPEED_SPEED_HIGH(USB_ID) | \
					 PIN_OSPEED_SPEED_HIGH(USB_DM) | \
					 PIN_OSPEED_SPEED_HIGH(USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(STM_SWIO) | \
					 PIN_OSPEED_SPEED_HIGH(STM_SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(MOTOR3))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_PULLDOWN(PA00) | \
					 PIN_PUPDR_FLOATING(MOTOR1) | \
					 PIN_PUPDR_FLOATING(E_TX2) | \
					 PIN_PUPDR_FLOATING(E_RX2) | \
					 PIN_PUPDR_FLOATING(NRF_FLOW_CTRL) | \
					 PIN_PUPDR_FLOATING(E_SCK) | \
					 PIN_PUPDR_FLOATING(E_MISO) | \
					 PIN_PUPDR_FLOATING(E_MOSI) | \
					 PIN_PUPDR_PULLUP(IMU_SCL) | \
					 PIN_PUPDR_PULLDOWN(PA09) | \
					 PIN_PUPDR_FLOATING(USB_ID) | \
					 PIN_PUPDR_FLOATING(USB_DM) | \
					 PIN_PUPDR_FLOATING(USB_DP) | \
					 PIN_PUPDR_FLOATING(STM_SWIO) | \
					 PIN_PUPDR_FLOATING(STM_SWCLK) | \
					 PIN_PUPDR_FLOATING(MOTOR3))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(PA00) | \
					 PIN_ODR_LEVEL_LOW(MOTOR1) | \
					 PIN_ODR_LEVEL_HIGH(E_TX2) | \
					 PIN_ODR_LEVEL_HIGH(E_RX2) | \
					 PIN_ODR_LEVEL_LOW(NRF_FLOW_CTRL) | \
					 PIN_ODR_LEVEL_HIGH(E_SCK) | \
					 PIN_ODR_LEVEL_HIGH(E_MISO) | \
					 PIN_ODR_LEVEL_HIGH(E_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(IMU_SCL) | \
					 PIN_ODR_LEVEL_LOW(PA09) | \
					 PIN_ODR_LEVEL_HIGH(USB_ID) | \
					 PIN_ODR_LEVEL_HIGH(USB_DM) | \
					 PIN_ODR_LEVEL_HIGH(USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(STM_SWIO) | \
					 PIN_ODR_LEVEL_HIGH(STM_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(MOTOR3))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00, 0) | \
					 PIN_AFIO_AF(MOTOR1, 1) | \
					 PIN_AFIO_AF(E_TX2, 7) | \
					 PIN_AFIO_AF(E_RX2, 7) | \
					 PIN_AFIO_AF(NRF_FLOW_CTRL, 0) | \
					 PIN_AFIO_AF(E_SCK, 5) | \
					 PIN_AFIO_AF(E_MISO, 5) | \
					 PIN_AFIO_AF(E_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(IMU_SCL, 4) | \
					 PIN_AFIO_AF(PA09, 0) | \
					 PIN_AFIO_AF(USB_ID, 10) | \
					 PIN_AFIO_AF(USB_DM, 10) | \
					 PIN_AFIO_AF(USB_DP, 10) | \
					 PIN_AFIO_AF(STM_SWIO, 0) | \
					 PIN_AFIO_AF(STM_SWCLK, 0) | \
					 PIN_AFIO_AF(MOTOR3, 1))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(PB00) | \
					 PIN_MODE_INPUT(PB01) | \
					 PIN_MODE_INPUT(BOOT1) | \
					 PIN_MODE_INPUT(PB03) | \
					 PIN_MODE_INPUT(E_CS1) | \
					 PIN_MODE_INPUT(E_CS2) | \
					 PIN_MODE_ALTERNATE(E_SCL) | \
					 PIN_MODE_ALTERNATE(E_SDA) | \
					 PIN_MODE_INPUT(E_CS3) | \
					 PIN_MODE_ALTERNATE(MOTOR4) | \
					 PIN_MODE_INPUT(PB10) | \
					 PIN_MODE_ALTERNATE(MOTOR2) | \
					 PIN_MODE_INPUT(PB12) | \
					 PIN_MODE_INPUT(NRF_SWCLK) | \
					 PIN_MODE_INPUT(PB14) | \
					 PIN_MODE_INPUT(NRF_SWIO))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00) | \
					 PIN_OTYPE_PUSHPULL(PB01) | \
					 PIN_OTYPE_OPENDRAIN(BOOT1) | \
					 PIN_OTYPE_PUSHPULL(PB03) | \
					 PIN_OTYPE_OPENDRAIN(E_CS1) | \
					 PIN_OTYPE_OPENDRAIN(E_CS2) | \
					 PIN_OTYPE_OPENDRAIN(E_SCL) | \
					 PIN_OTYPE_OPENDRAIN(E_SDA) | \
					 PIN_OTYPE_OPENDRAIN(E_CS3) | \
					 PIN_OTYPE_PUSHPULL(MOTOR4) | \
					 PIN_OTYPE_PUSHPULL(PB10) | \
					 PIN_OTYPE_PUSHPULL(MOTOR2) | \
					 PIN_OTYPE_PUSHPULL(PB12) | \
					 PIN_OTYPE_OPENDRAIN(NRF_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PB14) | \
					 PIN_OTYPE_OPENDRAIN(NRF_SWIO))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PB00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB01) | \
					 PIN_OSPEED_SPEED_VERYLOW(BOOT1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB03) | \
					 PIN_OSPEED_SPEED_VERYLOW(E_CS1) | \
					 PIN_OSPEED_SPEED_VERYLOW(E_CS2) | \
					 PIN_OSPEED_SPEED_HIGH(E_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(E_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(E_CS3) | \
					 PIN_OSPEED_SPEED_HIGH(MOTOR4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB10) | \
					 PIN_OSPEED_SPEED_HIGH(MOTOR2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB12) | \
					 PIN_OSPEED_SPEED_VERYLOW(NRF_SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB14) | \
					 PIN_OSPEED_SPEED_VERYLOW(NRF_SWIO))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLDOWN(PB00) | \
					 PIN_PUPDR_PULLDOWN(PB01) | \
					 PIN_PUPDR_FLOATING(BOOT1) | \
					 PIN_PUPDR_PULLDOWN(PB03) | \
					 PIN_PUPDR_FLOATING(E_CS1) | \
					 PIN_PUPDR_FLOATING(E_CS2) | \
					 PIN_PUPDR_PULLUP(E_SCL) | \
					 PIN_PUPDR_PULLUP(E_SDA) | \
					 PIN_PUPDR_FLOATING(E_CS3) | \
					 PIN_PUPDR_FLOATING(MOTOR4) | \
					 PIN_PUPDR_PULLDOWN(PB10) | \
					 PIN_PUPDR_FLOATING(MOTOR2) | \
					 PIN_PUPDR_PULLDOWN(PB12) | \
					 PIN_PUPDR_FLOATING(NRF_SWCLK) | \
					 PIN_PUPDR_PULLDOWN(PB14) | \
					 PIN_PUPDR_FLOATING(NRF_SWIO))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00) | \
					 PIN_ODR_LEVEL_LOW(PB01) | \
					 PIN_ODR_LEVEL_LOW(BOOT1) | \
					 PIN_ODR_LEVEL_LOW(PB03) | \
					 PIN_ODR_LEVEL_LOW(E_CS1) | \
					 PIN_ODR_LEVEL_LOW(E_CS2) | \
					 PIN_ODR_LEVEL_HIGH(E_SCL) | \
					 PIN_ODR_LEVEL_HIGH(E_SDA) | \
					 PIN_ODR_LEVEL_LOW(E_CS3) | \
					 PIN_ODR_LEVEL_LOW(MOTOR4) | \
					 PIN_ODR_LEVEL_LOW(PB10) | \
					 PIN_ODR_LEVEL_LOW(MOTOR2) | \
					 PIN_ODR_LEVEL_LOW(PB12) | \
					 PIN_ODR_LEVEL_LOW(NRF_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PB14) | \
					 PIN_ODR_LEVEL_LOW(NRF_SWIO))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00, 0) | \
					 PIN_AFIO_AF(PB01, 0) | \
					 PIN_AFIO_AF(BOOT1, 0) | \
					 PIN_AFIO_AF(PB03, 0) | \
					 PIN_AFIO_AF(E_CS1, 0) | \
					 PIN_AFIO_AF(E_CS2, 0) | \
					 PIN_AFIO_AF(E_SCL, 4) | \
					 PIN_AFIO_AF(E_SDA, 4))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(E_CS3, 0) | \
					 PIN_AFIO_AF(MOTOR4, 2) | \
					 PIN_AFIO_AF(PB10, 0) | \
					 PIN_AFIO_AF(MOTOR2, 1) | \
					 PIN_AFIO_AF(PB12, 0) | \
					 PIN_AFIO_AF(NRF_SWCLK, 0) | \
					 PIN_AFIO_AF(PB14, 0) | \
					 PIN_AFIO_AF(NRF_SWIO, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_OUTPUT(LED_RED_L) | \
					 PIN_MODE_OUTPUT(LED_GREEN_L) | \
					 PIN_MODE_OUTPUT(LED_GREEN_R) | \
					 PIN_MODE_OUTPUT(LED_RED_R) | \
					 PIN_MODE_INPUT(PC04) | \
					 PIN_MODE_INPUT(PC05) | \
					 PIN_MODE_ALTERNATE(NRF_TX) | \
					 PIN_MODE_ALTERNATE(NRF_RX) | \
					 PIN_MODE_INPUT(PC08) | \
					 PIN_MODE_ALTERNATE(IMU_SDA) | \
					 PIN_MODE_ALTERNATE(E_TX1) | \
					 PIN_MODE_ALTERNATE(E_RX1) | \
					 PIN_MODE_INPUT(E_CS0) | \
					 PIN_MODE_INPUT(PC13) | \
					 PIN_MODE_INPUT(PC14) | \
					 PIN_MODE_INPUT(PC15))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(LED_RED_L) | \
					 PIN_OTYPE_PUSHPULL(LED_GREEN_L) | \
					 PIN_OTYPE_PUSHPULL(LED_GREEN_R) | \
					 PIN_OTYPE_PUSHPULL(LED_RED_R) | \
					 PIN_OTYPE_PUSHPULL(PC04) | \
					 PIN_OTYPE_PUSHPULL(PC05) | \
					 PIN_OTYPE_PUSHPULL(NRF_TX) | \
					 PIN_OTYPE_PUSHPULL(NRF_RX) | \
					 PIN_OTYPE_PUSHPULL(PC08) | \
					 PIN_OTYPE_OPENDRAIN(IMU_SDA) | \
					 PIN_OTYPE_PUSHPULL(E_TX1) | \
					 PIN_OTYPE_PUSHPULL(E_RX1) | \
					 PIN_OTYPE_OPENDRAIN(E_CS0) | \
					 PIN_OTYPE_PUSHPULL(PC13) | \
					 PIN_OTYPE_PUSHPULL(PC14) | \
					 PIN_OTYPE_PUSHPULL(PC15))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(LED_RED_L) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_GREEN_L) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_GREEN_R) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_RED_R) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_HIGH(NRF_TX) | \
					 PIN_OSPEED_SPEED_HIGH(NRF_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC08) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(E_TX1) | \
					 PIN_OSPEED_SPEED_HIGH(E_RX1) | \
					 PIN_OSPEED_SPEED_VERYLOW(E_CS0) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC15))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(LED_RED_L) | \
					 PIN_PUPDR_FLOATING(LED_GREEN_L) | \
					 PIN_PUPDR_FLOATING(LED_GREEN_R) | \
					 PIN_PUPDR_FLOATING(LED_RED_R) | \
					 PIN_PUPDR_PULLDOWN(PC04) | \
					 PIN_PUPDR_PULLDOWN(PC05) | \
					 PIN_PUPDR_FLOATING(NRF_TX) | \
					 PIN_PUPDR_FLOATING(NRF_RX) | \
					 PIN_PUPDR_PULLDOWN(PC08) | \
					 PIN_PUPDR_PULLUP(IMU_SDA) | \
					 PIN_PUPDR_FLOATING(E_TX1) | \
					 PIN_PUPDR_FLOATING(E_RX1) | \
					 PIN_PUPDR_FLOATING(E_CS0) | \
					 PIN_PUPDR_PULLDOWN(PC13) | \
					 PIN_PUPDR_PULLDOWN(PC14) | \
					 PIN_PUPDR_PULLDOWN(PC15))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(LED_RED_L) | \
					 PIN_ODR_LEVEL_LOW(LED_GREEN_L) | \
					 PIN_ODR_LEVEL_LOW(LED_GREEN_R) | \
					 PIN_ODR_LEVEL_LOW(LED_RED_R) | \
					 PIN_ODR_LEVEL_LOW(PC04) | \
					 PIN_ODR_LEVEL_LOW(PC05) | \
					 PIN_ODR_LEVEL_HIGH(NRF_TX) | \
					 PIN_ODR_LEVEL_HIGH(NRF_RX) | \
					 PIN_ODR_LEVEL_LOW(PC08) | \
					 PIN_ODR_LEVEL_HIGH(IMU_SDA) | \
					 PIN_ODR_LEVEL_HIGH(E_TX1) | \
					 PIN_ODR_LEVEL_HIGH(E_RX1) | \
					 PIN_ODR_LEVEL_LOW(E_CS0) | \
					 PIN_ODR_LEVEL_LOW(PC13) | \
					 PIN_ODR_LEVEL_LOW(PC14) | \
					 PIN_ODR_LEVEL_LOW(PC15))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(LED_RED_L, 0) | \
					 PIN_AFIO_AF(LED_GREEN_L, 0) | \
					 PIN_AFIO_AF(LED_GREEN_R, 0) | \
					 PIN_AFIO_AF(LED_RED_R, 0) | \
					 PIN_AFIO_AF(PC04, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(NRF_TX, 8) | \
					 PIN_AFIO_AF(NRF_RX, 8))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08, 0) | \
					 PIN_AFIO_AF(IMU_SDA, 4) | \
					 PIN_AFIO_AF(E_TX1, 7) | \
					 PIN_AFIO_AF(E_RX1, 7) | \
					 PIN_AFIO_AF(E_CS0, 0) | \
					 PIN_AFIO_AF(PC13, 0) | \
					 PIN_AFIO_AF(PC14, 0) | \
					 PIN_AFIO_AF(PC15, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(PD00) | \
					 PIN_MODE_INPUT(PD01) | \
					 PIN_MODE_OUTPUT(LED_BLUE_L) | \
					 PIN_MODE_INPUT(PD03) | \
					 PIN_MODE_INPUT(PD04) | \
					 PIN_MODE_INPUT(PD05) | \
					 PIN_MODE_INPUT(PD06) | \
					 PIN_MODE_INPUT(PD07) | \
					 PIN_MODE_INPUT(PD08) | \
					 PIN_MODE_INPUT(PD09) | \
					 PIN_MODE_INPUT(PD10) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_INPUT(PD12) | \
					 PIN_MODE_INPUT(PD13) | \
					 PIN_MODE_INPUT(PD14) | \
					 PIN_MODE_INPUT(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00) | \
					 PIN_OTYPE_PUSHPULL(PD01) | \
					 PIN_OTYPE_PUSHPULL(LED_BLUE_L) | \
					 PIN_OTYPE_PUSHPULL(PD03) | \
					 PIN_OTYPE_PUSHPULL(PD04) | \
					 PIN_OTYPE_PUSHPULL(PD05) | \
					 PIN_OTYPE_PUSHPULL(PD06) | \
					 PIN_OTYPE_PUSHPULL(PD07) | \
					 PIN_OTYPE_PUSHPULL(PD08) | \
					 PIN_OTYPE_PUSHPULL(PD09) | \
					 PIN_OTYPE_PUSHPULL(PD10) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_PUSHPULL(PD12) | \
					 PIN_OTYPE_PUSHPULL(PD13) | \
					 PIN_OTYPE_PUSHPULL(PD14) | \
					 PIN_OTYPE_PUSHPULL(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PD00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD01) | \
					 PIN_OSPEED_SPEED_VERYLOW(LED_BLUE_L) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_PULLDOWN(PD00) | \
					 PIN_PUPDR_PULLDOWN(PD01) | \
					 PIN_PUPDR_FLOATING(LED_BLUE_L) | \
					 PIN_PUPDR_PULLDOWN(PD03) | \
					 PIN_PUPDR_PULLDOWN(PD04) | \
					 PIN_PUPDR_PULLDOWN(PD05) | \
					 PIN_PUPDR_PULLDOWN(PD06) | \
					 PIN_PUPDR_PULLDOWN(PD07) | \
					 PIN_PUPDR_PULLDOWN(PD08) | \
					 PIN_PUPDR_PULLDOWN(PD09) | \
					 PIN_PUPDR_PULLDOWN(PD10) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_PULLDOWN(PD12) | \
					 PIN_PUPDR_PULLDOWN(PD13) | \
					 PIN_PUPDR_PULLDOWN(PD14) | \
					 PIN_PUPDR_PULLDOWN(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_LOW(PD00) | \
					 PIN_ODR_LEVEL_LOW(PD01) | \
					 PIN_ODR_LEVEL_LOW(LED_BLUE_L) | \
					 PIN_ODR_LEVEL_LOW(PD03) | \
					 PIN_ODR_LEVEL_LOW(PD04) | \
					 PIN_ODR_LEVEL_LOW(PD05) | \
					 PIN_ODR_LEVEL_LOW(PD06) | \
					 PIN_ODR_LEVEL_LOW(PD07) | \
					 PIN_ODR_LEVEL_LOW(PD08) | \
					 PIN_ODR_LEVEL_LOW(PD09) | \
					 PIN_ODR_LEVEL_LOW(PD10) | \
					 PIN_ODR_LEVEL_LOW(PD11) | \
					 PIN_ODR_LEVEL_LOW(PD12) | \
					 PIN_ODR_LEVEL_LOW(PD13) | \
					 PIN_ODR_LEVEL_LOW(PD14) | \
					 PIN_ODR_LEVEL_LOW(PD15))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00, 0) | \
					 PIN_AFIO_AF(PD01, 0) | \
					 PIN_AFIO_AF(LED_BLUE_L, 0) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(PD05, 0) | \
					 PIN_AFIO_AF(PD06, 0) | \
					 PIN_AFIO_AF(PD07, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08, 0) | \
					 PIN_AFIO_AF(PD09, 0) | \
					 PIN_AFIO_AF(PD10, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12, 0) | \
					 PIN_AFIO_AF(PD13, 0) | \
					 PIN_AFIO_AF(PD14, 0) | \
					 PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_INPUT(PE00) | \
					 PIN_MODE_INPUT(PE01) | \
					 PIN_MODE_INPUT(PE02) | \
					 PIN_MODE_INPUT(PE03) | \
					 PIN_MODE_INPUT(PE04) | \
					 PIN_MODE_INPUT(PE05) | \
					 PIN_MODE_INPUT(PE06) | \
					 PIN_MODE_INPUT(PE07) | \
					 PIN_MODE_INPUT(PE08) | \
					 PIN_MODE_INPUT(PE09) | \
					 PIN_MODE_INPUT(PE10) | \
					 PIN_MODE_INPUT(PE11) | \
					 PIN_MODE_INPUT(PE12) | \
					 PIN_MODE_INPUT(PE13) | \
					 PIN_MODE_INPUT(PE14) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00) | \
					 PIN_OTYPE_PUSHPULL(PE01) | \
					 PIN_OTYPE_PUSHPULL(PE02) | \
					 PIN_OTYPE_PUSHPULL(PE03) | \
					 PIN_OTYPE_PUSHPULL(PE04) | \
					 PIN_OTYPE_PUSHPULL(PE05) | \
					 PIN_OTYPE_PUSHPULL(PE06) | \
					 PIN_OTYPE_PUSHPULL(PE07) | \
					 PIN_OTYPE_PUSHPULL(PE08) | \
					 PIN_OTYPE_PUSHPULL(PE09) | \
					 PIN_OTYPE_PUSHPULL(PE10) | \
					 PIN_OTYPE_PUSHPULL(PE11) | \
					 PIN_OTYPE_PUSHPULL(PE12) | \
					 PIN_OTYPE_PUSHPULL(PE13) | \
					 PIN_OTYPE_PUSHPULL(PE14) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PE00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_PULLDOWN(PE00) | \
					 PIN_PUPDR_PULLDOWN(PE01) | \
					 PIN_PUPDR_PULLDOWN(PE02) | \
					 PIN_PUPDR_PULLDOWN(PE03) | \
					 PIN_PUPDR_PULLDOWN(PE04) | \
					 PIN_PUPDR_PULLDOWN(PE05) | \
					 PIN_PUPDR_PULLDOWN(PE06) | \
					 PIN_PUPDR_PULLDOWN(PE07) | \
					 PIN_PUPDR_PULLDOWN(PE08) | \
					 PIN_PUPDR_PULLDOWN(PE09) | \
					 PIN_PUPDR_PULLDOWN(PE10) | \
					 PIN_PUPDR_PULLDOWN(PE11) | \
					 PIN_PUPDR_PULLDOWN(PE12) | \
					 PIN_PUPDR_PULLDOWN(PE13) | \
					 PIN_PUPDR_PULLDOWN(PE14) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_LOW(PE00) | \
					 PIN_ODR_LEVEL_LOW(PE01) | \
					 PIN_ODR_LEVEL_LOW(PE02) | \
					 PIN_ODR_LEVEL_LOW(PE03) | \
					 PIN_ODR_LEVEL_LOW(PE04) | \
					 PIN_ODR_LEVEL_LOW(PE05) | \
					 PIN_ODR_LEVEL_LOW(PE06) | \
					 PIN_ODR_LEVEL_LOW(PE07) | \
					 PIN_ODR_LEVEL_LOW(PE08) | \
					 PIN_ODR_LEVEL_LOW(PE09) | \
					 PIN_ODR_LEVEL_LOW(PE10) | \
					 PIN_ODR_LEVEL_LOW(PE11) | \
					 PIN_ODR_LEVEL_LOW(PE12) | \
					 PIN_ODR_LEVEL_LOW(PE13) | \
					 PIN_ODR_LEVEL_LOW(PE14) | \
					 PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00, 0) | \
					 PIN_AFIO_AF(PE01, 0) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(PE04, 0) | \
					 PIN_AFIO_AF(PE05, 0) | \
					 PIN_AFIO_AF(PE06, 0) | \
					 PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08, 0) | \
					 PIN_AFIO_AF(PE09, 0) | \
					 PIN_AFIO_AF(PE10, 0) | \
					 PIN_AFIO_AF(PE11, 0) | \
					 PIN_AFIO_AF(PE12, 0) | \
					 PIN_AFIO_AF(PE13, 0) | \
					 PIN_AFIO_AF(PE14, 0) | \
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

#define AF_MOTOR1                        1U
#define AF_LINE_MOTOR1                   1U
#define AF_E_TX2                         7U
#define AF_LINE_E_TX2                    7U
#define AF_E_RX2                         7U
#define AF_LINE_E_RX2                    7U
#define AF_E_SCK                         5U
#define AF_LINE_E_SCK                    5U
#define AF_E_MISO                        5U
#define AF_LINE_E_MISO                   5U
#define AF_E_MOSI                        5U
#define AF_LINE_E_MOSI                   5U
#define AF_IMU_SCL                       4U
#define AF_LINE_IMU_SCL                  4U
#define AF_USB_ID                        10U
#define AF_LINE_USB_ID                   10U
#define AF_USB_DM                        10U
#define AF_LINE_USB_DM                   10U
#define AF_USB_DP                        10U
#define AF_LINE_USB_DP                   10U
#define AF_STM_SWIO                      0U
#define AF_LINE_STM_SWIO                 0U
#define AF_STM_SWCLK                     0U
#define AF_LINE_STM_SWCLK                0U
#define AF_MOTOR3                        1U
#define AF_LINE_MOTOR3                   1U
#define AF_E_SCL                         4U
#define AF_LINE_E_SCL                    4U
#define AF_E_SDA                         4U
#define AF_LINE_E_SDA                    4U
#define AF_MOTOR4                        2U
#define AF_LINE_MOTOR4                   2U
#define AF_MOTOR2                        1U
#define AF_LINE_MOTOR2                   1U
#define AF_NRF_TX                        8U
#define AF_LINE_NRF_TX                   8U
#define AF_NRF_RX                        8U
#define AF_LINE_NRF_RX                   8U
#define AF_IMU_SDA                       4U
#define AF_LINE_IMU_SDA                  4U
#define AF_E_TX1                         7U
#define AF_LINE_E_TX1                    7U
#define AF_E_RX1                         7U
#define AF_LINE_E_RX1                    7U
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

