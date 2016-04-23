/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef CONFIG_OPA_FTD_1_0_H
#define CONFIG_OPA_FTD_1_0_H

#define BOARD_OPA_AP

/* OPA/FTD has a 12MHz external clock and 168MHz internal. */
#define EXT_CLK 12000000
#define AHB_CLK 168000000

/*
 * Power control
 */

/* VISION power */
#define VISION_PWR GPIOA
#define VISION_PWR_PIN GPIO1
#define VISION_PWR_ON gpio_set
#define VISION_PWR_OFF gpio_clear

/*
 * Onboard LEDs
 */

/* Status (red), on PA8 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOA
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear
#define LED_1_AFIO_REMAP ((void)0)


/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOB
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOB
#define UART3_GPIO_TX GPIO10

#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOC
#define UART4_GPIO_RX GPIO11
#define UART4_GPIO_PORT_TX GPIOC
#define UART4_GPIO_TX GPIO10

#define UART5_GPIO_AF GPIO_AF8
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2

/* SPI */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4

#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12

#define SPI_SELECT_SLAVE0_PORT GPIOA  // SD CARD (on spi1)
#define SPI_SELECT_SLAVE0_PIN GPIO4

#define SPI_SELECT_SLAVE1_PORT GPIOB  // IMU (on spi2)
#define SPI_SELECT_SLAVE1_PIN GPIO12

#define SPI_SELECT_SLAVE2_PORT GPIOC  // BARO (on spi2)
#define SPI_SELECT_SLAVE2_PIN GPIO13

/*
 * ADC
 */

/* BATT PC4/ADC14 */
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL 14
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO4
#endif

/* CURRENT PC3/ADC13 */
#if USE_ADC_2
#define AD1_2_CHANNEL 13
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO3
#endif

/* TEMP_MOTOR PC0/ADC10 */
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL 10
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO0
#endif

/* TEMP_BATT PC1/ADC11 */
#ifndef USE_ADC_4
#define USE_ADC_4 1
#endif
#if USE_ADC_4
#define AD1_4_CHANNEL 11
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO1
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

// Default: 10k / 2k2
// #define DefaultVoltageOfAdc(adc) (0.0045*adc)
// Opa: 31k6 / 4k7
#define DefaultVoltageOfAdc(adc) (0.0063*adc)

// ADC756-B050: +/-50Amp range bidirectional
// 40 mV / Ampere
// 1 ADC-count = 0,000805664 Volt, 0.040 V = 1 Ampere -> 1 Ampere = 49,6484887 ADC ticks: 1 ADC-tck = 20mA
// 5.0/3.3 from datasheet
#define DefaultMilliAmpereOfAdc(adc) (20.142*(adc-2048) * 5.0/3.3)

/* TEMP MOTOR: NTC with 2k fixed pull up */
// R0: 10k (@25C)
// T0: 25C -> 298.15K
// B: 3976 K
// a = (1/T0) - (1/B)*ln(R0) = 0.00103753243
// b = 1/B = 0.00025150905
// c = 0
#define TEMP_ADC_CHANNEL1 ADC_3
#define TEMP_ADC_CHANNEL1_TYPE NTC
#define TEMP_ADC_CHANNEL1_PU_R 2000
#define TEMP_ADC_CHANNEL1_A 0.00103753243
#define TEMP_ADC_CHANNEL1_B 0.00025150905
#define TEMP_ADC_CHANNEL1_C 0

/* TEMP BATT: NTC with 2k fixed pull up */
// R0: 10k (@25C)
// T0: 25C -> 298.15K
// B: 3976 K
// a = (1/T0) - (1/B)*ln(R0) = 0.00103753243
// b = 1/B = 0.00025150905
// c = 0
#define TEMP_ADC_CHANNEL2 ADC_4
#define TEMP_ADC_CHANNEL2_TYPE NTC
#define TEMP_ADC_CHANNEL2_PU_R 2000
#define TEMP_ADC_CHANNEL2_A 0.00103753243
#define TEMP_ADC_CHANNEL2_B 0.00025150905
#define TEMP_ADC_CHANNEL2_C 0

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_OPA_AP_1_0_H */
