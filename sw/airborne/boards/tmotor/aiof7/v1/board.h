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
#define BOARD_AIOF7TMOTOR
#define BOARD_NAME                  "AIO F7 TMOTOR"

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
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F722xx

/**
 * DSHOT
 */



#define DSHOT_TIM3_TELEMETRY_DEV NULL
#define DSHOT_TIM4_TELEMETRY_DEV NULL
#define DSHOT_TIM8_TELEMETRY_DEV NULL


#ifndef USE_DSHOT_TIM3
#define USE_DSHOT_TIM3 1 // SERVO1 SERVO2 SERVO3 SERVO4
#endif

#ifndef USE_DSHOT_TIM4
#define USE_DSHOT_TIM4 0 // SERVO5 SERVO6 SERVO7 
#endif


#ifndef USE_DSHOT_TIM8
#define USE_DSHOT_TIM8 0 // SERVO8
#endif



#if USE_DSHOT_TIM3 // SERVO1 SERVO2 SERVO3 SERVO4 on TIM3

#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO PAL_PORT(LINE_SERVO1)
#define DSHOT_SERVO_1_PIN PAL_PAD(LINE_SERVO1)
#define DSHOT_SERVO_1_AF AF_LINE_SERVO1
#define DSHOT_SERVO_1_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO1_TIM)
#define DSHOT_SERVO_1_CHANNEL SERVO1_TIM_CH

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO PAL_PORT(LINE_SERVO2)
#define DSHOT_SERVO_2_PIN PAL_PAD(LINE_SERVO2)
#define DSHOT_SERVO_2_AF AF_LINE_SERVO2
#define DSHOT_SERVO_2_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO2_TIM)
#define DSHOT_SERVO_2_CHANNEL SERVO2_TIM_CH

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO PAL_PORT(LINE_SERVO3)
#define DSHOT_SERVO_3_PIN PAL_PAD(LINE_SERVO3)
#define DSHOT_SERVO_3_AF AF_LINE_SERVO3
#define DSHOT_SERVO_3_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO3_TIM)
#define DSHOT_SERVO_3_CHANNEL SERVO3_TIM_CH

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO PAL_PORT(LINE_SERVO4)
#define DSHOT_SERVO_4_PIN PAL_PAD(LINE_SERVO4)
#define DSHOT_SERVO_4_AF AF_LINE_SERVO4
#define DSHOT_SERVO_4_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO4_TIM)
#define DSHOT_SERVO_4_CHANNEL SERVO4_TIM_CH


#define DSHOT_CONF_TIM3 1
#define DSHOT_CONF3_DEF { \
  .dma_stream = STM32_PWM3_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM3_UP_DMA_CHANNEL, \
  .pwmp = &PWMD3,                           \
  .tlm_sd = DSHOT_TIM3_TELEMETRY_DEV,       \
  .dma_buf = &dshot3DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif


#if USE_DSHOT_TIM4 // SERVO5 SERVO6 SERVO7 on TIM4

#define DSHOT_SERVO_5 5
#define DSHOT_SERVO_5_GPIO PAL_PORT(LINE_SERVO5)
#define DSHOT_SERVO_5_PIN PAL_PAD(LINE_SERVO5)
#define DSHOT_SERVO_5_AF AF_LINE_SERVO5
#define DSHOT_SERVO_5_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO5_TIM)
#define DSHOT_SERVO_5_CHANNEL SERVO5_TIM_CH

#define DSHOT_SERVO_6 6
#define DSHOT_SERVO_6_GPIO PAL_PORT(LINE_SERVO6)
#define DSHOT_SERVO_6_PIN PAL_PAD(LINE_SERVO6)
#define DSHOT_SERVO_6_AF AF_LINE_SERVO6
#define DSHOT_SERVO_6_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO6_TIM)
#define DSHOT_SERVO_6_CHANNEL SERVO6_TIM_CH

#define DSHOT_SERVO_7 7
#define DSHOT_SERVO_7_GPIO PAL_PORT(LINE_SERVO7)
#define DSHOT_SERVO_7_PIN PAL_PAD(LINE_SERVO7)
#define DSHOT_SERVO_7_AF AF_LINE_SERVO7
#define DSHOT_SERVO_7_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO7_TIM)
#define DSHOT_SERVO_7_CHANNEL SERVO7_TIM_CH

#define DSHOT_CONF_TIM4 1
#define DSHOT_CONF4_DEF { \
  .dma_stream = STM32_PWM4_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM4_UP_DMA_CHANNEL, \
  .pwmp = &PWMD4,                           \
  .tlm_sd = DSHOT_TIM4_TELEMETRY_DEV,       \
  .dma_buf = &dshot4DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif

#if USE_DSHOT_TIM8 // SERVO8

#define DSHOT_SERVO_8 8
#define DSHOT_SERVO_8_GPIO PAL_PORT(LINE_SERVO8)
#define DSHOT_SERVO_8_PIN PAL_PAD(LINE_SERVO8)
#define DSHOT_SERVO_8_AF AF_LINE_SERVO8
#define DSHOT_SERVO_8_DRIVER CONCAT_BOARD_PARAM(DSHOTD, SERVO8_TIM)
#define DSHOT_SERVO_8_CHANNEL SERVO8_TIM_CH


#define DSHOT_CONF_TIM8 1
#define DSHOT_CONF8_DEF { \
  .dma_stream = STM32_PWM8_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM8_UP_DMA_CHANNEL, \
  .pwmp = &PWMD8,                           \
  .tlm_sd = DSHOT_TIM8_TELEMETRY_DEV,       \
  .dma_buf = &dshot8DmaBuffer,              \
  .dcache_memory_in_use = false             \
}
#endif

/*
 * enable TIM1, TIM3, TIM4 by default
 */
#ifndef USE_PWM_TIM1
#define USE_PWM_TIM1 0
#endif

#ifndef USE_PWM_TIM3
#define USE_PWM_TIM3 1
#endif

#ifndef USE_PWM_TIM4
#define USE_PWM_TIM4 1
#endif

#ifndef USE_PWM_TIM8
#define USE_PWM_TIM8 1
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*10.91*adc)

/*
 * IO pins assignments.
 */
#define	PA00_UART4_TX                  0U
#define	PA01_UART4_RX                  1U
#define	PA02_UART2_TX                  2U
#define	PA03_UART2_RX                  3U
#define	PA04_SPI_SLAVE0                4U
#define	PA05_SPI1_CLK                  5U
#define	PA06_SPI1_MISO                 6U
#define	PA07_SPI1_MOSI                 7U
#define	PA08_AUX                       8U
#define	PA09_UART1_TX                  9U
#define	PA10_RC1                       10U
#define	PA11_OTG_FS_DM                 11U
#define	PA12_OTG_FS_DP                 12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15                           15U

#define	PB00_SERVO3                    0U
#define	PB01_SERVO4                    1U
#define	PB02_LED1                      2U
#define	PB03_SPI3_CLK                  3U
#define	PB04_SPI3_MISO                 4U
#define	PB05_SPI3_MOSI                 5U
#define	PB06_SERVO5                    6U
#define	PB07_SERVO7                    7U
#define	PB08_SERVO6                    8U
#define	PB09_AUX2                      9U
#define	PB10_I2C2_SCL                  10U
#define	PB11_I2C2_SDA                  11U
#define	PB12_SPI_SLAVE1                12U
#define	PB13_SPI2_CLK                  13U
#define	PB14_SPI2_MISO                 14U
#define	PB15_SPI2_MOSI                 15U

#define	PC00                           0U
#define	PC01_ADC1                      1U
#define	PC02_ADC2                      2U
#define	PC03                           3U
#define	PC04_GYRO_EXTI_1               4U
#define	PC05                           5U
#define	PC06_SERVO1                    6U
#define	PC07_SERVO2                    7U
#define	PC08_SPI_SLAVE2                8U
#define	PC09_SERVO8                    9U
#define	PC10_UART3_TX                  10U
#define	PC11_UART3_RX                  11U
#define	PC12_UART5_TX                  12U
#define	PC13_BEEPER                    13U
#define	PC14                           14U
#define	PC15_SPI_SLAVE3                15U

#define	PD00                           0U
#define	PD01                           1U
#define	PD02_UART5_RX                  2U
#define	PD03                           3U
#define	PD04                           4U
#define	PD05                           5U
#define	PD06                           6U
#define	PD07                           7U
#define	PD08                           8U
#define	PD09                           9U
#define	PD10                           10U
#define	PD11                           11U
#define	PD12                           12U
#define	PD13                           13U
#define	PD14                           14U
#define	PD15                           15U

#define	PE00                           0U
#define	PE01                           1U
#define	PE02                           2U
#define	PE03                           3U
#define	PE04                           4U
#define	PE05                           5U
#define	PE06                           6U
#define	PE07                           7U
#define	PE08                           8U
#define	PE09                           9U
#define	PE10                           10U
#define	PE11                           11U
#define	PE12                           12U
#define	PE13                           13U
#define	PE14                           14U
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
#define	LINE_UART4_TX                  PAL_LINE(GPIOA, 0U)
#define	LINE_UART4_RX                  PAL_LINE(GPIOA, 1U)
#define	LINE_UART2_TX                  PAL_LINE(GPIOA, 2U)
#define	LINE_UART2_RX                  PAL_LINE(GPIOA, 3U)
#define	LINE_SPI_SLAVE0                PAL_LINE(GPIOA, 4U)
#define	LINE_SPI1_CLK                  PAL_LINE(GPIOA, 5U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOA, 7U)
#define	LINE_AUX                       PAL_LINE(GPIOA, 8U)
#define	LINE_UART1_TX                  PAL_LINE(GPIOA, 9U)
#define	LINE_RC1                       PAL_LINE(GPIOA, 10U)
#define	LINE_OTG_FS_DM                 PAL_LINE(GPIOA, 11U)
#define	LINE_OTG_FS_DP                 PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)

#define	LINE_SERVO3                    PAL_LINE(GPIOB, 0U)
#define	LINE_SERVO4                    PAL_LINE(GPIOB, 1U)
#define	LINE_LED1                      PAL_LINE(GPIOB, 2U)
#define	LINE_SPI3_CLK                  PAL_LINE(GPIOB, 3U)
#define	LINE_SPI3_MISO                 PAL_LINE(GPIOB, 4U)
#define	LINE_SPI3_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_SERVO5                    PAL_LINE(GPIOB, 6U)
#define	LINE_SERVO7                    PAL_LINE(GPIOB, 7U)
#define	LINE_SERVO6                    PAL_LINE(GPIOB, 8U)
#define	LINE_AUX2                      PAL_LINE(GPIOB, 9U)
#define	LINE_I2C2_SCL                  PAL_LINE(GPIOB, 10U)
#define	LINE_I2C2_SDA                  PAL_LINE(GPIOB, 11U)
#define	LINE_SPI_SLAVE1                PAL_LINE(GPIOB, 12U)
#define	LINE_SPI2_CLK                  PAL_LINE(GPIOB, 13U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOB, 14U)
#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOB, 15U)

#define	LINE_ADC1                      PAL_LINE(GPIOC, 1U)
#define	LINE_ADC2                      PAL_LINE(GPIOC, 2U)
#define	LINE_GYRO_EXTI_1               PAL_LINE(GPIOC, 4U)
#define	LINE_SERVO1                    PAL_LINE(GPIOC, 6U)
#define	LINE_SERVO2                    PAL_LINE(GPIOC, 7U)
#define	LINE_SPI_SLAVE2                PAL_LINE(GPIOC, 8U)
#define	LINE_SERVO8                    PAL_LINE(GPIOC, 9U)
#define	LINE_UART3_TX                  PAL_LINE(GPIOC, 10U)
#define	LINE_UART3_RX                  PAL_LINE(GPIOC, 11U)
#define	LINE_UART5_TX                  PAL_LINE(GPIOC, 12U)
#define	LINE_BEEPER                    PAL_LINE(GPIOC, 13U)
#define	LINE_SPI_SLAVE3                PAL_LINE(GPIOC, 15U)

#define	LINE_UART5_RX                  PAL_LINE(GPIOD, 2U)

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

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(PA00_UART4_TX) | \
					 PIN_MODE_ALTERNATE(PA01_UART4_RX) | \
					 PIN_MODE_ALTERNATE(PA02_UART2_TX) | \
					 PIN_MODE_ALTERNATE(PA03_UART2_RX) | \
					 PIN_MODE_OUTPUT(PA04_SPI_SLAVE0) | \
					 PIN_MODE_ALTERNATE(PA05_SPI1_CLK) | \
					 PIN_MODE_ALTERNATE(PA06_SPI1_MISO) | \
					 PIN_MODE_ALTERNATE(PA07_SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(PA08_AUX) | \
					 PIN_MODE_ALTERNATE(PA09_UART1_TX) | \
					 PIN_MODE_ALTERNATE(PA10_RC1) | \
					 PIN_MODE_ALTERNATE(PA11_OTG_FS_DM) | \
					 PIN_MODE_ALTERNATE(PA12_OTG_FS_DP) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_INPUT(PA15))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(PA01_UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(PA02_UART2_TX) | \
					 PIN_OTYPE_PUSHPULL(PA03_UART2_RX) | \
					 PIN_OTYPE_PUSHPULL(PA04_SPI_SLAVE0) | \
					 PIN_OTYPE_PUSHPULL(PA05_SPI1_CLK) | \
					 PIN_OTYPE_PUSHPULL(PA06_SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(PA07_SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PA08_AUX) | \
					 PIN_OTYPE_PUSHPULL(PA09_UART1_TX) | \
					 PIN_OTYPE_PUSHPULL(PA10_RC1) | \
					 PIN_OTYPE_PUSHPULL(PA11_OTG_FS_DM) | \
					 PIN_OTYPE_PUSHPULL(PA12_OTG_FS_DP) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PA00_UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA01_UART4_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PA02_UART2_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA03_UART2_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PA04_SPI_SLAVE0) | \
					 PIN_OSPEED_SPEED_HIGH(PA05_SPI1_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(PA06_SPI1_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PA07_SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PA08_AUX) | \
					 PIN_OSPEED_SPEED_HIGH(PA09_UART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA10_RC1) | \
					 PIN_OSPEED_SPEED_HIGH(PA11_OTG_FS_DM) | \
					 PIN_OSPEED_SPEED_HIGH(PA12_OTG_FS_DP) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA15))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_UART4_TX) | \
					 PIN_PUPDR_FLOATING(PA01_UART4_RX) | \
					 PIN_PUPDR_FLOATING(PA02_UART2_TX) | \
					 PIN_PUPDR_FLOATING(PA03_UART2_RX) | \
					 PIN_PUPDR_FLOATING(PA04_SPI_SLAVE0) | \
					 PIN_PUPDR_FLOATING(PA05_SPI1_CLK) | \
					 PIN_PUPDR_FLOATING(PA06_SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(PA07_SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(PA08_AUX) | \
					 PIN_PUPDR_FLOATING(PA09_UART1_TX) | \
					 PIN_PUPDR_FLOATING(PA10_RC1) | \
					 PIN_PUPDR_FLOATING(PA11_OTG_FS_DM) | \
					 PIN_PUPDR_FLOATING(PA12_OTG_FS_DP) | \
					 PIN_PUPDR_FLOATING(PA13_SWDIO) | \
					 PIN_PUPDR_FLOATING(PA14_SWCLK) | \
					 PIN_PUPDR_PULLDOWN(PA15))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(PA00_UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA01_UART4_RX) | \
					 PIN_ODR_LEVEL_HIGH(PA02_UART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA03_UART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(PA04_SPI_SLAVE0) | \
					 PIN_ODR_LEVEL_HIGH(PA05_SPI1_CLK) | \
					 PIN_ODR_LEVEL_HIGH(PA06_SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PA07_SPI1_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PA08_AUX) | \
					 PIN_ODR_LEVEL_HIGH(PA09_UART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA10_RC1) | \
					 PIN_ODR_LEVEL_HIGH(PA11_OTG_FS_DM) | \
					 PIN_ODR_LEVEL_HIGH(PA12_OTG_FS_DP) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PA15))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_UART4_TX, 8) | \
					 PIN_AFIO_AF(PA01_UART4_RX, 8) | \
					 PIN_AFIO_AF(PA02_UART2_TX, 7) | \
					 PIN_AFIO_AF(PA03_UART2_RX, 7) | \
					 PIN_AFIO_AF(PA04_SPI_SLAVE0, 0) | \
					 PIN_AFIO_AF(PA05_SPI1_CLK, 5) | \
					 PIN_AFIO_AF(PA06_SPI1_MISO, 5) | \
					 PIN_AFIO_AF(PA07_SPI1_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_AUX, 1) | \
					 PIN_AFIO_AF(PA09_UART1_TX, 7) | \
					 PIN_AFIO_AF(PA10_RC1, 7) | \
					 PIN_AFIO_AF(PA11_OTG_FS_DM, 10) | \
					 PIN_AFIO_AF(PA12_OTG_FS_DP, 10) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_ALTERNATE(PB00_SERVO3) | \
					 PIN_MODE_ALTERNATE(PB01_SERVO4) | \
					 PIN_MODE_OUTPUT(PB02_LED1) | \
					 PIN_MODE_ALTERNATE(PB03_SPI3_CLK) | \
					 PIN_MODE_ALTERNATE(PB04_SPI3_MISO) | \
					 PIN_MODE_ALTERNATE(PB05_SPI3_MOSI) | \
					 PIN_MODE_ALTERNATE(PB06_SERVO5) | \
					 PIN_MODE_ALTERNATE(PB07_SERVO7) | \
					 PIN_MODE_ALTERNATE(PB08_SERVO6) | \
					 PIN_MODE_ALTERNATE(PB09_AUX2) | \
					 PIN_MODE_ALTERNATE(PB10_I2C2_SCL) | \
					 PIN_MODE_ALTERNATE(PB11_I2C2_SDA) | \
					 PIN_MODE_OUTPUT(PB12_SPI_SLAVE1) | \
					 PIN_MODE_ALTERNATE(PB13_SPI2_CLK) | \
					 PIN_MODE_ALTERNATE(PB14_SPI2_MISO) | \
					 PIN_MODE_ALTERNATE(PB15_SPI2_MOSI))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00_SERVO3) | \
					 PIN_OTYPE_PUSHPULL(PB01_SERVO4) | \
					 PIN_OTYPE_PUSHPULL(PB02_LED1) | \
					 PIN_OTYPE_PUSHPULL(PB03_SPI3_CLK) | \
					 PIN_OTYPE_PUSHPULL(PB04_SPI3_MISO) | \
					 PIN_OTYPE_PUSHPULL(PB05_SPI3_MOSI) | \
					 PIN_OTYPE_PUSHPULL(PB06_SERVO5) | \
					 PIN_OTYPE_PUSHPULL(PB07_SERVO7) | \
					 PIN_OTYPE_PUSHPULL(PB08_SERVO6) | \
					 PIN_OTYPE_PUSHPULL(PB09_AUX2) | \
					 PIN_OTYPE_OPENDRAIN(PB10_I2C2_SCL) | \
					 PIN_OTYPE_OPENDRAIN(PB11_I2C2_SDA) | \
					 PIN_OTYPE_PUSHPULL(PB12_SPI_SLAVE1) | \
					 PIN_OTYPE_PUSHPULL(PB13_SPI2_CLK) | \
					 PIN_OTYPE_PUSHPULL(PB14_SPI2_MISO) | \
					 PIN_OTYPE_PUSHPULL(PB15_SPI2_MOSI))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PB00_SERVO3) | \
					 PIN_OSPEED_SPEED_HIGH(PB01_SERVO4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02_LED1) | \
					 PIN_OSPEED_SPEED_HIGH(PB03_SPI3_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(PB04_SPI3_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PB05_SPI3_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_SERVO5) | \
					 PIN_OSPEED_SPEED_HIGH(PB07_SERVO7) | \
					 PIN_OSPEED_SPEED_HIGH(PB08_SERVO6) | \
					 PIN_OSPEED_SPEED_HIGH(PB09_AUX2) | \
					 PIN_OSPEED_SPEED_HIGH(PB10_I2C2_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(PB11_I2C2_SDA) | \
					 PIN_OSPEED_SPEED_HIGH(PB12_SPI_SLAVE1) | \
					 PIN_OSPEED_SPEED_HIGH(PB13_SPI2_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(PB14_SPI2_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PB15_SPI2_MOSI))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PB00_SERVO3) | \
					 PIN_PUPDR_FLOATING(PB01_SERVO4) | \
					 PIN_PUPDR_FLOATING(PB02_LED1) | \
					 PIN_PUPDR_FLOATING(PB03_SPI3_CLK) | \
					 PIN_PUPDR_FLOATING(PB04_SPI3_MISO) | \
					 PIN_PUPDR_FLOATING(PB05_SPI3_MOSI) | \
					 PIN_PUPDR_FLOATING(PB06_SERVO5) | \
					 PIN_PUPDR_FLOATING(PB07_SERVO7) | \
					 PIN_PUPDR_FLOATING(PB08_SERVO6) | \
					 PIN_PUPDR_FLOATING(PB09_AUX2) | \
					 PIN_PUPDR_PULLUP(PB10_I2C2_SCL) | \
					 PIN_PUPDR_PULLUP(PB11_I2C2_SDA) | \
					 PIN_PUPDR_FLOATING(PB12_SPI_SLAVE1) | \
					 PIN_PUPDR_FLOATING(PB13_SPI2_CLK) | \
					 PIN_PUPDR_FLOATING(PB14_SPI2_MISO) | \
					 PIN_PUPDR_FLOATING(PB15_SPI2_MOSI))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00_SERVO3) | \
					 PIN_ODR_LEVEL_LOW(PB01_SERVO4) | \
					 PIN_ODR_LEVEL_LOW(PB02_LED1) | \
					 PIN_ODR_LEVEL_HIGH(PB03_SPI3_CLK) | \
					 PIN_ODR_LEVEL_HIGH(PB04_SPI3_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PB05_SPI3_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PB06_SERVO5) | \
					 PIN_ODR_LEVEL_LOW(PB07_SERVO7) | \
					 PIN_ODR_LEVEL_LOW(PB08_SERVO6) | \
					 PIN_ODR_LEVEL_LOW(PB09_AUX2) | \
					 PIN_ODR_LEVEL_HIGH(PB10_I2C2_SCL) | \
					 PIN_ODR_LEVEL_HIGH(PB11_I2C2_SDA) | \
					 PIN_ODR_LEVEL_HIGH(PB12_SPI_SLAVE1) | \
					 PIN_ODR_LEVEL_HIGH(PB13_SPI2_CLK) | \
					 PIN_ODR_LEVEL_HIGH(PB14_SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PB15_SPI2_MOSI))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_SERVO3, 2) | \
					 PIN_AFIO_AF(PB01_SERVO4, 2) | \
					 PIN_AFIO_AF(PB02_LED1, 0) | \
					 PIN_AFIO_AF(PB03_SPI3_CLK, 6) | \
					 PIN_AFIO_AF(PB04_SPI3_MISO, 6) | \
					 PIN_AFIO_AF(PB05_SPI3_MOSI, 6) | \
					 PIN_AFIO_AF(PB06_SERVO5, 2) | \
					 PIN_AFIO_AF(PB07_SERVO7, 2))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08_SERVO6, 2) | \
					 PIN_AFIO_AF(PB09_AUX2, 3) | \
					 PIN_AFIO_AF(PB10_I2C2_SCL, 4) | \
					 PIN_AFIO_AF(PB11_I2C2_SDA, 4) | \
					 PIN_AFIO_AF(PB12_SPI_SLAVE1, 0) | \
					 PIN_AFIO_AF(PB13_SPI2_CLK, 5) | \
					 PIN_AFIO_AF(PB14_SPI2_MISO, 5) | \
					 PIN_AFIO_AF(PB15_SPI2_MOSI, 5))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(PC00) | \
					 PIN_MODE_ANALOG(PC01_ADC1) | \
					 PIN_MODE_ANALOG(PC02_ADC2) | \
					 PIN_MODE_INPUT(PC03) | \
					 PIN_MODE_INPUT(PC04_GYRO_EXTI_1) | \
					 PIN_MODE_INPUT(PC05) | \
					 PIN_MODE_ALTERNATE(PC06_SERVO1) | \
					 PIN_MODE_ALTERNATE(PC07_SERVO2) | \
					 PIN_MODE_OUTPUT(PC08_SPI_SLAVE2) | \
					 PIN_MODE_ALTERNATE(PC09_SERVO8) | \
					 PIN_MODE_ALTERNATE(PC10_UART3_TX) | \
					 PIN_MODE_ALTERNATE(PC11_UART3_RX) | \
					 PIN_MODE_ALTERNATE(PC12_UART5_TX) | \
					 PIN_MODE_OUTPUT(PC13_BEEPER) | \
					 PIN_MODE_INPUT(PC14) | \
					 PIN_MODE_OUTPUT(PC15_SPI_SLAVE3))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(PC00) | \
					 PIN_OTYPE_PUSHPULL(PC01_ADC1) | \
					 PIN_OTYPE_PUSHPULL(PC02_ADC2) | \
					 PIN_OTYPE_PUSHPULL(PC03) | \
					 PIN_OTYPE_OPENDRAIN(PC04_GYRO_EXTI_1) | \
					 PIN_OTYPE_PUSHPULL(PC05) | \
					 PIN_OTYPE_PUSHPULL(PC06_SERVO1) | \
					 PIN_OTYPE_PUSHPULL(PC07_SERVO2) | \
					 PIN_OTYPE_PUSHPULL(PC08_SPI_SLAVE2) | \
					 PIN_OTYPE_PUSHPULL(PC09_SERVO8) | \
					 PIN_OTYPE_PUSHPULL(PC10_UART3_TX) | \
					 PIN_OTYPE_PUSHPULL(PC11_UART3_RX) | \
					 PIN_OTYPE_PUSHPULL(PC12_UART5_TX) | \
					 PIN_OTYPE_PUSHPULL(PC13_BEEPER) | \
					 PIN_OTYPE_PUSHPULL(PC14) | \
					 PIN_OTYPE_PUSHPULL(PC15_SPI_SLAVE3))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01_ADC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02_ADC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04_GYRO_EXTI_1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_HIGH(PC06_SERVO1) | \
					 PIN_OSPEED_SPEED_HIGH(PC07_SERVO2) | \
					 PIN_OSPEED_SPEED_HIGH(PC08_SPI_SLAVE2) | \
					 PIN_OSPEED_SPEED_HIGH(PC09_SERVO8) | \
					 PIN_OSPEED_SPEED_HIGH(PC10_UART3_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PC11_UART3_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PC12_UART5_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PC13_BEEPER) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC14) | \
					 PIN_OSPEED_SPEED_HIGH(PC15_SPI_SLAVE3))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(PC00) | \
					 PIN_PUPDR_FLOATING(PC01_ADC1) | \
					 PIN_PUPDR_FLOATING(PC02_ADC2) | \
					 PIN_PUPDR_PULLDOWN(PC03) | \
					 PIN_PUPDR_PULLUP(PC04_GYRO_EXTI_1) | \
					 PIN_PUPDR_PULLDOWN(PC05) | \
					 PIN_PUPDR_FLOATING(PC06_SERVO1) | \
					 PIN_PUPDR_FLOATING(PC07_SERVO2) | \
					 PIN_PUPDR_FLOATING(PC08_SPI_SLAVE2) | \
					 PIN_PUPDR_FLOATING(PC09_SERVO8) | \
					 PIN_PUPDR_FLOATING(PC10_UART3_TX) | \
					 PIN_PUPDR_FLOATING(PC11_UART3_RX) | \
					 PIN_PUPDR_FLOATING(PC12_UART5_TX) | \
					 PIN_PUPDR_FLOATING(PC13_BEEPER) | \
					 PIN_PUPDR_PULLDOWN(PC14) | \
					 PIN_PUPDR_FLOATING(PC15_SPI_SLAVE3))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(PC00) | \
					 PIN_ODR_LEVEL_LOW(PC01_ADC1) | \
					 PIN_ODR_LEVEL_LOW(PC02_ADC2) | \
					 PIN_ODR_LEVEL_LOW(PC03) | \
					 PIN_ODR_LEVEL_LOW(PC04_GYRO_EXTI_1) | \
					 PIN_ODR_LEVEL_LOW(PC05) | \
					 PIN_ODR_LEVEL_LOW(PC06_SERVO1) | \
					 PIN_ODR_LEVEL_LOW(PC07_SERVO2) | \
					 PIN_ODR_LEVEL_HIGH(PC08_SPI_SLAVE2) | \
					 PIN_ODR_LEVEL_LOW(PC09_SERVO8) | \
					 PIN_ODR_LEVEL_HIGH(PC10_UART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(PC11_UART3_RX) | \
					 PIN_ODR_LEVEL_HIGH(PC12_UART5_TX) | \
					 PIN_ODR_LEVEL_LOW(PC13_BEEPER) | \
					 PIN_ODR_LEVEL_LOW(PC14) | \
					 PIN_ODR_LEVEL_HIGH(PC15_SPI_SLAVE3))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00, 0) | \
					 PIN_AFIO_AF(PC01_ADC1, 0) | \
					 PIN_AFIO_AF(PC02_ADC2, 0) | \
					 PIN_AFIO_AF(PC03, 0) | \
					 PIN_AFIO_AF(PC04_GYRO_EXTI_1, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(PC06_SERVO1, 2) | \
					 PIN_AFIO_AF(PC07_SERVO2, 2))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08_SPI_SLAVE2, 0) | \
					 PIN_AFIO_AF(PC09_SERVO8, 3) | \
					 PIN_AFIO_AF(PC10_UART3_TX, 7) | \
					 PIN_AFIO_AF(PC11_UART3_RX, 7) | \
					 PIN_AFIO_AF(PC12_UART5_TX, 8) | \
					 PIN_AFIO_AF(PC13_BEEPER, 0) | \
					 PIN_AFIO_AF(PC14, 0) | \
					 PIN_AFIO_AF(PC15_SPI_SLAVE3, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(PD00) | \
					 PIN_MODE_INPUT(PD01) | \
					 PIN_MODE_ALTERNATE(PD02_UART5_RX) | \
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
					 PIN_OTYPE_PUSHPULL(PD02_UART5_RX) | \
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
					 PIN_OSPEED_SPEED_HIGH(PD02_UART5_RX) | \
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
					 PIN_PUPDR_FLOATING(PD02_UART5_RX) | \
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
					 PIN_ODR_LEVEL_HIGH(PD02_UART5_RX) | \
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
					 PIN_AFIO_AF(PD02_UART5_RX, 8) | \
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

#define VAL_GPIOH_MODER                 (PIN_MODE_INPUT(PH00_OSC_IN) | \
					 PIN_MODE_INPUT(PH01_OSC_OUT) | \
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

#define AF_PA00_UART4_TX                 8U
#define AF_LINE_UART4_TX                 8U
#define AF_PA01_UART4_RX                 8U
#define AF_LINE_UART4_RX                 8U
#define AF_PA02_UART2_TX                 7U
#define AF_LINE_UART2_TX                 7U
#define AF_PA03_UART2_RX                 7U
#define AF_LINE_UART2_RX                 7U
#define AF_PA05_SPI1_CLK                 5U
#define AF_LINE_SPI1_CLK                 5U
#define AF_PA06_SPI1_MISO                5U
#define AF_LINE_SPI1_MISO                5U
#define AF_PA07_SPI1_MOSI                5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_PA08_AUX                      1U
#define AF_LINE_AUX                      1U
#define AF_PA09_UART1_TX                 7U
#define AF_LINE_UART1_TX                 7U
#define AF_PA10_RC1                      7U
#define AF_LINE_RC1                      7U
#define AF_PA11_OTG_FS_DM                10U
#define AF_LINE_OTG_FS_DM                10U
#define AF_PA12_OTG_FS_DP                10U
#define AF_LINE_OTG_FS_DP                10U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PB00_SERVO3                   2U
#define AF_LINE_SERVO3                   2U
#define AF_PB01_SERVO4                   2U
#define AF_LINE_SERVO4                   2U
#define AF_PB03_SPI3_CLK                 6U
#define AF_LINE_SPI3_CLK                 6U
#define AF_PB04_SPI3_MISO                6U
#define AF_LINE_SPI3_MISO                6U
#define AF_PB05_SPI3_MOSI                6U
#define AF_LINE_SPI3_MOSI                6U
#define AF_PB06_SERVO5                   2U
#define AF_LINE_SERVO5                   2U
#define AF_PB07_SERVO7                   2U
#define AF_LINE_SERVO7                   2U
#define AF_PB08_SERVO6                   2U
#define AF_LINE_SERVO6                   2U
#define AF_PB09_AUX2                     3U
#define AF_LINE_AUX2                     3U
#define AF_PB10_I2C2_SCL                 4U
#define AF_LINE_I2C2_SCL                 4U
#define AF_PB11_I2C2_SDA                 4U
#define AF_LINE_I2C2_SDA                 4U
#define AF_PB13_SPI2_CLK                 5U
#define AF_LINE_SPI2_CLK                 5U
#define AF_PB14_SPI2_MISO                5U
#define AF_LINE_SPI2_MISO                5U
#define AF_PB15_SPI2_MOSI                5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_PC06_SERVO1                   2U
#define AF_LINE_SERVO1                   2U
#define AF_PC07_SERVO2                   2U
#define AF_LINE_SERVO2                   2U
#define AF_PC09_SERVO8                   3U
#define AF_LINE_SERVO8                   3U
#define AF_PC10_UART3_TX                 7U
#define AF_LINE_UART3_TX                 7U
#define AF_PC11_UART3_RX                 7U
#define AF_LINE_UART3_RX                 7U
#define AF_PC12_UART5_TX                 8U
#define AF_LINE_UART5_TX                 8U
#define AF_PD02_UART5_RX                 8U
#define AF_LINE_UART5_RX                 8U


#define AUX_TIM	 1
#define AUX_TIM_FN	 CH
#define AUX_TIM_CH	 1
#define AUX_TIM_AF	 1
#define RC1_USART	 1
#define RC1_USART_FN	 RX
#define RC1_USART_AF	 7
#define SERVO3_TIM	 3
#define SERVO3_TIM_FN	 CH
#define SERVO3_TIM_CH	 3
#define SERVO3_TIM_AF	 2
#define SERVO4_TIM	 3
#define SERVO4_TIM_FN	 CH
#define SERVO4_TIM_CH	 4
#define SERVO4_TIM_AF	 2
#define SERVO5_TIM	 4
#define SERVO5_TIM_FN	 CH
#define SERVO5_TIM_CH	 1
#define SERVO5_TIM_AF	 2
#define SERVO7_TIM	 4
#define SERVO7_TIM_FN	 CH
#define SERVO7_TIM_CH	 2
#define SERVO7_TIM_AF	 2
#define SERVO6_TIM	 4
#define SERVO6_TIM_FN	 CH
#define SERVO6_TIM_CH	 3
#define SERVO6_TIM_AF	 2
#define AUX2_TIM	 11
#define AUX2_TIM_FN	 CH
#define AUX2_TIM_CH	 1
#define AUX2_TIM_AF	 3
#define ADC1_ADC	 1
#define ADC1_ADC_FN	 IN
#define ADC1_ADC_IN	 11
#define ADC2_ADC	 1
#define ADC2_ADC_FN	 IN
#define ADC2_ADC_IN	 12
#define SERVO1_TIM	 3
#define SERVO1_TIM_FN	 CH
#define SERVO1_TIM_CH	 1
#define SERVO1_TIM_AF	 2
#define SERVO2_TIM	 3
#define SERVO2_TIM_FN	 CH
#define SERVO2_TIM_CH	 2
#define SERVO2_TIM_AF	 2
#define SERVO8_TIM	 8
#define SERVO8_TIM_FN	 CH
#define SERVO8_TIM_CH	 4
#define SERVO8_TIM_AF	 3

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
	LINE_LED1
#define ENERGY_SAVE_INPUT_SIZE 	 1

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

