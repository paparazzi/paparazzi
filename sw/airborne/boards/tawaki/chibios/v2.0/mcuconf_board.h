/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * STM32H7xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define STM32H7xx_MCUCONF
#define STM32H742_MCUCONF
#define STM32H743_MCUCONF
#define STM32H753_MCUCONF
#define STM32H745_MCUCONF
#define STM32H755_MCUCONF
#define STM32H747_MCUCONF
#define STM32H757_MCUCONF
#define STM32H750_MCUCONF

/*
 * General settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_TARGET_CORE                   1

/*
 * Memory attributes settings.
 */
// nocache regions are configured in mcu_arch.c because we need to configure 3 regions.
// This ChibiOS configuration can only configure 1 nocache region.
#define STM32_NOCACHE_ENABLE                FALSE


/*
 * PWR system settings.
 * Reading STM32 Reference Manual is required, settings in PWR_CR3 are
 * very critical.
 * Register constants are taken from the ST header.
 */
#define STM32_VOS                           STM32_VOS_SCALE1
#define STM32_PWR_CR1                       (PWR_CR1_SVOS_1 | PWR_CR1_SVOS_0)
#define STM32_PWR_CR2                       (PWR_CR2_BREN)
#define STM32_PWR_CR3                       (PWR_CR3_LDOEN | PWR_CR3_USB33DEN)
#define STM32_PWR_CPUCR                     0

/*
 * Clock tree static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_CSI_ENABLED                   TRUE
#define STM32_HSI48_ENABLED                 TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_HSIDIV                        STM32_HSIDIV_DIV1

/*
 * PLLs static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK
#define STM32_PLLCFGR_MASK                  ~0
#define STM32_PLL1_ENABLED                  TRUE
#define STM32_PLL1_P_ENABLED                TRUE
#define STM32_PLL1_Q_ENABLED                TRUE
#define STM32_PLL1_R_ENABLED                TRUE
#define STM32_PLL1_DIVM_VALUE               4
#define STM32_PLL1_DIVN_VALUE               480
#define STM32_PLL1_FRACN_VALUE              0
#define STM32_PLL1_DIVP_VALUE               2
#define STM32_PLL1_DIVQ_VALUE               20
#define STM32_PLL1_DIVR_VALUE               8
#define STM32_PLL2_ENABLED                  TRUE
#define STM32_PLL2_P_ENABLED                TRUE
#define STM32_PLL2_Q_ENABLED                TRUE
#define STM32_PLL2_R_ENABLED                TRUE
#define STM32_PLL2_DIVM_VALUE               4
#define STM32_PLL2_DIVN_VALUE               400
#define STM32_PLL2_FRACN_VALUE              0
#define STM32_PLL2_DIVP_VALUE               40
#define STM32_PLL2_DIVQ_VALUE               8
#define STM32_PLL2_DIVR_VALUE               8
#define STM32_PLL3_ENABLED                  TRUE
#define STM32_PLL3_P_ENABLED                TRUE
#define STM32_PLL3_Q_ENABLED                TRUE
#define STM32_PLL3_R_ENABLED                TRUE
#define STM32_PLL3_DIVM_VALUE               4
#define STM32_PLL3_DIVN_VALUE               400
#define STM32_PLL3_FRACN_VALUE              0
#define STM32_PLL3_DIVP_VALUE               8
#define STM32_PLL3_DIVQ_VALUE               8
#define STM32_PLL3_DIVR_VALUE               8

/*
 * Core clocks dynamic settings (can be changed at runtime).
 * Reading STM32 Reference Manual is required.
 */
#define STM32_SW                            STM32_SW_PLL1_P_CK

#if HAL_USE_RTC
#define STM32_RTCSEL                        STM32_RTCSEL_LSI_CK
#else
#define STM32_RTCSEL                        STM32_RTCSEL_NOCLK
#endif

#define STM32_D1CPRE                        STM32_D1CPRE_DIV1
#define STM32_D1HPRE                        STM32_D1HPRE_DIV4
#define STM32_D1PPRE3                       STM32_D1PPRE3_DIV1
#define STM32_D2PPRE1                       STM32_D2PPRE1_DIV1
#define STM32_D2PPRE2                       STM32_D2PPRE2_DIV1
#define STM32_D3PPRE4                       STM32_D3PPRE4_DIV1

/*
 * Peripherals clocks static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI_CK
#define STM32_MCO1PRE_VALUE                 4
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYS_CK
#define STM32_MCO2PRE_VALUE                 4
#define STM32_TIMPRE_ENABLE                 TRUE
#define STM32_HRTIMSEL                      0
#define STM32_STOPKERWUCK                   0
#define STM32_STOPWUCK                      0
#define STM32_RTCPRE_VALUE                  8
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSE_CK
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL1_Q_CK
#define STM32_QSPISEL                       STM32_QSPISEL_HCLK
#define STM32_FMCSEL                        STM32_QSPISEL_HCLK
#define STM32_SWPSEL                        STM32_SWPSEL_PCLK1
#define STM32_FDCANSEL                      STM32_FDCANSEL_HSE_CK
#define STM32_DFSDM1SEL                     STM32_DFSDM1SEL_PCLK2
#define STM32_SPDIFSEL                      STM32_SPDIFSEL_PLL1_Q_CK
#define STM32_SPI45SEL                      STM32_SPI45SEL_PCLK2
#define STM32_SPI123SEL                     STM32_SPI123SEL_PLL1_Q_CK
#define STM32_SAI23SEL                      STM32_SAI23SEL_PLL1_Q_CK
#define STM32_SAI1SEL                       STM32_SAI1SEL_PLL1_Q_CK
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#define STM32_CECSEL                        STM32_CECSEL_LSE_CK
#define STM32_USBSEL                        STM32_USBSEL_PLL1_Q_CK
#define STM32_I2C123SEL                     STM32_I2C123SEL_PCLK1
#define STM32_RNGSEL                        STM32_RNGSEL_HSI48_CK
#define STM32_USART16SEL                    STM32_USART16SEL_PCLK2
#define STM32_USART234578SEL                STM32_USART234578SEL_PCLK1
#define STM32_SPI6SEL                       STM32_SPI6SEL_PCLK4
#define STM32_SAI4BSEL                      STM32_SAI4BSEL_PLL1_Q_CK
#define STM32_SAI4ASEL                      STM32_SAI4ASEL_PLL1_Q_CK
#define STM32_ADCSEL                        STM32_ADCSEL_PLL2_P_CK
#define STM32_LPTIM345SEL                   STM32_LPTIM345SEL_PCLK4
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK4
#define STM32_I2C4SEL                       STM32_I2C4SEL_PCLK4
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK4

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#define STM32_IRQ_EXTI16_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           6
#define STM32_IRQ_EXTI18_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6
#define STM32_IRQ_EXTI20_21_PRIORITY        6

#define STM32_IRQ_FDCAN1_PRIORITY           10
#define STM32_IRQ_FDCAN2_PRIORITY           10

#define STM32_IRQ_MDMA_PRIORITY             9

#define STM32_IRQ_QUADSPI1_PRIORITY         10

#define STM32_IRQ_SDMMC1_PRIORITY           9
#define STM32_IRQ_SDMMC2_PRIORITY           9

#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   7
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    7
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM15_PRIORITY            7
#define STM32_IRQ_TIM16_PRIORITY            7
#define STM32_IRQ_TIM17_PRIORITY            7

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12
#define STM32_IRQ_LPUART1_PRIORITY          12

/*
 * ADC driver system settings.
 */
#define STM32_ADC_DUAL_MODE                 FALSE
#define STM32_ADC_SAMPLES_SIZE              16
#define STM32_ADC_USE_ADC12                 TRUE
#define STM32_ADC_USE_ADC3                  TRUE
#define STM32_ADC_ADC12_DMA_STREAM          STM32_DMA_STREAM_ID_ANY
#define STM32_ADC_ADC3_BDMA_STREAM          STM32_BDMA_STREAM_ID_ANY
#define STM32_ADC_ADC12_DMA_PRIORITY        2
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#define STM32_ADC_ADC12_IRQ_PRIORITY        5
#define STM32_ADC_ADC3_IRQ_PRIORITY         5
#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC_ADC3_CLOCK_MODE           ADC_CCR_CKMODE_AHB_DIV4

/*
 * CAN driver system settings.
 */

#if USE_CAN1
#define STM32_CAN_USE_FDCAN1                TRUE
#else
#define STM32_CAN_USE_FDCAN1                FALSE
#endif
#define STM32_CAN_USE_FDCAN2                FALSE
/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 FALSE
#define STM32_DAC_USE_DAC1_CH1              FALSE
#define STM32_DAC_USE_DAC1_CH2              FALSE
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH1_DMA_STREAM       STM32_DMA_STREAM_ID_ANY
#define STM32_DAC_DAC1_CH2_DMA_STREAM       STM32_DMA_STREAM_ID_ANY

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  FALSE
#define STM32_GPT_USE_TIM4                  FALSE
#if USE_GPT5
#define STM32_GPT_USE_TIM5                  TRUE
#else
#define STM32_GPT_USE_TIM5                  FALSE
#endif
#define STM32_GPT_USE_TIM6                  TRUE
#if USE_GPT7
#define STM32_GPT_USE_TIM7                  TRUE
#else
#define STM32_GPT_USE_TIM7                  FALSE
#endif
#if USE_GPT8
#define STM32_GPT_USE_TIM8                  TRUE
#else
#define STM32_GPT_USE_TIM8                  FALSE
#endif
#if USE_GPT12
#define STM32_GPT_USE_TIM12                  TRUE
#else
#define STM32_GPT_USE_TIM12                  FALSE
#endif
#if USE_GPT13
#define STM32_GPT_USE_TIM13                  TRUE
#else
#define STM32_GPT_USE_TIM13                  FALSE
#endif
#define STM32_GPT_USE_TIM14                 FALSE
#define STM32_GPT_USE_TIM15                 FALSE
#define STM32_GPT_USE_TIM16                 FALSE
#define STM32_GPT_USE_TIM17                 FALSE

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  FALSE
#if USE_I2C2
#define STM32_I2C_USE_I2C2                  TRUE
#else
#define STM32_I2C_USE_I2C2                  FALSE
#endif
#define STM32_I2C_USE_I2C3                  FALSE
#if USE_I2C4
#define STM32_I2C_USE_I2C4                  TRUE
#else
#define STM32_I2C_USE_I2C4                  FALSE
#endif
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C1_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C2_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C2_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C3_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C3_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C4_RX_BDMA_STREAM       STM32_BDMA_STREAM_ID_ANY
#define STM32_I2C_I2C4_TX_BDMA_STREAM       STM32_BDMA_STREAM_ID_ANY
#define STM32_I2C_I2C1_IRQ_PRIORITY         5
#define STM32_I2C_I2C2_IRQ_PRIORITY         5
#define STM32_I2C_I2C3_IRQ_PRIORITY         5
#define STM32_I2C_I2C4_IRQ_PRIORITY         5
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_I2C3_DMA_PRIORITY         3
#define STM32_I2C_I2C4_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  TRUE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_USE_TIM8                  FALSE
#define STM32_ICU_USE_TIM12                 FALSE
#define STM32_ICU_USE_TIM13                 FALSE
#define STM32_ICU_USE_TIM14                 FALSE
#define STM32_ICU_USE_TIM15                 FALSE
#define STM32_ICU_USE_TIM16                 FALSE
#define STM32_ICU_USE_TIM17                 FALSE

/*
 * MAC driver system settings.
 */
#define STM32_MAC_TRANSMIT_BUFFERS          2
#define STM32_MAC_RECEIVE_BUFFERS           4
#define STM32_MAC_BUFFERS_SIZE              1522
#define STM32_MAC_PHY_TIMEOUT               100
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     TRUE
#define STM32_MAC_ETH1_IRQ_PRIORITY         13
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       0

/*
 * PWM driver system settings.
 */
#ifndef STM32_PWM_USE_TIM1
#define STM32_PWM_USE_TIM1                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM2
#define STM32_PWM_USE_TIM2                  FALSE
#endif
#ifndef STM32_PWM_USE_TIM3
#define STM32_PWM_USE_TIM3                  TRUE
#endif
#ifndef STM32_PWM_USE_TIM4
#define STM32_PWM_USE_TIM4                  TRUE
#endif
#define STM32_PWM_USE_TIM5                  FALSE
#define STM32_PWM_USE_TIM8                  FALSE
#define STM32_PWM_USE_TIM12                 FALSE
#define STM32_PWM_USE_TIM13                 FALSE
#define STM32_PWM_USE_TIM14                 FALSE
#ifndef STM32_PWM_USE_TIM15
#define STM32_PWM_USE_TIM15                 FALSE
#endif
#define STM32_PWM_USE_TIM16                 FALSE
#define STM32_PWM_USE_TIM17                 FALSE

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               32
#define STM32_RTC_PRESS_VALUE               1024
#define STM32_RTC_CR_INIT                   0
#define STM32_RTC_TAMPCR_INIT               0

/*
 * SDC driver system settings.
 */
#define STM32_SDC_USE_SDMMC1                TRUE
#define STM32_SDC_USE_SDMMC2                FALSE
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   TRUE
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       1000000
#define STM32_SDC_SDMMC_READ_TIMEOUT        1000000
#define STM32_SDC_SDMMC_CLOCK_DELAY         10
#define STM32_SDC_SDMMC_PWRSAV              TRUE

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             TRUE // enabled by default for dshot telemetry
#if USE_UART2
#define STM32_SERIAL_USE_USART2             TRUE
#else
#define STM32_SERIAL_USE_USART2             FALSE
#endif
#if USE_UART3
#define STM32_SERIAL_USE_USART3             TRUE
#else
#define STM32_SERIAL_USE_USART3             FALSE
#endif
#if USE_UART4
#define STM32_SERIAL_USE_USART4             TRUE
#else
#define STM32_SERIAL_USE_USART4             FALSE
#endif
#define STM32_SERIAL_USE_UART5              FALSE
#define STM32_SERIAL_USE_USART6             FALSE
#if USE_UART7
#define STM32_SERIAL_USE_UART7             TRUE
#else
#define STM32_SERIAL_USE_UART7             FALSE
#endif
#if USE_UART8
#define STM32_SERIAL_USE_UART8             TRUE
#else
#define STM32_SERIAL_USE_UART8             FALSE
#endif
#define STM32_SERIAL_USE_LPUART1            FALSE

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                FALSE
#define STM32_SIO_USE_USART2                FALSE
#define STM32_SIO_USE_USART3                FALSE
#define STM32_SIO_USE_UART4                 FALSE
#define STM32_SIO_USE_UART5                 FALSE
#define STM32_SIO_USE_USART6                FALSE
#define STM32_SIO_USE_UART7                 FALSE
#define STM32_SIO_USE_UART8                 FALSE
#define STM32_SIO_USE_LPUART1               FALSE

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  FALSE
#if USE_SPI2
#define STM32_SPI_USE_SPI2                  TRUE
#else
#define STM32_SPI_USE_SPI2                  FALSE
#endif
#define STM32_SPI_USE_SPI3                  FALSE
#define STM32_SPI_USE_SPI4                  FALSE
#define STM32_SPI_USE_SPI5                  FALSE
#if USE_SPI6
#define STM32_SPI_USE_SPI6                  TRUE
#else
#define STM32_SPI_USE_SPI6                  FALSE
#endif
#define STM32_SPI_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI2_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI2_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI3_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI3_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI4_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI4_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI5_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI5_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI6_RX_BDMA_STREAM       STM32_BDMA_STREAM_ID_ANY
#define STM32_SPI_SPI6_TX_BDMA_STREAM       STM32_BDMA_STREAM_ID_ANY
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#define STM32_SPI_SPI5_DMA_PRIORITY         1
#define STM32_SPI_SPI6_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_SPI4_IRQ_PRIORITY         10
#define STM32_SPI_SPI5_IRQ_PRIORITY         10
#define STM32_SPI_SPI6_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2

/*
 * TRNG driver system settings.
 */
#define STM32_TRNG_USE_RNG1                 FALSE

/*
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               FALSE
#define STM32_UART_USE_USART2               TRUE
#define STM32_UART_USE_USART3               FALSE
#define STM32_UART_USE_UART4                FALSE
#define STM32_UART_USE_UART5                FALSE
#define STM32_UART_USE_USART6               FALSE
#define STM32_UART_USE_UART7                FALSE
#define STM32_UART_USE_UART8                FALSE
#define STM32_UART_USART1_RX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART1_TX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART2_RX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART2_TX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART3_RX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART3_TX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART4_RX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART4_TX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART5_RX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART5_TX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART6_RX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART6_TX_DMA_STREAM     STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART7_RX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART7_TX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART8_RX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_UART8_TX_DMA_STREAM      STM32_DMA_STREAM_ID_ANY
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_UART4_DMA_PRIORITY       0
#define STM32_UART_UART5_DMA_PRIORITY       0
#define STM32_UART_USART6_DMA_PRIORITY      0
#define STM32_UART_UART7_DMA_PRIORITY       0
#define STM32_UART_UART8_DMA_PRIORITY       0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    osalSysHalt("DMA failure")

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_USB_USE_OTG2                  FALSE
#define STM32_USB_OTG1_IRQ_PRIORITY         14
#define STM32_USB_OTG2_IRQ_PRIORITY         14
#define STM32_USB_OTG1_RX_FIFO_SIZE         512
#define STM32_USB_OTG2_RX_FIFO_SIZE         1024
#define STM32_USB_HOST_WAKEUP_DURATION      2

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

/*
 * WSPI driver system settings.
 */
#define STM32_WSPI_USE_QUADSPI1             FALSE
#define STM32_WSPI_QUADSPI1_PRESCALER_VALUE 1
#define STM32_WSPI_SET_CR_SSHIFT            TRUE
#define STM32_WSPI_QUADSPI1_MDMA_CHANNEL    STM32_MDMA_CHANNEL_ID_ANY
#define STM32_WSPI_QUADSPI1_MDMA_PRIORITY   1
#define STM32_WSPI_MDMA_ERROR_HOOK(wspip)   osalSysHalt("MDMA failure")

/*
  sdlog message buffer and queue configuration
 */
#define SDLOG_QUEUE_BUCKETS    1024
#define SDLOG_MAX_MESSAGE_LEN  300
#define SDLOG_NUM_FILES        2
#define SDLOG_ALL_BUFFERS_SIZE (SDLOG_NUM_FILES*16*1024)

// #define CH_HEAP_SIZE (32*1024)
// #define CH_HEAP_USE_TLSF 0 // if 0 or undef, chAlloc will be used
// #define CONSOLE_DEV_SD SD3

#endif /* MCUCONF_H */
