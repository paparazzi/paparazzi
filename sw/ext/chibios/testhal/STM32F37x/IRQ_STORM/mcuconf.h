/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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

/*
 * STM32F30x drivers configuration.
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

#define STM32F37x_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PREDIV_VALUE                  1
#define STM32_PLLMUL_VALUE                  9
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_MCOSEL                        STM32_MCOSEL_NOCLOCK
#define STM32_ADCPRE                        STM32_ADCPRE_DIV4
#define STM32_SDPRE                         STM32_SDPRE_DIV12
#define STM32_USART1SW                      STM32_USART1SW_PCLK
#define STM32_USART2SW                      STM32_USART2SW_PCLK
#define STM32_USART3SW                      STM32_USART3SW_PCLK
#define STM32_I2C1SW                        STM32_I2C1SW_SYSCLK
#define STM32_I2C2SW                        STM32_I2C2SW_SYSCLK
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#define STM32_USB_CLOCK_REQUIRED            TRUE
#define STM32_USBPRE                        STM32_USBPRE_DIV1P5

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  FALSE
#define STM32_ADC_USE_SDADC1                FALSE
#define STM32_ADC_USE_SDADC2                FALSE
#define STM32_ADC_USE_SDADC3                FALSE
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_SDADC1_DMA_PRIORITY       2
#define STM32_ADC_SDADC2_DMA_PRIORITY       2
#define STM32_ADC_SDADC3_DMA_PRIORITY       2
#define STM32_ADC_IRQ_PRIORITY              5
#define STM32_ADC_SDADC1_IRQ_PRIORITY       5
#define STM32_ADC_SDADC2_IRQ_PRIORITY       5
#define STM32_ADC_SDADC3_IRQ_PRIORITY       5
#define STM32_ADC_SDADC1_DMA_IRQ_PRIORITY   5
#define STM32_ADC_SDADC2_DMA_IRQ_PRIORITY   5
#define STM32_ADC_SDADC3_DMA_IRQ_PRIORITY   5

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  TRUE
#define STM32_CAN_CAN1_IRQ_PRIORITY         11

/*
 * EXT driver system settings.
 */
#define STM32_EXT_EXTI0_IRQ_PRIORITY        6
#define STM32_EXT_EXTI1_IRQ_PRIORITY        6
#define STM32_EXT_EXTI2_IRQ_PRIORITY        6
#define STM32_EXT_EXTI3_IRQ_PRIORITY        6
#define STM32_EXT_EXTI4_IRQ_PRIORITY        6
#define STM32_EXT_EXTI5_9_IRQ_PRIORITY      6
#define STM32_EXT_EXTI10_15_IRQ_PRIORITY    6
#define STM32_EXT_EXTI16_IRQ_PRIORITY       6
#define STM32_EXT_EXTI17_IRQ_PRIORITY       6
#define STM32_EXT_EXTI18_IRQ_PRIORITY       6
#define STM32_EXT_EXTI19_IRQ_PRIORITY       6
#define STM32_EXT_EXTI20_23_IRQ_PRIORITY    6
#define STM32_EXT_EXTI30_32_IRQ_PRIORITY    6
#define STM32_EXT_EXTI33_IRQ_PRIORITY       6

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM2                  TRUE
#define STM32_GPT_USE_TIM3                  TRUE
#define STM32_GPT_USE_TIM4                  TRUE
#define STM32_GPT_USE_TIM5                  TRUE
#define STM32_GPT_USE_TIM6                  TRUE
#define STM32_GPT_USE_TIM7                  TRUE
#define STM32_GPT_USE_TIM12                 TRUE
#define STM32_GPT_USE_TIM14                 TRUE
#define STM32_GPT_TIM2_IRQ_PRIORITY         6
#define STM32_GPT_TIM3_IRQ_PRIORITY         10
#define STM32_GPT_TIM4_IRQ_PRIORITY         7
#define STM32_GPT_TIM5_IRQ_PRIORITY         7
#define STM32_GPT_TIM6_IRQ_PRIORITY         7
#define STM32_GPT_TIM7_IRQ_PRIORITY         7
#define STM32_GPT_TIM12_IRQ_PRIORITY        7
#define STM32_GPT_TIM14_IRQ_PRIORITY        7

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  FALSE
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_I2C1_IRQ_PRIORITY         10
#define STM32_I2C_I2C2_IRQ_PRIORITY         10
#define STM32_I2C_I2C1_DMA_PRIORITY         1
#define STM32_I2C_I2C2_DMA_PRIORITY         1
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      chSysHalt()

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#define STM32_ICU_TIM5_IRQ_PRIORITY         7

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM2                  FALSE
#define STM32_PWM_USE_TIM3                  FALSE
#define STM32_PWM_USE_TIM4                  FALSE
#define STM32_PWM_USE_TIM5                  FALSE
#define STM32_PWM_TIM2_IRQ_PRIORITY         7
#define STM32_PWM_TIM3_IRQ_PRIORITY         7
#define STM32_PWM_TIM4_IRQ_PRIORITY         7
#define STM32_PWM_TIM5_IRQ_PRIORITY         7

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             FALSE
#define STM32_SERIAL_USE_USART2             TRUE
#define STM32_SERIAL_USE_USART3             FALSE
#define STM32_SERIAL_USE_UART4              FALSE
#define STM32_SERIAL_USE_UART5              FALSE
#define STM32_SERIAL_USART1_PRIORITY        12
#define STM32_SERIAL_USART2_PRIORITY        12
#define STM32_SERIAL_USART3_PRIORITY        12
#define STM32_SERIAL_UART4_PRIORITY         12
#define STM32_SERIAL_UART5_PRIORITY         12

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  FALSE
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      chSysHalt()

/*
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               FALSE
#define STM32_UART_USE_USART2               FALSE
#define STM32_UART_USE_USART3               FALSE
#define STM32_UART_USART1_IRQ_PRIORITY      12
#define STM32_UART_USART2_IRQ_PRIORITY      12
#define STM32_UART_USART3_IRQ_PRIORITY      12
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    chSysHalt()

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_USB1                  FALSE
#define STM32_USB_LOW_POWER_ON_SUSPEND      FALSE
#define STM32_USB_USB1_HP_IRQ_PRIORITY      13
#define STM32_USB_USB1_LP_IRQ_PRIORITY      14
