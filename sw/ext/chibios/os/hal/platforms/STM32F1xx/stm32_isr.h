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

/**
 * @file    STM32F1xx/stm32_isr.h
 * @brief   ISR remapper driver header.
 *
 * @addtogroup STM32F1xx_ISR
 * @{
 */

#ifndef _STM32_ISR_H_
#define _STM32_ISR_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */
/*
 * CAN units.
 */
#define STM32_CAN1_TX_HANDLER       CAN1_TX_IRQHandler
#define STM32_CAN1_RX0_HANDLER      CAN1_RX0_IRQHandler
#define STM32_CAN1_RX1_HANDLER      CAN1_RX1_IRQHandler
#define STM32_CAN1_SCE_HANDLER      CAN1_SCE_IRQHandler
#define STM32_CAN2_TX_HANDLER       CAN2_TX_IRQHandler
#define STM32_CAN2_RX0_HANDLER      CAN2_RX0_IRQHandler
#define STM32_CAN2_RX1_HANDLER      CAN2_RX1_IRQHandler
#define STM32_CAN2_SCE_HANDLER      CAN2_SCE_IRQHandler

#ifdef STM32F10X_CL
#define STM32_CAN1_TX_NUMBER        CAN1_TX_IRQn
#define STM32_CAN1_RX0_NUMBER       CAN1_RX0_IRQn
#else
#define STM32_CAN1_TX_NUMBER        USB_HP_CAN1_TX_IRQn
#define STM32_CAN1_RX0_NUMBER       USB_LP_CAN1_RX0_IRQn
#endif
#define STM32_CAN1_RX1_NUMBER       CAN1_RX1_IRQn
#define STM32_CAN1_SCE_NUMBER       CAN1_SCE_IRQn
#define STM32_CAN2_TX_NUMBER        CAN2_TX_IRQn
#define STM32_CAN2_RX0_NUMBER       CAN2_RX0_IRQn
#define STM32_CAN2_RX1_NUMBER       CAN2_RX1_IRQn
#define STM32_CAN2_SCE_NUMBER       CAN2_SCE_IRQn

/*
 * OTG units.
 */
#define STM32_OTG1_HANDLER          OTG_FS_IRQHandler

#define STM32_OTG1_NUMBER           OTG_FS_IRQn

/*
 * SDIO unit.
 */
#define STM32_SDIO_HANDLER          SDIO_IRQHandler

#define STM32_SDIO_NUMBER           SDIO_IRQn

/*
 * TIM units.
 */
#if defined(STM32F10X_XL)
#define STM32_TIM1_UP_HANDLER       TIM1_UP_IRQHandler
#elif defined(STM32F10X_LD_VL) || defined(STM32F10X_MD_VL) ||               \
      defined(STM32F10X_HD_VL)
#define STM32_TIM1_UP_HANDLER       TIM1_UP_IRQHandler
#else
#define STM32_TIM1_UP_HANDLER       TIM1_UP_IRQHandler
#endif
#define STM32_TIM1_CC_HANDLER       TIM1_CC_IRQHandler
#define STM32_TIM2_HANDLER          TIM2_IRQHandler
#define STM32_TIM3_HANDLER          TIM3_IRQHandler
#define STM32_TIM4_HANDLER          TIM4_IRQHandler
#define STM32_TIM5_HANDLER          TIM5_IRQHandler
#define STM32_TIM8_UP_HANDLER       TIM8_UP_IRQHandler
#define STM32_TIM8_CC_HANDLER       TIM8_CC_IRQHandler

#if defined(STM32F10X_XL)
#define STM32_TIM1_UP_NUMBER        TIM1_UP_TIM10_IRQn
#elif defined(STM32F10X_LD_VL) || defined(STM32F10X_MD_VL) ||               \
      defined(STM32F10X_HD_VL)
#define STM32_TIM1_UP_NUMBER        TIM1_UP_TIM16_IRQn
#else
#define STM32_TIM1_UP_NUMBER        TIM1_UP_IRQn
#endif
#define STM32_TIM1_CC_NUMBER        TIM1_CC_IRQn
#define STM32_TIM2_NUMBER           TIM2_IRQn
#define STM32_TIM3_NUMBER           TIM3_IRQn
#define STM32_TIM4_NUMBER           TIM4_IRQn
#define STM32_TIM5_NUMBER           TIM5_IRQn
#ifdef STM32F10X_XL
#define STM32_TIM8_UP_NUMBER        TIM8_UP_TIM13_IRQn
#else
#define STM32_TIM8_UP_NUMBER        TIM8_UP_IRQn
#endif
#define STM32_TIM8_CC_NUMBER        TIM8_CC_IRQn

/*
 * USART units.
 */
#define STM32_USART1_HANDLER        USART1_IRQHandler
#define STM32_USART2_HANDLER        USART2_IRQHandler
#define STM32_USART3_HANDLER        USART3_IRQHandler
#define STM32_UART4_HANDLER         UART4_IRQHandler
#define STM32_UART5_HANDLER         UART5_IRQHandler

#define STM32_USART1_NUMBER         USART1_IRQn
#define STM32_USART2_NUMBER         USART2_IRQn
#define STM32_USART3_NUMBER         USART3_IRQn
#define STM32_UART4_NUMBER          UART4_IRQn
#define STM32_UART5_NUMBER          UART5_IRQn

/*
 * USB units.
 */
#define STM32_USB1_HP_HANDLER       Vector8C
#define STM32_USB1_LP_HANDLER       Vector90

#define STM32_USB1_HP_NUMBER        USB_HP_CAN1_TX_IRQn
#define STM32_USB1_LP_NUMBER        USB_LP_CAN1_RX0_IRQn
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* _STM32_ISR_H_ */

/** @} */
