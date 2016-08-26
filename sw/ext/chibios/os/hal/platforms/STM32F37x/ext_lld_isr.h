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
 * @file    STM32F37x/ext_lld_isr.h
 * @brief   STM32F37x EXT subsystem low level driver ISR header.
 *
 * @addtogroup EXT
 * @{
 */

#ifndef _EXT_LLD_ISR_H_
#define _EXT_LLD_ISR_H_

#if HAL_USE_EXT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EXTI0 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI0_IRQ_PRIORITY        6
#endif

/**
 * @brief   EXTI1 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI1_IRQ_PRIORITY        6
#endif

/**
 * @brief   EXTI2 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI2_IRQ_PRIORITY        6
#endif

/**
 * @brief   EXTI3 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI3_IRQ_PRIORITY        6
#endif

/**
 * @brief   EXTI4 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI4_IRQ_PRIORITY        6
#endif

/**
 * @brief   EXTI5..9 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI5_9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI5_9_IRQ_PRIORITY      6
#endif

/**
 * @brief   EXTI10..15 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI10_15_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI10_15_IRQ_PRIORITY    6
#endif

/**
 * @brief   EXTI16 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI16_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI16_IRQ_PRIORITY       6
#endif

/**
 * @brief   EXTI17 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI17_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI17_IRQ_PRIORITY       6
#endif

/**
 * @brief   EXTI18 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI18_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI18_IRQ_PRIORITY       6
#endif

/**
 * @brief   EXTI19 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI19_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI19_IRQ_PRIORITY       6
#endif

/**
 * @brief   EXTI20 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI20_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI20_IRQ_PRIORITY       6
#endif

/**
 * @brief   EXTI21..22 interrupt priority level setting.
 */
#if !defined(STM32_EXT_EXTI21_22_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EXT_EXTI21_22_IRQ_PRIORITY    6
#endif
/** @} */

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

#ifdef __cplusplus
extern "C" {
#endif
  void ext_lld_exti_irq_enable(void);
  void ext_lld_exti_irq_disable(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EXT */

#endif /* _EXT_LLD_ISR_H_ */

/** @} */
