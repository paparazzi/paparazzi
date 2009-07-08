/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_lib.h
* Author             : MCD Application Team
* Date First Issued  : 09/29/2006
* Description        : This file includes the peripherals header files in the
*                      user application.
********************************************************************************
* History:
* 04/02/2007: V0.2
* 02/05/2007: V0.1
* 09/29/2006: V0.01
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_LIB_H
#define __STM32F10x_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

#ifdef _GPIO
  #include "stm32f10x_gpio.h"
#endif /*_GPIO */

#ifdef _RCC
  #include "stm32f10x_rcc.h"
#endif /*_RCC */

#ifdef _USART
  #include "stm32f10x_usart.h"
#endif /*_USART */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void debug(void);

#endif /* __STM32F10x_LIB_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
