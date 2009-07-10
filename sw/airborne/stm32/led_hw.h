#ifndef LED_HW_H
#define LED_HW_H

#include CONFIG
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "std.h"

#define _LED_GPIO_CLK(i)  i
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i

#define LED_GPIO_CLK(i) _LED_GPIO_CLK(LED_ ## i ## _GPIO_CLK)
#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)

/* set pin as output */
#define LED_INIT(i) {					\
    GPIO_InitTypeDef GPIO_InitStructure; 		\
    RCC_APB2PeriphClockCmd(LED_GPIO_CLK(i), ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN(i);	\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	\
    GPIO_Init(LED_GPIO(i), &GPIO_InitStructure);	\
  }

#define LED_ON(i) { LED_GPIO(i)->BRR  = LED_GPIO_PIN(i);}

#define LED_OFF(i) {LED_GPIO(i)->BSRR = LED_GPIO_PIN(i);}

#if 0
#define LED_TOGGLE(i) {				\
    if (LED_PIN_REG(i) & _BV(LED_PIN(i)))	\
      LED_ON(i)				        \
    else					\
      LED_OFF(i)				\
}
#endif



#endif /* LED_HW_H */
