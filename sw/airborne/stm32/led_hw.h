#ifndef LED_HW_H
#define LED_HW_H

#include CONFIG
#include <stm32/gpio.h>


#define __LED_GPIO_CLCK(i) RCC_APB2Periph_## i ##
#define _LED_GPIO_CLCK(i) __LED_GPIO_CLCK(i)

#define __LED_GPIO_PIN(i) GPIO_Pin_## i ##
#define _LED_GPIO_PIN(i) __LED_GPIO_PIN(i)


#define LED_DIR(i) _LED_DIR(LED_ ## i ## _BANK)
#define LED_CLR(i) _LED_CLR(LED_ ## i ## _BANK)
#define LED_SET(i) _LED_SET(LED_ ## i ## _BANK)
#define LED_PIN_REG(i) _LED_PIN_REG(LED_ ## i ## _BANK)
#define LED_PIN(i) LED_ ## i ## _PIN

/* set pin as output */
#define LED_INIT(i) {					\
    GPIO_InitTypeDef GPIO_InitStructure; 		\
    RCC_APB2PeriphClockCmd(_LED_GPIO_CLK(i), ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = _GPIO_PIN(i);		\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	\
    GPIO_Init(LED_GPIO_PORT(i), &GPIO_InitStructure);	\
  }

#if 0
#define LED_ON(i) LED_CLR(i) = _BV(LED_PIN(i));
#define LED_OFF(i) LED_SET(i) = _BV(LED_PIN(i));
#define LED_TOGGLE(i) {				\
    if (LED_PIN_REG(i) & _BV(LED_PIN(i)))	\
      LED_ON(i)				        \
    else					\
      LED_OFF(i)				\
}
#endif

#endif /* LED_HW_H */
