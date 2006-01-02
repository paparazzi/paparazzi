#ifndef LED_H
#define LED_H

#ifdef LED

#include "led_hw.h"

static inline void led_init ( void ) {
#ifdef LED_1_BANK
  LED_INIT(1);
#endif /* LED_1_BANK */
#ifdef LED_2_BANK
  LED_INIT(2);
#endif /* LED_2_BANK */
#ifdef LED_3_BANK
  LED_INIT(3);
#endif /* LED_3_BANK */
#ifdef LED_4_BANK
  LED_INIT(4);
#endif /* LED_4_BANK */
}

#endif /* LED */

#endif /* LED_H */
