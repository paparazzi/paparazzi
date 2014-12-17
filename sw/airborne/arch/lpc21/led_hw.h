#ifndef LED_HW_H
#define LED_HW_H

#include BOARD_CONFIG
#include "LPC21xx.h"
#include "std.h"

#define LED_PERIODIC() {}

#define __LED_DIR(i) IO ## i ## DIR
#define _LED_DIR(i) __LED_DIR(i)
#define __LED_CLR(i) IO ## i ## CLR
#define _LED_CLR(i) __LED_CLR(i)
#define __LED_SET(i) IO ## i ## SET
#define _LED_SET(i) __LED_SET(i)
#define __LED_PIN_REG(i) IO ## i ## PIN
#define _LED_PIN_REG(i) __LED_PIN_REG(i)

#define LED_DIR(i) _LED_DIR(LED_ ## i ## _BANK)
#define LED_CLR(i) _LED_CLR(LED_ ## i ## _BANK)
#define LED_SET(i) _LED_SET(LED_ ## i ## _BANK)
#define LED_PIN_REG(i) _LED_PIN_REG(LED_ ## i ## _BANK)
#define LED_PIN(i) LED_ ## i ## _PIN

/* set pin as output */
#define LED_INIT(i)  LED_DIR(i) |= _BV(LED_PIN(i))

#define LED_ON(i) LED_CLR(i) = _BV(LED_PIN(i));
#define LED_OFF(i) LED_SET(i) = _BV(LED_PIN(i));
#define LED_TOGGLE(i) {       \
    if (LED_PIN_REG(i) & _BV(LED_PIN(i))) \
      LED_ON(i)               \
      else          \
        LED_OFF(i)        \
      }

#endif /* LED_HW_H */
