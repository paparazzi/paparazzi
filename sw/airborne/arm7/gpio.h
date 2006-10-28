#ifndef GPIO_H
#define GPIO_H

#include "std.h"
#include "LPC21xx.h"

extern bool_t gpio1_status;

#define GPIO_1_BANK 0
#define GPIO_1_PIN 7
#define GPIO_1_PINSEL PINSEL0
#define GPIO_1_PINSEL_BIT 14
#define GPIO_1_PINSEL_VAL 0
#define GPIO_1_DIR IO0DIR

#define GpioOn1() { \
 gpio1_status = TRUE; \
 IO0SET = _BV(GPIO_1_PIN); \
} 

#define GpioOff1() { \
 gpio1_status = FALSE; \
 IO0CLR = _BV(GPIO_1_PIN); \
}

#define GpioUpdate1() { \
 if (gpio1_status) \
   IO0SET = _BV(GPIO_1_PIN); \
 else \
   IO0CLR = _BV(GPIO_1_PIN); \
}

#define GpioInit() { \
  GPIO_1_PINSEL |= GPIO_1_PINSEL_VAL << GPIO_1_PINSEL_BIT; \
  GPIO_1_DIR |= _BV(GPIO_1_PIN); \
  GpioOff1(); \
}

#endif /* GPIO_H */
