#ifndef MY_DEBUG_SERVO_H
#define MY_DEBUG_SERVO_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/* using servo 2 connector as debug */

#define DEBUG_S1_TOGGLE() {  GPIOC_ODR ^= GPIO6; }
#define DEBUG_S1_ON()     {  GPIOC_BSRR = GPIO6; }
#define DEBUG_S1_OFF()    {  GPIOC_BRR  = GPIO6; }

#define DEBUG_S2_TOGGLE() {  GPIOC_ODR ^= GPIO7; }
#define DEBUG_S2_ON()     {  GPIOC_BSRR = GPIO7; }
#define DEBUG_S2_OFF()    {  GPIOC_BRR  = GPIO7; }

#define DEBUG_S3_TOGGLE() {  GPIOC_ODR ^= GPIO8; }
#define DEBUG_S3_ON()     {  GPIOC_BSRR = GPIO8; }
#define DEBUG_S3_OFF()    {  GPIOC_BRR  = GPIO8; }

#define DEBUG_S4_TOGGLE() {  GPIOC_ODR ^= GPIO9; }
#define DEBUG_S4_ON()     {  GPIOC_BSRR = GPIO9; }
#define DEBUG_S4_OFF()    {  GPIOC_BRR  = GPIO9; }

#define DEBUG_S5_TOGGLE() {  GPIOB_ODR ^= GPIO8; }
#define DEBUG_S5_ON()     {  GPIOB_BSRR = GPIO8; }
#define DEBUG_S5_OFF()    {  GPIOB_BRR  = GPIO8; }

#define DEBUG_S6_TOGGLE() {  GPIOB_ODR ^= GPIO9; }
#define DEBUG_S6_ON()     {  GPIOB_BSRR = GPIO9; }
#define DEBUG_S6_OFF()    {  GPIOB_BRR  = GPIO9; }



#define DEBUG_SERVO1_INIT() {                                           \
    /* S1: PC6    S2: PC7    S3: PC8 */                                 \
    GPIOC_BSRR = GPIO6 | GPIO7 | GPIO8 ;                                \
    rcc_periph_clock_enable(RCC_GPIOC);                                 \
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,                       \
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO6 | GPIO7 | GPIO8);     \
    DEBUG_S1_OFF();                                                     \
    DEBUG_S2_OFF();                                                     \
    DEBUG_S3_OFF();                                                     \
  }

#define DEBUG_SERVO2_INIT() {                                           \
    /* S4: PC9 */                                                       \
    GPIOC_BSRR = GPIO9;                                                 \
    rcc_periph_clock_enable(RCC_GPIOC);                                 \
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,                       \
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);                     \
    /* S5: PB8 and S6: PB9 */                                           \
    GPIOB_BSRR = GPIO8 | GPIO9;                                         \
    rcc_periph_clock_enable(RCC_GPIOB);                                 \
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,                       \
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9);             \
    DEBUG_S4_OFF();                                                     \
    DEBUG_S5_OFF();                                                     \
    DEBUG_S6_OFF();                                                     \
  }


#endif /* MY_DEBUG_SERVO_H */
