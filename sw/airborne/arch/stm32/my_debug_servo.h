#ifndef MY_DEBUG_SERVO_H
#define MY_DEBUG_SERVO_H

#include <stm32/gpio.h>
#include <stm32/rcc.h>

/* using servo 2 connector as debug */

#define DEBUG_S1_TOGGLE() {  GPIOC->ODR ^= GPIO_Pin_6; }
#define DEBUG_S1_ON()     {  GPIOC->BSRR = GPIO_Pin_6; }
#define DEBUG_S1_OFF()    {  GPIOC->BRR  = GPIO_Pin_6; }

#define DEBUG_S2_TOGGLE() {  GPIOC->ODR ^= GPIO_Pin_7; }
#define DEBUG_S2_ON()     {  GPIOC->BSRR = GPIO_Pin_7; }
#define DEBUG_S2_OFF()    {  GPIOC->BRR  = GPIO_Pin_7; }

#define DEBUG_S3_TOGGLE() {  GPIOC->ODR ^= GPIO_Pin_8; }
#define DEBUG_S3_ON()     {  GPIOC->BSRR = GPIO_Pin_8; }
#define DEBUG_S3_OFF()    {  GPIOC->BRR  = GPIO_Pin_8; }

#define DEBUG_S4_TOGGLE() {  GPIOC->ODR ^= GPIO_Pin_9; }
#define DEBUG_S4_ON()     {  GPIOC->BSRR = GPIO_Pin_9; }
#define DEBUG_S4_OFF()    {  GPIOC->BRR  = GPIO_Pin_9; }

#define DEBUG_S5_TOGGLE() {  GPIOB->ODR ^= GPIO_Pin_8; }
#define DEBUG_S5_ON()     {  GPIOB->BSRR = GPIO_Pin_8; }
#define DEBUG_S5_OFF()    {  GPIOB->BRR  = GPIO_Pin_8; }

#define DEBUG_S6_TOGGLE() {  GPIOB->ODR ^= GPIO_Pin_9; }
#define DEBUG_S6_ON()     {  GPIOB->BSRR = GPIO_Pin_9; }
#define DEBUG_S6_OFF()    {  GPIOB->BRR  = GPIO_Pin_9; }



#define DEBUG_SERVO1_INIT() {						\
    /* S1: PC6    S2: PC7    S3: PC8 */					\
    GPIO_InitTypeDef GPIO_InitStructure;				\
    GPIOC->BSRR = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 ;		\
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);		\
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;	\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			\
    GPIO_Init(GPIOC, &GPIO_InitStructure);				\
    DEBUG_S1_OFF();							\
    DEBUG_S2_OFF();							\
    DEBUG_S3_OFF();							\
 }

#define DEBUG_SERVO2_INIT() {					\
    /* S4: PC9 */						\
    GPIO_InitTypeDef GPIO_InitStructure;			\
    GPIOC->BSRR = GPIO_Pin_9;					\
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		\
    GPIO_Init(GPIOC, &GPIO_InitStructure);			\
    /* S5: PB8 and S6: PB9 */					\
    GPIOB->BSRR = GPIO_Pin_8 | GPIO_Pin_9;			\
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		\
    GPIO_Init(GPIOB, &GPIO_InitStructure);			\
    DEBUG_S4_OFF();						\
    DEBUG_S5_OFF();						\
    DEBUG_S6_OFF();						\
  }


#endif /* MY_DEBUG_SERVO_H */
