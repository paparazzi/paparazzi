#ifndef MY_GPIO_ARCH_H
#define MY_GPIO_ARCH_H

#include <stm32/gpio.h>
#include <stm32/rcc.h>

#define GPIO_ARCH_SET_SPI_CS_HIGH()					\
{									\
  GPIO_InitTypeDef GPIO_InitStructure;					\
  /* initialise peripheral clock for port B */				\
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);		\
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				\
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			\
  GPIO_Init(GPIOB, &GPIO_InitStructure);				\
  /* set port B pin 12 to be high */					\
  GPIO_WriteBit(GPIOB, GPIO_Pin_12 , Bit_SET );				\
}


#endif /* MY_GPIO_ARCH_H */
