#ifndef MY_GPIO_ARCH_H
#define MY_GPIO_ARCH_H

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#define GPIO_ARCH_SET_SPI_CS_HIGH()					\
{									\
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);        \
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,                         \
	        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);                      \
  gpio_set(GPIOB, GPIO12);                                              \
}


#endif /* MY_GPIO_ARCH_H */
