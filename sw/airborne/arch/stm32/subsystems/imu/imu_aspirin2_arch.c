#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>


void imu_aspirin2_arch_init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Set baro CS (PC.13) as input with pullup to prevent floating input
   * Aspirin 2.1 has this pin connected to ground
   * Aspirin 2.2 has the ms5611 baro CS on this line
   */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}
