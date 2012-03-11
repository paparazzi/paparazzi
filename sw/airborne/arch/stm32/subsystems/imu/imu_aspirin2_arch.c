#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>




// gyro interupt handler
void exti15_10_irq_handler(void);

void imu_aspirin2_arch_int_enable(void) {
/*
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
*/
}

void imu_aspirin2_arch_int_disable(void) {
/*
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
*/
}

void imu_aspirin2_arch_init(void) {
/*
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  // Set "mag ss" and "mag reset" as floating inputs ------------------------
  // "mag ss"    (PC12) is shorted to I2C2 SDA
  // "mag reset" (PC13) is shorted to I2C2 SCL
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Gyro --------------------------------------------------------------------
  // set "eeprom ss" as floating input (on PC14) = gyro int          ---------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  // configure external interrupt exti15_10 on PC14( gyro int )
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ifdef ASPIRIN_USE_GYRO_INT
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
*/

}



// Gyro data ready
void exti15_10_irq_handler(void) {
  // clear EXTI
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line14);

/*
  imu_aspirin.gyro_eoc = TRUE;
  imu_aspirin.status = AspirinStatusReadingGyro;
*/
}

