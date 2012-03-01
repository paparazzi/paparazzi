#ifndef IMU_ASPIRIN2_ARCH_H
#define IMU_ASPIRIN2_ARCH_H

#include "subsystems/imu.h"
#include <libopencm3/stm32/f1/gpio.h>



extern void imu_aspirin2_arch_int_enable(void);
extern void imu_aspirin2_arch_int_disable(void);

/*
static inline int imu_aspirin2_eoc(void)
{
  return !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
}
*/


#endif
