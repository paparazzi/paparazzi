#ifndef IMU_ASPIRIN_ARCH_H
#define IMU_ASPIRIN_ARCH_H

#include "modules/imu/imu.h"
#include <libopencm3/stm32/gpio.h>

extern void imu_aspirin_arch_int_enable(void);
extern void imu_aspirin_arch_int_disable(void);

// gyro eoc
static inline int imu_aspirin_eoc(void)
{
  return (gpio_get(GPIOC, GPIO14) == 0);
}



#endif /* IMU_ASPIRIN_ARCH_H */
