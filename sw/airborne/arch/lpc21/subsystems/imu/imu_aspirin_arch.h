#ifndef IMU_ASPIRIN_ARCH_H
#define IMU_ASPIRIN_ARCH_H

#include "subsystems/imu.h"
#include "std.h"
#include "LPC21xx.h"

#if !defined ASPIRIN_GYRO_EOC_IODIR && !defined ASPIRIN_GYRO_EOC_IOPIN && !defined ASPIRIN_GYRO_EOC_PIN
#define ASPIRIN_GYRO_EOC_IODIR IO0DIR
#define ASPIRIN_GYRO_EOC_IOPIN IO0PIN
#define ASPIRIN_GYRO_EOC_PIN 16
#endif

extern void imu_aspirin_arch_int_enable(void);
extern void imu_aspirin_arch_int_disable(void);

// gyro eoc
static inline int imu_aspirin_eoc(void)
{
  return bit_is_set(ASPIRIN_GYRO_EOC_IOPIN, ASPIRIN_GYRO_EOC_PIN);
}
#endif /* IMU_ASPIRIN_ARCH_H */
