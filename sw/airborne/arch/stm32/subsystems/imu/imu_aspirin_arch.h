#ifndef IMU_ASPIRIN_ARCH_H
#define IMU_ASPIRIN_ARCH_H

#include "subsystems/imu.h"

#include "led.h"

extern void imu_aspirin_arch_init(void);
extern void imu_aspirin_arch_int_enable(void);
extern void imu_aspirin_arch_int_disable(void);
extern void adxl345_write_to_reg(uint8_t addr, uint8_t val);
extern void adxl345_clear_rx_buf(void);
extern void adxl345_start_reading_data(void);


#endif /* IMU_ASPIRIN_ARCH_H */
