#ifndef BOOZ_IMU_ASPIRIN_ARCH_H
#define BOOZ_IMU_ASPIRIN_ARCH_H

#include "booz_imu.h"

#include "led.h"

extern void booz_imu_aspirin_arch_init(void);
extern void adxl345_write_to_reg(uint8_t addr, uint8_t val);
extern void adxl345_clear_rx_buf(void);
extern void adxl345_start_reading_data(void);

#define OnI2CDone() {							\
    switch (imu_aspirin.status) {					\
    case AspirinStatusReadingGyro:					\
      {									\
									\
	int16_t gp = i2c2.buf[0]<<8 | i2c2.buf[1];			\
	int16_t gq = i2c2.buf[2]<<8 | i2c2.buf[3];			\
	int16_t gr = i2c2.buf[4]<<8 | i2c2.buf[5];			\
	RATES_ASSIGN(booz_imu.gyro_unscaled, gp, gq, gr);		\
	/*if (abs(booz_imu.gyro_unscaled.p) > 32000 || abs(booz_imu.gyro_unscaled.q) > 32000) LED_ON(4);*/ \
      }									\
      break;								\
    default:								\
      break;								\
    }									\
  }

#endif /* BOOZ_IMU_ASPIRIN_ARCH_H */
