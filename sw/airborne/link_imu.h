#ifndef LINK_IMU_H
#define LINK_IMU_H

#include <inttypes.h>

#include "frames.h"

/* angles are transmitted as int16 : PI -> 16384 */
/* rates are transmitted as int16 : 1 PI s-1 -> 16384 */
#define ANGLE_PI 0X3FFF
#define RATE_PI_S 0X3FF

#define IMU_UNINIT  0
#define IMU_RUNNING 1
#define IMU_CRASHED 2

struct imu_state {
  int16_t rates[AXIS_NB];
  int16_t eulers[AXIS_NB];
  float cos_theta;
  float sin_theta;
  uint8_t status;
};


extern struct imu_state link_imu_state;
extern void link_imu_init ( void );

#ifdef IMU

extern volatile uint8_t spi0_data_available;
extern void link_imu_send( void );

#endif /* IMU */

#ifdef CONTROLLER


extern void link_imu_event_task( void );


#endif /* CONTROLLER */

#endif /* LINK_IMU_H */
