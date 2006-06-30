#ifndef LINK_IMU_H
#define LINK_IMU_H

#include <inttypes.h>

#include "frames.h"

struct imu_state {
  float rates[AXIS_NB];
  float eulers[AXIS_NB];
};


extern struct imu_state link_imu_state;
extern void link_imu_init ( void );

#ifdef IMU

extern volatile uint8_t spi0_data_available;
extern void link_imu_send( void );

#endif /* IMU */

#ifdef FBW


void link_imu_event_task( void );


#endif /* FBW */

#endif /* LINK_IMU_H */
