#ifndef IMU_H
#define IMU_H

#include <inttypes.h>

#ifdef SECTION_IMU_3DMG
extern int16_t roll, pitch, yaw;
#endif
extern int16_t roll_dot, pitch_dot, yaw_dot;

extern void imu_init ( void );
extern void imu_update ( void );
extern void imu_capture_neutral ( void );

#endif /* IMU_H */
