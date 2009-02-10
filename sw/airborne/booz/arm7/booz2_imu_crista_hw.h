#ifndef BOOZ_IMU_INT_HW_H
#define BOOZ_IMU_INT_HW_H

#include "std.h"

extern void booz2_imu_crista_hw_init(void);



#define Booz2ImuCristaHwPeriodic() {		\
    ADS8344_start();				\
  }

extern void ADS8344_start( void );

#endif /* BOOZ_IMU_INT_HW_H */

