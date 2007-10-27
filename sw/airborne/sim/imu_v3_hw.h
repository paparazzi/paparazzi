#ifndef IMU_V3_HW_H
#define IMU_V3_HW_H

#include "std.h"

//#include "booz_sensors_model.h"

extern void imu_v3_hw_init(void);

extern bool_t imu_data_available;

#define ImuEventCheckAndHandle(user_handler) {	\
    if (imu_data_available) {		\
      imu_data_available = FALSE;		\
      /* ImuUpdateGyros(); */			\
      /* ImuUpdateAccels();*/			\
      user_handler();				\
    }						\
  }


#define ImuPeriodic() {	}

#endif /* IMU_V3_HW_H */
