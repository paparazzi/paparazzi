#include "booz2_imu_b2.h"

void booz2_imu_impl_init(void) {
  
  booz2_imu_b2_hw_init();

  booz2_max1168_init();
#ifdef USE_MICROMAG
  micromag_init();
#endif

}


