#include "booz2_imu_crista.h"

void booz2_imu_impl_init(void) {

  ADS8344_available = FALSE;

  booz2_imu_crista_hw_init();

}

void booz2_imu_periodic(void) {

  Booz2ImuCristaHwPeriodic();

}
