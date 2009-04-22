#include "booz2_imu_b2.h"

uint8_t booz2_imu_spi_selected;

void booz2_imu_impl_init(void) {
  
  booz2_imu_spi_selected = BOOZ2_SPI_NONE;

  booz2_imu_b2_hw_init();

  booz2_max1168_init();

#ifdef USE_MICROMAG
  booz2_micromag_init();
#endif

}


void booz2_imu_periodic(void) {
  
  do_booz2_max1168_read = true;

}
