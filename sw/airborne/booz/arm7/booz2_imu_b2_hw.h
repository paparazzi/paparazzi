#ifndef BOOZ2_IMU_B2_HW_H
#define BOOZ2_IMU_B2_HW_H

/*

  MAX1168 SPI ADC connected on SPI1 
  SS on P0.20
  EOC on P0.16 ( EINT0 )

  PNI mag on same bus
  SS on p1.28
  EOC P0.30 ( EINT3 )
  RESET P1.19

*/

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"  
#include "spi_hw.h"

//#include "booz2_debug.h"


extern void booz2_imu_b2_hw_init(void);

#define Booz2ImuSetSpi8bits() { \
  SSPCR0 &= (~(0xF << 0)); \
  SSPCR0 |= (0x07 << 0); /* data size : 8 bits */ \
}

#define Booz2ImuSetSpi16bits() { \
  SSPCR0 &= (~(0xF << 0)); \
  SSPCR0 |= (0x0F << 0); /* data size : 16 bits */ \
}


#ifdef USE_MICROMAG
#define Booz2ImuSpiEvent(_handler1,_handler2) { \
  if (do_booz2_max1168_read && booz2_imu_spi_selected == BOOZ2_SPI_NONE) { \
    Booz2ImuSetSpi16bits(); \
    booz2_imu_spi_selected = BOOZ2_SPI_SLAVE_MAX1168; \
    do_booz2_max1168_read = false; \
    _handler1(); \
  } \
  if (do_booz2_micromag_read && booz2_imu_spi_selected == BOOZ2_SPI_NONE) { \
    Booz2ImuSetSpi8bits(); \
    booz2_imu_spi_selected = BOOZ2_SPI_SLAVE_MM; \
    do_booz2_micromag_read = false; \
  } \
  if (booz2_imu_spi_selected == BOOZ2_SPI_SLAVE_MM) _handler2();\
}
#else // NO MICROMAG
#define Booz2ImuSpiEvent(_handler) { \
  booz2_imu_spi_selected = BOOZ2_SPI_SLAVE_MAX1168; \
  do_booz2_max1168_read = false; \
  _handler(); \
}
#endif



#endif /* BOOZ2_IMU_B2_HW_H */
