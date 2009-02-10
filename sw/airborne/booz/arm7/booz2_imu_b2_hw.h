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




#endif /* BOOZ2_IMU_B2_HW_H */
