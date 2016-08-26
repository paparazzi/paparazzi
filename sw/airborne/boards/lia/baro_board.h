/** 
 *  Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for I2C
 *  
 * Edit by: Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 */

#ifndef BOARDS_LIA_BARO_H
#define BOARDS_LIA_BARO_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#ifndef MS5611_I2C_DEV
#define MS5611_I2C_DEV i2c2
#endif

/* MS5611 register addresses */
#define MS5611_SOFT_RESET       0x1E
#define MS5611_PROM_READ        0xA0
#define MS5611_START_CONV_D1    0x48 /* we use OSR=4096 for maximum resolution */
#define MS5611_START_CONV_D2    0x58 /* we use OSR=4096 for maximum resolution */
#define MS5611_ADC_READ         0x00
#define PROM_NB                 8

/* address can be 0xEC or 0xEE (CSB\ grounded = 0xEE) */
#define MS5611_SLAVE_ADDR 0xEE

enum ms5611_stat{
  MS5611_UNINIT,
  MS5611_RESET,
  MS5611_RESET_OK,
  MS5611_PROM,
  MS5611_IDLE,
  MS5611_CONV_D1,
  MS5611_CONV_D1_OK,
  MS5611_ADC_D1,
  MS5611_CONV_D2,
  MS5611_CONV_D2_OK,
  MS5611_ADC_D2
};



extern void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LIA_BARO_H */
