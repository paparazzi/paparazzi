/** 
 *  Generic barometer interface, assuming the barometer is read through Aspirin IMU directly
 *  
 * Edit by: Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 */

#include "subsystems/sensors/baro.h"
#include "baro_board.h"
#include "led.h"

#ifdef DEBUG
#pragma message "Generic baro driver"
#endif

struct Baro baro;
struct BaroBoard baro_board;
struct i2c_transaction baro_trans;
struct bmp085_baro_calibration calibration;

struct Baro baro;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  baro_board.status = LBS_UNINITIALIZED;
  bmp085_baro_read_calibration();

  /* STM32 specific (maybe this is a LISA/M specific driver anyway?) */
  gpio_clear(GPIOB, GPIO0);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
}

void baro_periodic(void) {}

void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void)){
      b_abs_handler();
      b_diff_handler();
}
