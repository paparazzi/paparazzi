
/** \file dpicco.c
 *  \brief DigiPicco I2C sensor interface
 *
 *   This reads the values for humidity and temperature from the IST DigiPicco sensor through I2C.
 */


#include "dpicco.h"

#include "i2c.h"
#include "led.h"

#include <string.h>

uint8_t dpicco_status;

uint16_t dpicco_val[DPICCO_NB_SENSOR];

float dpicco_humid;
float dpicco_temp;

#define DPICCO_SLAVE_ADDR 0xF0


volatile bool_t dpicco_i2c_done;

void dpicco_init( void ) {
  uint8_t i;
  for (i=0; i< DPICCO_NB_SENSOR; i++) {
    dpicco_val[i] = 0;
  }
  dpicco_status = DPICCO_IDLE;
  dpicco_i2c_done = TRUE;
}

void dpicco_periodic( void ) {
  if (dpicco_i2c_done) {
    if (dpicco_status == DPICCO_IDLE) {
      /* init first read */
      dpicco_status = DPICCO_MEASURING_RD;
      dpicco_i2c_done = FALSE;
      i2c_receive(DPICCO_SLAVE_ADDR, 4, &dpicco_i2c_done);
      
//      LED_ON(2);
    }
    else if (dpicco_status == DPICCO_MEASURING_RD) {
      /* get data */
      dpicco_val[0] = (i2c_buf[0]<<8) | i2c_buf[1];
      dpicco_val[1] = (i2c_buf[2]<<8) | i2c_buf[3];
      
      /* start next one right away */
      dpicco_i2c_done = FALSE;
      i2c_receive(DPICCO_SLAVE_ADDR, 4, &dpicco_i2c_done);
      
//      LED_TOGGLE(2);
    }
  }
}
