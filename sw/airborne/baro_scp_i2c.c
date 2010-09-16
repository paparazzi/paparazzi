
/** \file baro_scp_i2c.c
 *  \brief VTI SCP1000 I2C sensor interface
 *
 *   This reads the values for pressure and temperature from the VTI SCP1000 sensor through I2C.
 */


#include "baro_scp_i2c.h"

#include "sys_time.h"
#include "i2c.h"
#include "led.h"

#include <string.h>

uint8_t  baro_scp_status;
uint32_t baro_scp_pressure;
uint16_t baro_scp_temperature;
bool_t   baro_scp_available;
volatile bool_t baro_scp_i2c_done;

#define SCP1000_SLAVE_ADDR 0x22


static void baro_scp_start_high_res_measurement(void) {
  /* write 0x0A to 0x03 */
  i2c0_buf[0] = 0x03;
  i2c0_buf[1] = 0x0A;
  i2c0_transmit(SCP1000_SLAVE_ADDR, 2, &baro_scp_i2c_done);
}

void baro_scp_init( void ) {
  baro_scp_status = BARO_SCP_UNINIT;
  baro_scp_i2c_done = FALSE;
}

void baro_scp_periodic( void ) {
  if (baro_scp_status == BARO_SCP_UNINIT && cpu_time_sec > 1) {
    baro_scp_start_high_res_measurement();
    baro_scp_status =  BARO_SCP_IDLE;
  } else if (baro_scp_i2c_done) {
    if (baro_scp_status == BARO_SCP_IDLE) {

      /* initial measurement */

      /* start two byte temperature */
      i2c0_buf[0] = 0x81;
      baro_scp_status = BARO_SCP_RD_TEMP;
      baro_scp_i2c_done = FALSE;
      i2c0_transceive(SCP1000_SLAVE_ADDR, 1, 2, &baro_scp_i2c_done);
//LED_ON(2);
    }

    else if (baro_scp_status == BARO_SCP_RD_TEMP) {

      /* read two byte temperature */
      baro_scp_temperature  = i2c0_buf[0] << 8;
      baro_scp_temperature |= i2c0_buf[1];
      if (baro_scp_temperature & 0x2000) {
        baro_scp_temperature |= 0xC000;
      }
      baro_scp_temperature *= 5;

      /* start one byte msb pressure */
      i2c0_buf[0] = 0x7F;
      baro_scp_status = BARO_SCP_RD_PRESS_0;
      baro_scp_i2c_done = FALSE;
      i2c0_transceive(SCP1000_SLAVE_ADDR, 1, 1, &baro_scp_i2c_done);
    }

    else if (baro_scp_status == BARO_SCP_RD_PRESS_0) {

      /* read one byte pressure */
      baro_scp_pressure = i2c0_buf[0] << 16;

      /* start two byte lsb pressure */
      i2c0_buf[0] = 0x80;
      baro_scp_status = BARO_SCP_RD_PRESS_1;
      baro_scp_i2c_done = FALSE;
      i2c0_transceive(SCP1000_SLAVE_ADDR, 1, 2, &baro_scp_i2c_done);
    }

    else if (baro_scp_status == BARO_SCP_RD_PRESS_1) {

      /* read two byte pressure */
      baro_scp_pressure |= i2c0_buf[0] << 8;
      baro_scp_pressure |= i2c0_buf[1];
      baro_scp_pressure *= 25;
      baro_scp_available = TRUE;
//LED_TOGGLE(2);

      /* start two byte temperature */
      i2c0_buf[0] = 0x81;
      baro_scp_status = BARO_SCP_RD_TEMP;
      baro_scp_i2c_done = FALSE;
      i2c0_transceive(SCP1000_SLAVE_ADDR, 1, 2, &baro_scp_i2c_done);
    }     
  }
}

