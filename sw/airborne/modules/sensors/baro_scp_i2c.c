/** \file baro_scp_i2c.c
 *  \brief VTI SCP1000 I2C sensor interface
 *
 *   This reads the values for pressure and temperature from the VTI SCP1000 sensor through I2C.
 */


#include "baro_scp_i2c.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "subsystems/abi.h"
#include "led.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef SENSOR_SYNC_SEND
#warning set SENSOR_SYNC_SEND to use baro_scp_i2c
#endif


uint8_t  baro_scp_status;
uint32_t baro_scp_pressure;
uint16_t baro_scp_temperature;

struct i2c_transaction scp_trans;

#ifndef SCP_I2C_DEV
#define SCP_I2C_DEV i2c0
#endif

#define SCP1000_SLAVE_ADDR 0x22

static void baro_scp_start_high_res_measurement(void)
{
  /* switch to high resolution */
  scp_trans.buf[0] = SCP1000_OPERATION;
  scp_trans.buf[1] = SCP1000_HIGH_RES;
  i2c_transmit(&SCP_I2C_DEV, &scp_trans, SCP1000_SLAVE_ADDR, 2);
}

void baro_scp_init(void)
{
  baro_scp_status = BARO_SCP_UNINIT;
}

void baro_scp_periodic(void)
{

  if (baro_scp_status == BARO_SCP_UNINIT && sys_time.nb_sec > 1) {

    baro_scp_start_high_res_measurement();
    baro_scp_status = BARO_SCP_IDLE;
  } else if (baro_scp_status == BARO_SCP_IDLE) {

    /* init: start two byte temperature */
    scp_trans.buf[0] = SCP1000_TEMPOUT;
    baro_scp_status = BARO_SCP_RD_TEMP;
    i2c_transceive(&SCP_I2C_DEV, &scp_trans, SCP1000_SLAVE_ADDR, 1, 2);
  }
}

void baro_scp_event(void)
{

  if (scp_trans.status == I2CTransSuccess) {

    if (baro_scp_status == BARO_SCP_RD_TEMP) {

      /* read two byte temperature */
      baro_scp_temperature  = scp_trans.buf[0] << 8;
      baro_scp_temperature |= scp_trans.buf[1];
      if (baro_scp_temperature & 0x2000) {
        baro_scp_temperature |= 0xC000;
      }
      baro_scp_temperature *= 5;

      /* start one byte msb pressure */
      scp_trans.buf[0] = SCP1000_DATARD8;
      baro_scp_status = BARO_SCP_RD_PRESS_0;
      i2c_transceive(&SCP_I2C_DEV, &scp_trans, SCP1000_SLAVE_ADDR, 1, 1);
    }

    else if (baro_scp_status == BARO_SCP_RD_PRESS_0) {

      /* read one byte pressure */
      baro_scp_pressure = scp_trans.buf[0] << 16;

      /* start two byte lsb pressure */
      scp_trans.buf[0] = SCP1000_DATARD16;
      baro_scp_status = BARO_SCP_RD_PRESS_1;
      i2c_transceive(&SCP_I2C_DEV, &scp_trans, SCP1000_SLAVE_ADDR, 1, 2);
    }

    else if (baro_scp_status == BARO_SCP_RD_PRESS_1) {

      /* read two byte pressure */
      baro_scp_pressure |= scp_trans.buf[0] << 8;
      baro_scp_pressure |= scp_trans.buf[1];
      baro_scp_pressure *= 25;

      uint32_t now_ts = get_sys_time_usec();
      float pressure = (float) baro_scp_pressure;
      AbiSendMsgBARO_ABS(BARO_SCP_SENDER_ID, now_ts, pressure);
#ifdef SENSOR_SYNC_SEND
      DOWNLINK_SEND_SCP_STATUS(DefaultChannel, DefaultDevice, &baro_scp_pressure, &baro_scp_temperature);
#endif

      baro_scp_status = BARO_SCP_IDLE;
    }

    else { baro_scp_status = BARO_SCP_IDLE; }
  }
}
