#include <string.h>

#include "enose.h"

#include "i2c.h"
#include "adc.h"
#include CONFIG


uint8_t enose_status;

uint8_t  enose_heat[ENOSE_NB_SENSOR];
uint16_t enose_val[ENOSE_NB_SENSOR];

#define ENOSE_SLAVE_ADDR 0xAE
#define ENOSE_PWM_ADDR   0x06
#define ENOSE_DATA_ADDR  0x00

#define ENOSE_HEAT_INIT 237

uint8_t enose_conf_requested;

volatile bool_t enose_i2c_done;


static struct adc_buf buf_PID;

void enose_init( void ) {
  uint8_t i;
  for (i=0; i< ENOSE_NB_SENSOR; i++) {
    enose_heat[i] = ENOSE_HEAT_INIT;
    enose_val[i] = 0;
  }
  enose_status = ENOSE_IDLE;
  enose_conf_requested = TRUE;
  enose_i2c_done = TRUE;

  adc_buf_channel(ADC_CHANNEL_PID, &buf_PID, ADC_CHANNEL_PID_NB_SAMPLES);
}


void enose_set_heat(uint8_t no_sensor, uint8_t value) {
  enose_heat[no_sensor] = value;
  enose_conf_requested = TRUE;
}

#include "led.h"

uint16_t enose_PID_val;


void enose_periodic( void ) {
  enose_PID_val = buf_PID.sum / buf_PID.av_nb_sample;

  if (enose_i2c_done) {
    if (enose_conf_requested) {
      const uint8_t msg[] = { ENOSE_PWM_ADDR, enose_heat[0], enose_heat[1], enose_heat[2] };
      memcpy(i2c_buf, msg, sizeof(msg));
      i2c_transmit(ENOSE_SLAVE_ADDR, sizeof(msg), &enose_i2c_done);
      enose_i2c_done = FALSE;
      enose_conf_requested = FALSE;
    }
    else if (enose_status == ENOSE_IDLE) {
      LED_OFF(2);
      enose_status = ENOSE_MEASURING_WR;
      const uint8_t msg[] = { ENOSE_DATA_ADDR };  
      memcpy(i2c_buf, msg, sizeof(msg));
      i2c_transmit(ENOSE_SLAVE_ADDR, sizeof(msg), &enose_i2c_done);
      enose_i2c_done = FALSE;
    }
    else if (enose_status == ENOSE_MEASURING_WR) {
      enose_status = ENOSE_MEASURING_RD;
      i2c_receive(ENOSE_SLAVE_ADDR, 6, &enose_i2c_done);
      enose_i2c_done = FALSE;
    }
    else if (enose_status == ENOSE_MEASURING_RD) {
      LED_ON(2);
      enose_val[0] = (i2c_buf[0]<<8) | i2c_buf[1];
      enose_val[1] = (i2c_buf[2]<<8) | i2c_buf[3];
      enose_val[2] = (i2c_buf[4]<<8) | i2c_buf[5];
      enose_status = ENOSE_IDLE;
    }
  }
}
