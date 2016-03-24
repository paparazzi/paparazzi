#include <string.h>

#include "modules/enose/enose.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG


uint8_t enose_status;
uint8_t  enose_heat[ENOSE_NB_SENSOR];
uint16_t enose_val[ENOSE_NB_SENSOR];
uint16_t enose_PID_val;

#define ENOSE_SLAVE_ADDR 0xAE
#define ENOSE_PWM_ADDR   0x06
#define ENOSE_DATA_ADDR  0x00
#define ENOSE_HEAT_INIT 237

static uint8_t enose_conf_requested;
static volatile bool enose_i2c_done;
static struct adc_buf buf_PID;


void enose_init(void)
{
  uint8_t i;
  for (i = 0; i < ENOSE_NB_SENSOR; i++) {
    enose_heat[i] = ENOSE_HEAT_INIT;
    enose_val[i] = 0;
  }
  enose_status = ENOSE_IDLE;
  enose_conf_requested = true;
  enose_i2c_done = true;

#ifdef ADC_CHANNEL_PID
  adc_buf_channel(ADC_CHANNEL_PID, &buf_PID, ADC_CHANNEL_PID_NB_SAMPLES);
#endif
}


void enose_set_heat(uint8_t no_sensor, uint8_t value)
{
  enose_heat[no_sensor] = value;
  enose_conf_requested = true;
}


#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

void enose_periodic(void)
{
  enose_PID_val = buf_PID.sum / buf_PID.av_nb_sample;

  if (enose_i2c_done) {
    if (enose_conf_requested) {
      const uint8_t msg[] = { ENOSE_PWM_ADDR, enose_heat[0], enose_heat[1], enose_heat[2] };
      memcpy((void *)i2c0_buf, msg, sizeof(msg));
      i2c0_transmit(ENOSE_SLAVE_ADDR, sizeof(msg), &enose_i2c_done);
      enose_i2c_done = false;
      enose_conf_requested = false;
    } else if (enose_status == ENOSE_IDLE) {
      enose_status = ENOSE_MEASURING_WR;
      const uint8_t msg[] = { ENOSE_DATA_ADDR };
      memcpy((void *)i2c0_buf, msg, sizeof(msg));
      i2c0_transmit(ENOSE_SLAVE_ADDR, sizeof(msg), &enose_i2c_done);
      enose_i2c_done = false;
    } else if (enose_status == ENOSE_MEASURING_WR) {
      enose_status = ENOSE_MEASURING_RD;
      i2c0_receive(ENOSE_SLAVE_ADDR, 6, &enose_i2c_done);
      enose_i2c_done = false;
    } else if (enose_status == ENOSE_MEASURING_RD) {
      uint16_t val = (i2c0_buf[0] << 8) | i2c0_buf[1];
      if (val < 5000) {
        enose_val[0] = val;
      }
      val = (i2c0_buf[2] << 8) | i2c0_buf[3];
      if (val < 5000) {
        enose_val[1] = val;
      }
      val = (i2c0_buf[4] << 8) | i2c0_buf[5];
      if (val < 5000) {
        enose_val[2] = val;
      }
      enose_status = ENOSE_IDLE;
    }
  }
  DOWNLINK_SEND_ENOSE_STATUS(DefaultChannel, DefaultDevice, &enose_val[0], &enose_val[1], &enose_val[2], &enose_PID_val,
                             3, enose_heat);
}
