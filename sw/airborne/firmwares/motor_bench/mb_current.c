#include "mb_current.h"

#include "mcu_periph/adc.h"

static struct adc_buf mb_current_buf;

float mb_current_amp;

void mb_current_init(void)
{
  adc_buf_channel(4, &mb_current_buf, 16);

}


void mb_current_periodic(void)
{
  mb_current_amp = (float)mb_current_buf.sum * 0.00113607  - 2.8202;
}
