#include "mb_current.h"

#include "adc.h"

static struct adc_buf mb_current_buf;

float mb_current_amp;

void mb_current_init(void) {
  adc_buf_channel(4, &mb_current_buf, 16);

}


void mb_current_periodic(void) {
  uint16_t cur_int = mb_current_buf.sum / mb_current_buf.av_nb_sample;
  //  uint16_t cur_int = adc0_val[0];
  mb_current_amp = cur_int * 1.;
}
