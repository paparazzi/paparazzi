#include "subsystems/electrical.h"

#include "mcu_periph/adc.h"
#include "commands.h"

#include "generated/airframe.h"
#include BOARD_CONFIG

struct Electrical electrical;

static struct {
  struct adc_buf vsupply_adc_buf;
#ifdef ADC_CHANNEL_CURRENT
  struct adc_buf current_adc_buf;
#endif
} electrical_priv;

#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) DefaultVoltageOfAdc(adc)
#endif
#ifndef MilliAmpereOfAdc
#define MilliAmpereOfAdc(adc) DefaultMilliAmpereOfAdc(adc)
#endif


void electrical_init(void) {
  electrical.vsupply = 0;
  electrical.current = 0;

  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &electrical_priv.vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#ifdef ADC_CHANNEL_CURRENT
  adc_buf_channel(ADC_CHANNEL_CURRENT, &electrical_priv.current_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif
}

void electrical_periodic(void) {
#ifndef SITL
  electrical.vsupply = VoltageOfAdc((10*(electrical_priv.vsupply_adc_buf.sum/electrical_priv.vsupply_adc_buf.av_nb_sample)));
#endif

#ifdef ADC_CHANNEL_CURRENT
#ifndef SITL
      electrical.current = MilliAmpereOfAdc((electrical_priv.current_adc_buf.sum/electrical_priv.current_adc_buf.av_nb_sample));
#endif
#else
#if defined MILLIAMP_AT_FULL_THROTTLE && defined COMMAND_THROTTLE
      electrical.current = ((float)commands[COMMAND_THROTTLE]) * ((float)MILLIAMP_AT_FULL_THROTTLE) / ((float)MAX_PPRZ);
#endif
#endif /* ADC_CHANNEL_CURRENT */

}
