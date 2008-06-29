#include <caml/mlvalues.h>
#include "adc_generic.h"

#ifdef USE_ADC_GENERIC
uint16_t adc_generic_val1;
uint16_t adc_generic_val2;
#endif

void adc_generic_init( void ) {
}

void adc_generic_periodic( void ) {
}

value update_adc1(value adc1) {
  adc_generic_val1 = Int_val(adc1);
  return Val_unit;
}
