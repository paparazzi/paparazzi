#include <caml/mlvalues.h>
#include <inttypes.h>

uint16_t adc_generic_val1;
uint16_t adc_generic_val2;

void adc_generic_init(void)
{
}

void adc_generic_periodic(void)
{
}

value update_adc1(value adc1)
{
  adc_generic_val1 = Int_val(adc1);
  return Val_unit;
}
