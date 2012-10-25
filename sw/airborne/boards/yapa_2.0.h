#include "boards/tiny_2.1.h"


// ADC / 1023 * 3.3V * (10k + 1k5) / 1k5
#undef DefaultVoltageOfAdc
#define DefaultVoltageOfAdc(adc) (0.0247311828*adc)
