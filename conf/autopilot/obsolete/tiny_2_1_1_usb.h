#define USE_USB_HIGH_PCLK

#include "tiny_2_1.h"

#undef DefaultVoltageOfAdc
#define DefaultVoltageOfAdc(adc) (0.0247311828*adc)
