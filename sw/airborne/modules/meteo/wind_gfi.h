#ifndef WIND_GFI_H
#define WIND_GFI_H

#include "std.h"

enum pcf_stat {
  PCF_IDLE,
  PCF_SET_OE_LSB,
  PCF_READ_LSB,
  PCF_SET_OE_MSB,
  PCF_READ_MSB
};

void wind_gfi_init(void);
void wind_gfi_periodic(void);
void wind_gfi_event(void);

#endif
