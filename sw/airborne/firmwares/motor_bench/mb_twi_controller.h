#ifndef MB_TWI_CONTROLLER_H
#define MB_TWI_CONTROLLER_H

#include "std.h"

extern void mb_twi_controller_init(void);

extern void mb_twi_controller_set(float throttle);

#define MB_TWI_CONTROLLER_MAX_CMD 65535
/*
  Slave address
  front = 0x52
  back  = 0x54
  right = 0x56
  left  = 0x58
*/
#define MB_TWI_CONTROLLER_ADDR 0x52

#endif /* MB_TWI_CONTROLLER_H */


