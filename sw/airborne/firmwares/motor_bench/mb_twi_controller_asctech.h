#ifndef MB_TWI_CONTROLLER_ASCTECH_H
#define MB_TWI_CONTROLLER_ASCTECH_H

#include "std.h"

extern void mb_twi_controller_init(void);
extern void mb_twi_controller_set(float throttle);
extern void mb_twi_controller_set_raw(uint8_t throttle);

#define MB_TWI_CONTROLLER_COMMAND_NONE     0
#define MB_TWI_CONTROLLER_COMMAND_TEST     1
#define MB_TWI_CONTROLLER_COMMAND_REVERSE  2
#define MB_TWI_CONTROLLER_COMMAND_SET_ADDR 3


extern bool_t  mb_twi_controller_asctech_command;
extern uint8_t mb_twi_controller_asctech_command_type;
extern uint8_t mb_twi_controller_asctech_addr;
extern uint8_t mb_twi_controller_asctech_new_addr;

#define mb_twi_controller_asctech_SetCommand(value) { \
    mb_twi_controller_asctech_command = TRUE;       \
    mb_twi_controller_asctech_command_type = value;   \
  }

#define mb_twi_controller_asctech_SetAddr(value) {      \
    mb_twi_controller_asctech_command = TRUE;       \
    mb_twi_controller_asctech_command_type = MB_TWI_CONTROLLER_COMMAND_SET_ADDR; \
    mb_twi_controller_asctech_new_addr = value;       \
  }


#endif /* MB_TWI_CONTROLLER_ASCTECH_H */
