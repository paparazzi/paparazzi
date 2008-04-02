#ifndef MB_TWI_CONTROLLER_ASCTECH_H
#define MB_TWI_CONTROLLER_ASCTECH_H

extern bool_t  mb_twi_controller_asctech_command;
extern uint8_t mb_twi_controller_asctech_command_type;

#define mb_twi_controller_asctech_SetCommand(value) { \
    mb_twi_controller_asctech_command = TRUE;	      \
    mb_twi_controller_asctech_command_type = value;   \
  }

#endif /* MB_TWI_CONTROLLER_ASCTECH_H */
