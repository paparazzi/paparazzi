

#include "mcu_periph/can_arch.h"
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time.h"
#include "stdio.h"
#include "string.h"



struct can_arch_periph {

};



void can_hw_init(void) {
  
}

int can_transmit_frame(struct pprzcan_frame* txframe __attribute__((__unused__)), struct pprzaddr_can* addr __attribute__((__unused__))) {
  // TODO
  return 0;
}