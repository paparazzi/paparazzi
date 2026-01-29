

#include "mcu_periph/can_arch.h"
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time_arch.h"
#include "stdio.h"
#include "string.h"



struct can_arch_periph {

};



void can_hw_init(void) {
  
}

int can_transmit_frame(struct pprzcan_frame* txframe, struct pprzaddr_can* addr) {
  // TODO
  return 0;
}