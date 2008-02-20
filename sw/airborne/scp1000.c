#include "scp1000.h"

volatile uint32_t scp1000_pressure;
volatile uint8_t  scp1000_status;

void scp1000_init(void) {

  scp1000_status = SCP1000_STA_STOPPED;
  
  scp1000_hw_init();
}
