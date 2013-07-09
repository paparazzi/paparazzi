
#include "subsystems/sensors/baro.h"
#include "baro_board.h"
/*
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
*/

struct Baro baro;

void baro_init(void) {
  baro_ms5611_init();
}

void baro_periodic(void) {
    static uint8_t cnt;
    switch(cnt) {
      case 0:
      baro_ms5611_periodic();
      cnt++;
      break;
      case 1:
      baro_ms5611_d1();
      cnt++;
      break;
      case 2:
      baro_ms5611_d2();
      cnt = 0;
      break;
    }
}
