#include "mcu_periph/sys_time.h"


#include "mcu.h"

struct sys_time sys_time;

uint8_t sys_time_register_timer(uint32_t duration, sys_time_cb cb) {

  uint32_t start_time = sys_time.nb_tick;
  for (int i = 0; i< SYS_TIME_NB_TIMER; i++) {
    if (!sys_time.timer[i].in_use) {
      sys_time.timer[i].cb         = cb;
      sys_time.timer[i].elapsed    = FALSE;
      sys_time.timer[i].end_time   = start_time + duration;
      sys_time.timer[i].duration   = duration;
      sys_time.timer[i].in_use     = TRUE;
      return i;
    }
  }
  return -1;
}

void sys_time_cancel_timer(uint8_t id) {
  sys_time.timer[id].in_use     = FALSE;
#if 0
  sys_time.timer[id].cb         = NULL;
  sys_time.timer[id].elapsed    = FALSE;
  sys_time.timer[id].end_time   = 0;
  sys_time.timer[id].duration   = 0;
#endif
}

// FIXME: race condition ??
void sys_time_update_timer(uint8_t id, uint32_t duration) {
  mcu_int_disable();
  sys_time.timer[id].end_time -= (sys_time.timer[id].duration - duration);
  sys_time.timer[id].duration = duration;
  mcu_int_enable();
}
