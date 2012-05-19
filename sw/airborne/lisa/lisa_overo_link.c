#include "lisa/lisa_overo_link.h"

struct LisaOveroLink overo_link;

void overo_link_init(void) {
  overo_link.status = IDLE;
  overo_link.timeout_cnt = OVERO_LINK_TIMEOUT-1;
  overo_link.msg_cnt = 0;
  overo_link.crc_err_cnt = 0;
  overo_link.crc_error = FALSE;
  overo_link.timeout = FALSE;
  overo_link_arch_init();
}









#ifdef USE_OVERO_LINK_TELEMETRY
#include <string.h>

uint8_t overo_link_telemetry_insert_idx;
uint8_t overo_link_telemetry_extract_idx;
uint8_t overo_link_telemetry_buf[OVERO_LINK_TELEMETRY_BUF_SIZE];

uint8_t overo_link_telemetry_get(char* buf, int len) {
  int8_t available = overo_link_telemetry_insert_idx - overo_link_telemetry_extract_idx;
  int8_t nb_put;
  /* we don't cross circular buffer end */
  if (available >= 0) {
    nb_put = Min(available, len);
    memcpy(buf, &overo_link_telemetry_buf[overo_link_telemetry_extract_idx], nb_put);
    overo_link_telemetry_extract_idx += nb_put;
  }
  /* we cross circular buffer end */
  else {
    available += OVERO_LINK_TELEMETRY_BUF_SIZE;
    nb_put = Min(available, len);
    uint8_t first_put =  OVERO_LINK_TELEMETRY_BUF_SIZE - 1 - overo_link_telemetry_extract_idx;
    memcpy(buf, &overo_link_telemetry_buf[overo_link_telemetry_extract_idx], first_put);
    overo_link_telemetry_extract_idx += nb_put;
    if (first_put < nb_put) {
      memcpy(&buf[first_put], &overo_link_telemetry_buf[0], nb_put - first_put);
      overo_link_telemetry_extract_idx -= OVERO_LINK_TELEMETRY_BUF_SIZE;
    }
  }
  return nb_put;
}

#endif /* USE_OVERO_LINK_TELEMETRY */
