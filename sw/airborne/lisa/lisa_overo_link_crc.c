#include "lisa/lisa_overo_link_crc.h"

struct LisaOveroLink overo_link;

void overo_link_init(void) {
  overo_link.status = IDLE;
  overo_link_arch_init();
}

