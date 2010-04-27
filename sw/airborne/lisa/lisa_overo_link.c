#include "lisa/lisa_overo_link.h"

struct LisaOveroLink overo_link;

void overo_link_init(void) {
  overo_link.status = LOST;
  overo_link_arch_init();
}

