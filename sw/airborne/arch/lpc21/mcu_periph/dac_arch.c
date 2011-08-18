#include "mcu_periph/dac.h"

/* turn on DAC pins */
void dac_init(void) {
  PINSEL1 |= 2 << 18;
}
