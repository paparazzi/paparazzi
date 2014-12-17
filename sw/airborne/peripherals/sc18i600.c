#include "peripherals/sc18is600.h"

struct Sc18Is600 sc18is600;

void sc18is600_init(void)
{

  sc18is600.status = Sc18Is600Idle;
  sc18is600_arch_init();

}
