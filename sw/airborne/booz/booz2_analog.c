#include "booz2_analog.h"

// battery on AD0.3 on P0.30
// baro    on AD0.1 on P0.28

#define CHAN_BAT  3
#define CHAN_BARO 1


void booz2_analog_init( void ) {

  booz2_analog_init_hw();

}

