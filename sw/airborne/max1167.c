#include "max1167.h"


//volatile uint8_t max1167_data_available;
volatile uint8_t max1167_status;
uint16_t max1167_values[MAX1167_NB_CHAN];

extern void max1167_init( void ) {
  uint8_t i;
  for (i=0; i<MAX1167_NB_CHAN; i++)
    max1167_values[i] = 0;

  //  max1167_data_available = FALSE;
  max1167_status = STA_MAX1167_IDLE;
}
