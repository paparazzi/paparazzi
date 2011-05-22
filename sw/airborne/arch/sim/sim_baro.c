#include <stdlib.h>
#include "estimator.h"
#include "subsystems/nav.h"
#include "subsystems/gps.h"
#include "baro_MS5534A.h"

bool_t alt_baro_enabled;
bool_t spi_message_received;
bool_t baro_MS5534A_available;
uint32_t baro_MS5534A_pressure;
uint32_t baro_MS5534A_ground_pressure;
uint16_t baro_MS5534A_temp;
float baro_MS5534A_r;
float baro_MS5534A_sigma2;
float baro_MS5534A_z;

void baro_MS5534A_init(void) {
  baro_MS5534A_ground_pressure = 100000;

  baro_MS5534A_r = 10.;
  baro_MS5534A_sigma2 = 1;
}

// void baro_MS5534A_reset(void);

void baro_MS5534A_send(void) {
  static bool_t even = FALSE;
  even = !even;

  spi_message_received = even;
}

void baro_MS5534A_event_task( void ) {
  baro_MS5534A_pressure = baro_MS5534A_ground_pressure - (gps.hmsl/1000.-ground_alt) / 0.08 + ((10.*random()) / RAND_MAX);
  baro_MS5534A_temp = 10 + estimator_z;
  baro_MS5534A_available = TRUE;
}
