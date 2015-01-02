#include "enose/enose.h"

uint8_t enose_status;
uint8_t  enose_heat[ENOSE_NB_SENSOR];
uint16_t enose_val[ENOSE_NB_SENSOR];
uint16_t enose_PID_val;

uint16_t nominal_val[ENOSE_NB_SENSOR] = {1500, 1500, 4000};
uint16_t min_val[ENOSE_NB_SENSOR] = {1100, 1200, 2500};

void enose_init(void)
{
  int i;
  for (i = 0; i < ENOSE_NB_SENSOR; i++) {
    enose_val[i] = nominal_val[i];
  }
}
void enose_set_heat(uint8_t no_sensor, uint8_t value) { }
void enose_periodic(void)
{
  int i;
  for (i = 0; i < ENOSE_NB_SENSOR; i++) {
    if (enose_val[i] < min_val[i]) {
      enose_val[i] = min_val[i];
    }
    int d = nominal_val[i] - enose_val[i];
    enose_val[i] += d / 10.;
  }
}
