#ifndef ALT_VFILTER_H
#define ALT_VFILTER_H

#define STATE_SIZE 3

extern float alt_vf_z;
extern float alt_vf_zdot;
extern float alt_vf_bias;
extern float alt_vf_P[STATE_SIZE][STATE_SIZE];

extern float alt_vf_z_meas;

extern void alt_vf_init(float z, float zdot, float bias);
extern void alt_vf_predict(float accel);
extern void alt_vf_update_z(float z_meas);
extern void alt_vf_update_vz(float vz);

extern void alt_vf_periodic_task(void);


#ifdef USE_BARO_MS5534A

#include "baro_MS5534A.h"

#define AltVFliterEvent() { \
  if (spi_message_received) { \
    /* Got a message on SPI. */ \
    spi_message_received = FALSE; \
    baro_MS5534A_event_task(); \
    if (baro_MS5534A_available) { \
      baro_MS5534A_available = FALSE; \
      baro_MS5534A_z = ground_alt +((float)baro_MS5534A_ground_pressure - baro_MS5534A_pressure)*0.084; \
      if (alt_baro_enabled) { \
        alt_vf_update_z(-baro_MS5534A_z); \
      } \
    } \
  } \
}

#else
#define AltVFliterEvent() {}
#endif


#endif /* ALT_VFILTER_H */
