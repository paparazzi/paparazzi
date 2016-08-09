#ifndef NPS_IVY
#define NPS_IVY

#include "nps_fdm.h"
#include "nps_sensors.h"

extern void nps_ivy_init(char *ivy_bus);
extern void nps_ivy_display(struct NpsFdm* fdm_ivy, struct NpsSensors* sensors_ivy);
extern void nps_ivy_send_WORLD_ENV_REQ(void);

void* ivy_main_loop(void* data __attribute__((unused)));

#endif /* NPS_IVY */
