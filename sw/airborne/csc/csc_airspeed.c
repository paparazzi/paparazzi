#include "csc_airspeed.h"
#include "csc_ap_link.h"

float estimator_airspeed;

void csc_airspeed_periodic(void)
{
  csc_ap_link_send_airspeed(estimator_airspeed, 0);
}
