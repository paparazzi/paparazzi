#include "ap_downlink.h"

#ifdef FBW
#ifndef TELEMETRY_MODE_FBW
#define TELEMETRY_MODE_FBW 0
#endif

uint8_t telemetry_mode_Fbw = TELEMETRY_MODE_FBW;
#endif

#ifdef AP
#ifndef TELEMETRY_MODE_AP
#define TELEMETRY_MODE_AP 0
#endif

uint8_t telemetry_mode_Ap =TELEMETRY_MODE_AP;
#endif

uint8_t downlink_nb_ovrn;
