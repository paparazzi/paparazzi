#include "radio_control.h"

pprz_t rc_values[RADIO_CTL_NB];
uint8_t rc_status;
pprz_t avg_rc_values[RADIO_CTL_NB];
uint8_t rc_values_contains_avg_channels = FALSE;
uint8_t time_sime_last_ppm;


