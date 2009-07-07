#include "booz_radio_control.h"

bool_t   rc_spk_parser_status;
uint8_t  rc_spk_parser_idx;
uint8_t  rc_spk_parser_buf[RADIO_CONTROL_NB_CHANNEL*2];
const int16_t rc_spk_throw[RADIO_CONTROL_NB_CHANNEL] = RC_SPK_THROWS;
