
#include "ppm.h"
#include "std.h"

#define PPM_NB_CHANNEL PPM_NB_PULSES

uint16_t ppm_pulses[PPM_NB_CHANNEL];
volatile bool_t ppm_valid;


void ppm_datalink( uint8_t throttle_mode,
                   int8_t roll,
                   int8_t pitch)
{
  int throttle = throttle_mode & 0xFC;
  int mode = throttle_mode & 0x03;

  ppm_pulses[RADIO_ROLL] = (roll * 4 + 1500) * 15;
  ppm_pulses[RADIO_PITCH] = (pitch * 4 + 1500) * 15;
  ppm_pulses[RADIO_THROTTLE] = (throttle * 4 + 1000) * 15;
  ppm_pulses[RADIO_MODE] = (1000 + mode * 500) * 15;

  ppm_valid = TRUE;
}


