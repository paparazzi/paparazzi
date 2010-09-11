
#include "ppm.h"
#include "std.h"
#include "sys_time.h"


#include "radio_control/booz_radio_control_ppm.h"
#include BOARD_CONFIG


uint16_t ppm_pulses[PPM_NB_CHANNEL];
volatile bool_t ppm_valid;

////////////////////////////////////////////////
// RADIO_CONTROL_NB_CHANNEL  == PPM_NB_CHANNEL ?

void ppm_init ( void ) 
{
  booz_radio_control_ppm_arch_init();
  ppm_valid = FALSE;
}

void ppm_copy( void )
{
  ppm_valid = booz_radio_control_ppm_frame_available;

  for (int i=0;i<PPM_NB_CHANNEL;i++)
  {
    ppm_pulses[i] = booz_radio_control_ppm_pulses[i]; 
  }
}
