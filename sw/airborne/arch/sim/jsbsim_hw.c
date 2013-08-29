/* Definitions and declarations required to compile autopilot code on a
   i386 architecture */

#include "jsbsim_hw.h"

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>


/* Dummy definitions to replace the ones from the files not compiled in the simulator */
//uint8_t ir_estim_mode;
//uint8_t vertical_mode;
//uint8_t inflight_calib_mode;
//bool_t rc_event_1, rc_event_2;
uint8_t gps_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;
//float alt_roll_pgain;
//float roll_rate_pgain;
uint16_t adc_generic_val1;
uint16_t adc_generic_val2;
//uint16_t ppm_pulses[ PPM_NB_PULSES ];
//volatile bool_t ppm_valid;

uint8_t ac_id;

void update_bat(double bat) {
  electrical.vsupply = (int) (bat * 10.);
}

void adc_generic_init( void ) {}
void adc_generic_periodic( void ) {}
