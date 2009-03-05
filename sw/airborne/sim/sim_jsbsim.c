/* Definitions and declarations required to compile autopilot code on a
   i386 architecture */

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>
#include "std.h"
#include "inter_mcu.h"
#include "autopilot.h"
#include "estimator.h"
#include "gps.h"
#include "traffic_info.h"
#include "flight_plan.h"
#include "settings.h"
#include "nav.h"
#include "fw_h_ctl.h"
#include "fw_v_ctl.h"
#include "infrared.h"
#include "cam.h"
#include "commands.h"
#include "main_ap.h"
#include "ap_downlink.h"
#include "sim_uart.h"
#include "latlong.h"
#include "datalink.h"
#include "adc_generic.h"
#include "ppm.h"


/* Dummy definitions to replace the ones from the files not compiled in the
   simulator */
uint8_t ir_estim_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
bool_t rc_event_1, rc_event_2;
uint8_t gps_mode;
uint32_t  gps_itow;    /* ms */
int32_t   gps_alt;    /* cm       */
uint16_t  gps_gspeed;  /* cm/s     */
int16_t   gps_climb;  /* cm/s     */
int16_t   gps_course; /* decideg     */
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
int32_t gps_lat, gps_lon;
struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels = 0;
uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;
uint16_t gps_reset;
uint8_t gps_nb_ovrn, modem_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;
float alt_roll_pgain;
float roll_rate_pgain;
bool_t gpio1_status;
uint16_t adc_generic_val1;
uint16_t adc_generic_val2;
uint16_t ppm_pulses[ PPM_NB_PULSES ];
volatile bool_t ppm_valid;

uint8_t ac_id;

void ir_gain_calib(void) {}
void adc_buf_channel(uint8_t adc_channel __attribute__ ((unused)), struct adc_buf* s __attribute__ ((unused)), uint8_t av_nb_sample __attribute__ ((unused))) {}
void ubxsend_cfg_rst(uint16_t bbr __attribute__ ((unused)), uint8_t reset_mode __attribute__ ((unused))) {}
void adc_generic_init( void ) {}
void adc_generic_periodic( void ) {}
