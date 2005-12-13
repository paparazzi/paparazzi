/* This file has been generated from /home/poine/work/paparazzi_savannah/paparazzi3/conf/radios/cockpitMM.xml */
/* Please DO NOT EDIT */

#ifndef RADIO_H
#define RADIO_H 

#define RADIO_NAME "cockpitMM"

#define RADIO_CTL_NB 7

#define RADIO_CTL_D 0
#define RADIO_ROLL RADIO_CTL_D
#define RADIO_CTL_C 1
#define RADIO_PITCH RADIO_CTL_C
#define RADIO_CTL_B 2
#define RADIO_YAW RADIO_CTL_B
#define RADIO_CTL_A 3
#define RADIO_THROTTLE RADIO_CTL_A
#define RADIO_CTL_G 4
#define RADIO_GAIN1 RADIO_CTL_G
#define RADIO_CTL_E 5
#define RADIO_MODE RADIO_CTL_E
#define RADIO_CTL_F 6
#define RADIO_CALIB RADIO_CTL_F


#define PPM_SYNC_MIN_LEN CLOCK_OF_US(3000)
#define PPM_SYNC_MAX_LEN CLOCK_OF_US(25000)
#define PPM_DATA_MIN_LEN CLOCK_OF_US(900)
#define PPM_DATA_MAX_LEN CLOCK_OF_US(2100)

#define NORMALISE_PPM() {\
  static uint8_t avg_cpt = 0; /* Counter for averaging */\
  int32_t tmp_radio;					 \
  tmp_radio = ppm_pulses[RADIO_ROLL] - CLOCK_OF_US(1498);		\
  rc_values[RADIO_ROLL] = tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/1/(float)(CLOCK_OF_US(1000-1498))) : (MIN_PPRZ/1/(float)(CLOCK_OF_US(2000-1498)))); \
  if (rc_values[RADIO_ROLL] > MAX_PPRZ) rc_values[RADIO_ROLL] = MAX_PPRZ; \
  else if (rc_values[RADIO_ROLL] < MIN_PPRZ) rc_values[RADIO_ROLL] = MIN_PPRZ; \
									\
  tmp_radio = ppm_pulses[RADIO_PITCH] - CLOCK_OF_US(1498);		\
  rc_values[RADIO_PITCH] = tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/1/(float)(CLOCK_OF_US(1000-1498))) : (MIN_PPRZ/1/(float)(CLOCK_OF_US(2000-1498)))); \
  if (rc_values[RADIO_PITCH] > MAX_PPRZ) rc_values[RADIO_PITCH] = MAX_PPRZ; \
  else if (rc_values[RADIO_PITCH] < MIN_PPRZ) rc_values[RADIO_PITCH] = MIN_PPRZ; \
									\
  tmp_radio = ppm_pulses[RADIO_YAW] - CLOCK_OF_US(1498);		\
  rc_values[RADIO_YAW] = tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/1/(float)(CLOCK_OF_US(1000-1498))) : (MIN_PPRZ/1/(float)(CLOCK_OF_US(2000-1498)))); \
  if (rc_values[RADIO_YAW] > MAX_PPRZ) rc_values[RADIO_YAW] = MAX_PPRZ; \
  else if (rc_values[RADIO_YAW] < MIN_PPRZ) rc_values[RADIO_YAW] = MIN_PPRZ; \
									\
  tmp_radio = ppm_pulses[RADIO_THROTTLE] - CLOCK_OF_US(1120);		\
  rc_values[RADIO_THROTTLE] = tmp_radio * (MAX_PPRZ / 1 / (float)(CLOCK_OF_US(2000-1120))); \
  if (rc_values[RADIO_THROTTLE] > MAX_PPRZ) rc_values[RADIO_THROTTLE] = MAX_PPRZ; \
  else if (rc_values[RADIO_THROTTLE] < 0) rc_values[RADIO_THROTTLE] = 0; \
									\
  tmp_radio = ppm_pulses[RADIO_GAIN1] - CLOCK_OF_US(1498);		\
  avg_last_radio[RADIO_GAIN1] += tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(1000-1498))) : (MIN_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(2000-1498)))); \
  tmp_radio = ppm_pulses[RADIO_MODE] - CLOCK_OF_US(1500);		\
  avg_last_radio[RADIO_MODE] += tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(1000-1500))) : (MIN_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(2000-1500)))); \
  tmp_radio = ppm_pulses[RADIO_CALIB] - CLOCK_OF_US(1500);		\
  avg_last_radio[RADIO_CALIB] += tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(1000-1500))) : (MIN_PPRZ/RC_AVG_PERIOD/(float)(CLOCK_OF_US(2000-1500)))); \
  avg_cpt++;								\
  if (avg_cpt == RC_AVG_PERIOD) {					\
    avg_cpt = 0;							\
    rc_values[RADIO_GAIN1] = avg_last_radio[RADIO_GAIN1];		\
    avg_last_radio[RADIO_GAIN1] = 0;					\
    if (rc_values[RADIO_GAIN1] > MAX_PPRZ) rc_values[RADIO_GAIN1] = MAX_PPRZ;	\
    else if (rc_values[RADIO_GAIN1] < MIN_PPRZ) rc_values[RADIO_GAIN1] = MIN_PPRZ; \
									\
    rc_values[RADIO_MODE] = avg_last_radio[RADIO_MODE];		\
    avg_last_radio[RADIO_MODE] = 0;					\
    if (rc_values[RADIO_MODE] > MAX_PPRZ) rc_values[RADIO_MODE] = MAX_PPRZ; \
    else if (rc_values[RADIO_MODE] < MIN_PPRZ) rc_values[RADIO_MODE] = MIN_PPRZ; \
									\
    rc_values[RADIO_CALIB] = avg_last_radio[RADIO_CALIB];		\
    avg_last_radio[RADIO_CALIB] = 0;					\
    if (rc_values[RADIO_CALIB] > MAX_PPRZ) rc_values[RADIO_CALIB] = MAX_PPRZ;	\
    else if (rc_values[RADIO_CALIB] < MIN_PPRZ) rc_values[RADIO_CALIB] = MIN_PPRZ; \
									\
    last_radio_contains_avg_channels = TRUE;				\
  }									\
}

#endif // RADIO_H
