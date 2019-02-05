/*
 * Copyright (C) 2018 Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/sonar/sonar_parrot_minidrone.c
 *  @brief Parrot Minidrones common sonar driver
 *
 *  This file contains the driver for the sonar on a Parrot Minidrone
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <linux/input.h>
#include <errno.h> //Remove after it not needed anymore

#include "sonar_parrot_minidrone.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/spi.h"
#include "subsystems/abi.h"

#include <pthread.h>
#include "subsystems/datalink/downlink.h"

#include "mcu.h"
#include "boards/parrot_minidrone.h"

#include "filters/median_filter.h" // So one can enable a median filter on the data if needed
#ifdef SITL
#include "state.h"
#endif

static int verbose = 1; //Set it to 1 For debugging purpose e.g. of Debugging the ultra sound code

/* SIZEMAX: Maximum size of ADC DMA-able memory 256KB */
#define SIZEMAX   (1 << 18)

#define USND_IO_MAGIC 'u'
#define USND_SETSIZE        _IOW(USND_IO_MAGIC, 1, int)
#define USND_COPYSAMPLE     _IOR(USND_IO_MAGIC, 2, int*)
#define USND_SPI_LEN        _IOW(USND_IO_MAGIC, 3, int)
#define USND_SPI_DAT        _IOW(USND_IO_MAGIC, 4, int)
#define BATTERY             _IOR(USND_IO_MAGIC, 5, int)
#define TEMPERATURE         _IOR(USND_IO_MAGIC, 6, int)
#define BATTERY_INIT        _IO(USND_IO_MAGIC, 13)
#define TEMPERATURE_INIT    _IO(USND_IO_MAGIC, 15)

#define USND_IOC_MAXNR 14

// HAL_P6I is the Hardware Abstraction of te P6I , the hardware of a minidrone

#define HAL_P6I_INI_ECHO_THR  500
#define HAL_P6I_THRESHOLD_NOISE (50*4)
#define HAL_P6I_ULTRASOUND_FREQUENCY_ADC        (44100*3)
#define HAL_P6I_THRESHOLD_SUM_ECHO 1100

#define HAL_P6I_VMAX 2.5

#define SPI_MAX (64*4)
#define NB_PULSES_MAX (64)

#define HAL_P6I_FREQUENCY_NEW_DATA 32
#define FREQUENCY_WANTED 200 //in Hz

#define GPIO_US_POWER 56

#define SPI_PATTERN_DEFAULT 0xFF
#define NB_PULSES_DEFAULT NB_PULSES_MAX
#define NB_SAMPLES_DEFAULT 1024
#define NB_SAMPLES_MAX SIZEMAX

/** SONAR_PARROT_MINIDRONE_TRANSITION_HIGH_TO_LOW below this altitude in meters we should use mode 0 */
#define SONAR_PARROT_MINIDRONE_TRANSITION_HIGH_TO_LOW 0.8

/** SONAR_PARROT_MINIDRONE_TRANSITION_LOW_TO_HIGH above this altitude in meters we should use mode 1 */
#define SONAR_PARROT_MINIDRONE_TRANSITION_LOW_TO_HIGH 1.2

/** SONAR_PARROT_MINIDRONE_TRANSITION_COUNT number of samples before switching mode */
#define SONAR_PARROT_MINIDRONE_TRANSITION_COUNT 7

/** SONAR_PARROT_MINIDRONE_PEAK_THRESHOLD minimum samples from broadcast stop */
#define SONAR_PARROT_MINIDRONE_PEAK_THRESHOLD 250

// List of Voltages available
typedef enum _HAL_P6I_us_voltage_t {
  HAL_P6I_US_33DV = 0,
  HAL_P6I_US_08DV = 1,
} HAL_P6I_us_voltage_t;

// List of Modes available
typedef enum _HAL_P6I_us_mode_t {
  HAL_P6I_US_MODE1 = 0,
  HAL_P6I_US_MODE2 = 1,
} HAL_P6I_us_mode_t;

struct ultra_sound_context {
  char        *name_device;
  int         fd;
  int         nb_acquisition;
  uint16_t      nb_samples_by_acquisition_adc;
  uint8_t       nb_pulses;
  struct pollfd   fds;
  uint32_t     *adc_raw_data;
  uint32_t      adc_raw_data_size;
  uint32_t     *Tab_SPI;
  uint8_t       SPI_pattern;
  int         voltage_gpio;
  HAL_P6I_us_voltage_t voltage_mode;
};

/** Mode holds the current sonar mode
 * mode = 0 used at high altitude, uses 16 wave patterns
 * mode = 1 used at low altitude, uses 4 wave patterns
 * mode = 2
 * mode = 3
 */
static uint8_t mode;

//FIXME: add these modes
//enum pulse_mode {
//  PULSE_x4 = 0,
//  PULSE_x8,
//  PULSE_x16,
//  PULSE_12x_5x_DEPHASED,
//  PULSE_16x_4x_DEPHASED
//};

static uint8_t pulse_transition_counter;

/** The waveforms emitted by the sonar
 * waveform 0 is long pulse used at high altitude
 * waveform 1 is shorter pulse used at low altitude
 */

#define NB_ACQUISITION_DEFAULT 1 // default is 1

struct SonarParrotMinidrone sonar_parrot_minidrone;
struct MedianFilterFloat sonar_filt;

void *sonar_parrot_minidrone_read(void *data);

#ifdef USE_SONAR
static pthread_t sonar_parrot_minidrone_thread;
#endif

/** sonar_parrot_minidrone_init
 * Initialize de module
 */
void sonar_parrot_minidrone_init(void)
{
  mode = 0; // Initial mode is low altitude, most common scenario
  pulse_transition_counter = 0;

  sonar_parrot_minidrone.meas = 333;//TODO: set to 0 for real after debugging this driver
  sonar_parrot_minidrone.offset = 0;//TODO: add define

#if USE_SONAR //FIXME:USE_SONAR indicated if sonar should be USE(d) in INS solution not if (to) USE the device...
  /* Start sonar thread */
  if (pthread_create(&sonar_parrot_minidrone_thread, NULL, sonar_parrot_minidrone_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create sonar reading thread!\nfor now under telnet do: ulimit -s 512\nFeel free to improve...");
  } else {
	//Success so start filter init
    //init_median_filter_f(&sonar_filt, 3);
  }
#endif

}

/** sonar_parrot_minidrone_read
 * Read ADC value to update sonar measurement
 */
//For some good ideas, left from Bebop DRV  (struct ultra_sound_context *ctx)
void *sonar_parrot_minidrone_read(void *data __attribute__((unused)))
{

//*********** Basic things ***********
	struct ultra_sound_context ctx; // = { 0 }; //Maybe enable then there is no need for damn memset...

	long int  nb_acquisition = NB_ACQUISITION_DEFAULT;
	uint8_t   SPI_pattern = SPI_PATTERN_DEFAULT;
	uint8_t   nb_pulses = 6;//NB_PULSES_DEFAULT;
	uint16_t  nb_samples = 2;//NB_SAMPLES_DEFAULT;

	int ret = 0;
	int i = 0;

//*********** sonar_parrot_minidrone_open_init_context ***********
	memset(&ctx, 0, sizeof ctx);
	ctx.name_device                   = "/dev/ultra_snd";
	ctx.fd	                         = -1;
	ctx.voltage_gpio                  = GPIO_US_POWER;
	ctx.voltage_mode                  = HAL_P6I_US_33DV;
	ctx.nb_pulses                     = nb_pulses;
	ctx.SPI_pattern                   = SPI_pattern;
	ctx.nb_samples_by_acquisition_adc = nb_samples;
	//ctx.nb_samples_by_acquisition_adc = HAL_P6I_ULTRASOUND_FREQUENCY_ADC/FREQUENCY_WANTED; //TODO:
	ctx.nb_acquisition                = nb_acquisition;

	int power2_found = 0;
	//Find power 2 closest and inferior to nb_samples_by_acquisition_adc
	i = 0;
	while ((i < 16) && (!power2_found)) {
	  if (ctx.nb_samples_by_acquisition_adc > (1 << (16 - (i + 1)))) {
		ctx.nb_samples_by_acquisition_adc = 1 << (16 - (i + 1));
		power2_found = 1;
	  }
	  i++;
	}
	ctx.adc_raw_data_size = ctx.nb_samples_by_acquisition_adc * sizeof(uint16_t); // Size in bytes

	if (verbose) {
	  printf("Number of acquisitions: %d\n", ctx.nb_acquisition);
	  printf("Number of samples per acquisitions: %d\n", ctx.nb_samples_by_acquisition_adc);
	  printf("Number of pulses sent: %d\n", ctx.nb_pulses);
	  printf("Pulse pattern: 0x%02X\n", ctx.SPI_pattern & 0xFF);
	}

//*********** sonar_parrot_minidrone_open ***********
	if (verbose) { printf("Opening...\n"); }

	ctx.fd = open(ctx.name_device, O_RDWR);
	if (ctx.fd == -1) {
	  fprintf(stderr, "could not open device %s: %s (%d)\n", ctx.name_device, strerror(errno), errno);
	  ret = -errno;
	  goto close_on_error;
	}
	ctx.fds.fd = ctx.fd;

	ctx.adc_raw_data = calloc(ctx.adc_raw_data_size, sizeof(char));
	if (ctx.adc_raw_data == NULL) {
	  fprintf(stderr, "could not alloc memory (%d bytes) for adc_raw_data: %s (%d)\n", ctx.adc_raw_data_size, strerror(errno), errno);
	  ret = -errno;
	  goto close_on_error;
	}

	memset(ctx.adc_raw_data, 0, ctx.adc_raw_data_size);

	ctx.Tab_SPI = calloc(ctx.nb_pulses, sizeof(uint32_t));
	if (ctx.Tab_SPI == NULL) {
	  fprintf(stderr, "could not alloc memory (%d bytes) for Tab_SPI: %s (%d)\n", ctx.nb_pulses * sizeof(uint32_t), strerror(errno), errno);
	  ret = -errno;
	  goto close_on_error;
	}

	for (i = 0; i < (int)(ctx.nb_pulses); i++) {
	  ctx.Tab_SPI[i] = ctx.SPI_pattern & 0x0FF;
	}

	if (ioctl(ctx.fd, USND_SETSIZE, &ctx.adc_raw_data_size) != 0) {
	  fprintf(stderr, "could not perform ioctl(USND_SETSIZE, %d): %s (%d)\n", ctx.adc_raw_data_size, strerror(errno), errno);
	  ret = -errno;
	  goto close_on_error;
	}

	if (verbose) { printf(". Opened sensor successfully\n"); }

	//*********** sonar_parrot_minidrone_setup ***********
		//if (verbose) { printf(". Setting up...\n"); }

		if (ioctl(ctx.fd, USND_SETSIZE, &ctx.adc_raw_data_size) != 0) {
		  fprintf(stderr, "could not perform ioctl(USND_SETSIZE, %d): %s (%d)\n", ctx.adc_raw_data_size, strerror(errno), errno);
		  ret = -errno;
		} else if (verbose) { printf(".. Setting up USND_SETSIZE %d\n", ctx.adc_raw_data_size); }

		if (ioctl(ctx.fd, USND_SPI_LEN, &ctx.nb_pulses) != 0) {
		  fprintf(stderr, "could not perform ioctl(USND_SPI_LEN, %d): %s (%d)\n", ctx.nb_pulses, strerror(errno), errno);
		  ret = -errno;
		} else if (verbose) { printf(".. Setting up USND_SPI_LEN %d\n", ctx.nb_pulses); }

		if (ioctl(ctx.fd, USND_SPI_DAT, ctx.Tab_SPI) != 0) {
		  fprintf(stderr, "could not perform ioctl(USND_SPI_DAT): %s (%d)\n", strerror(errno), errno);
		  ret = -errno;
		} else if (verbose) { printf(".. Setting up USND_SPI_DAT\n"); }

		if (verbose && ret == 0) { printf(". Setup of sensor successfully\n"); }

//*********** START ***********
	while (true) {

#ifndef SITL

//*********** sonar_parrot_minidrone_polling ***********
	int ret_poll = 0;
	ret = 0;
	//if (verbose) { printf(". Polling...\n"); }

	ctx.fds.events = POLLIN;
	ret_poll = poll(&(ctx.fds), 1, -1);

	if (ret_poll < 0) {
	  fprintf(stderr, "poll(ADC) error: %s (%d)\n", strerror(errno), errno);
	  ret = -1;
	}

	if (ret >= 0)
	{
	  if (ctx.fds.revents & POLLIN)
	  {
		ret = 0;
	  }
	  else
	  {
		fprintf(stderr, "poll(ADC) no data\n");
		ret = -1;
	  }
	}

	//if (verbose && ret == 0) { printf(". Polled successfully\n"); }

//*********** sonar_parrot_minidrone_get_data ***********
	//if (verbose) { printf(". Getting data...\n"); }

	if (ioctl(ctx.fds.fd, USND_COPYSAMPLE, ctx.adc_raw_data) != 0) {
	  fprintf(stderr, "could not perform ioctl(USND_COPYSAMPLE): %s (%d)\n", strerror(errno), errno);
	  ret = -errno;
	}

	if (verbose && ret==0) {
	  printf(". Got data successfully ADC size: %d a sampled value %d\n", ctx.adc_raw_data_size, ctx.adc_raw_data);
	}

	//TODO: Now here the nifty code already in Bebop Sonar Driver...

	// debugyyyyyyy
    uint32_t tempy = 45678;
    if (ret == 0) {
      //printf(". Got data successfully!");
      tempy = &ctx.adc_raw_data[0];
      sonar_parrot_minidrone.meas = (uint16_t)(tempy/10000);//rand() % 100;//to see if we have updat data in a quick n dirty way(uint16_t)(tempy/10000);//note that with cast n stuff the UINT16_MAX
      sonar_parrot_minidrone.distance = (float)(tempy);//DEBUGGY...
      //printf(". Got data successfully ADC size: %d a sampled value %d\n", ctx.adc_raw_data_size, ctx.adc_raw_data);
    }
    //else
    //{
    //	break;
    //}

#else // SITL
    sonar_parrot_minidrone.distance = stateGetPositionEnu_f()->z;
    Bound(sonar_parrot_minidrone.distance, 0.1f, 7.0f);
    uint16_t peak_distance = 1;
#endif // SITL

//    if (peak_distance > 0) {
      // Send ABI message
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_parrot_minidrone.distance);
//#ifdef SENSOR_SYNC_SEND_SONAR
      // Send Telemetry report
      DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_parrot_minidrone.meas, &sonar_parrot_minidrone.distance);
//#endif
//    }

  }
  //*********** END  ***********

  //if (ctx.Tab_SPI) { free(ctx.Tab_SPI); }
  //if (ctx.adc_raw_data) { free(ctx.adc_raw_data); }
  //if (ctx.fd != -1) { close(ctx.fd); }

  usleep(200000); //5Hz ??
  return NULL;

close_on_error:
//*********** sonar_parrot_minidrone_close ***********
	if (verbose) { printf(". Closing...\n"); }
	if (ctx.Tab_SPI) { free(ctx.Tab_SPI); }
	if (ctx.adc_raw_data) { free(ctx.adc_raw_data); }
	if (ctx.fd != -1) { close(ctx.fd); }
	return NULL;
}

//not used yet... see dubug above
void sonar_parrot_minidrone_downlink(void)
{
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_parrot_minidrone.meas, &sonar_parrot_minidrone.distance);
}
