/*
 * Copyright (C) 2013 Dino Hensen, Vincent van Hoek
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/ardrone/navdata.c
 * ardrone2 navdata aquisition driver.
 *
 * The ardrone2 provides a navdata stream of packets
 * containing info about all sensors at a rate of 200Hz.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>   // for baud rates and options
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>

#include "std.h"
#include "navdata.h"
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "mcu_periph/gpio.h"

/* Internal used functions */
static void *navdata_read(void *data __attribute__((unused)));
static void navdata_cmd_send(uint8_t cmd);
static bool_t navdata_baro_calib(void);
static void mag_freeze_check(void);
static void baro_update_logic(void);

/* Main navdata structure */
struct navdata_t navdata;

/** Buffer filled in the thread (maximum one navdata packet) */
static uint8_t navdata_buffer[NAVDATA_PACKET_SIZE];
/** flag to indicate new packet is available in buffer */
static bool_t navdata_available = FALSE;

/* syncronization variables */
static pthread_mutex_t navdata_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  navdata_cond  = PTHREAD_COND_INITIALIZER;

#ifndef NAVDATA_FILTER_ID
#define NAVDATA_FILTER_ID 2
#endif

/** Sonar offset.
 *  Offset value in ADC
 *  equals to the ADC value so that height is zero
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 880
#endif

/** Sonar scale.
 *  Sensor sensitivity in m/adc (float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 0.00047
#endif

/**
 * Write to fd even while being interrupted
 */
ssize_t full_write(int fd, const uint8_t *buf, size_t count)
{
  size_t written = 0;

  while (written < count) {
    ssize_t n = write(fd, buf + written, count - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return n;
    }
    written += n;
  }
  return written;
}

/**
 * Read from fd even while being interrupted
 */
ssize_t full_read(int fd, uint8_t *buf, size_t count)
{
  /* Apologies for illiteracy, but we can't overload |read|.*/
  size_t readed = 0;

  while (readed < count) {
    ssize_t n = read(fd, buf + readed, count - readed);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return n;
    }
    readed += n;
  }
  return readed;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_navdata(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ARDRONE_NAVDATA(trans, dev, AC_ID,
                                &navdata.measure.taille,
                                &navdata.measure.nu_trame,
                                &navdata.measure.ax,
                                &navdata.measure.ay,
                                &navdata.measure.az,
                                &navdata.measure.vx,
                                &navdata.measure.vy,
                                &navdata.measure.vz,
                                &navdata.measure.temperature_acc,
                                &navdata.measure.temperature_gyro,
                                &navdata.measure.ultrasound,
                                &navdata.measure.us_debut_echo,
                                &navdata.measure.us_fin_echo,
                                &navdata.measure.us_association_echo,
                                &navdata.measure.us_distance_echo,
                                &navdata.measure.us_curve_time,
                                &navdata.measure.us_curve_value,
                                &navdata.measure.us_curve_ref,
                                &navdata.measure.nb_echo,
                                &navdata.measure.sum_echo,
                                &navdata.measure.gradient,
                                &navdata.measure.flag_echo_ini,
                                &navdata.measure.pressure,
                                &navdata.measure.temperature_pressure,
                                &navdata.measure.mx,
                                &navdata.measure.my,
                                &navdata.measure.mz,
                                &navdata.measure.chksum,
                                &navdata.checksum_errors);
}
#endif

/**
 * Initialize the navdata board
 */
bool_t navdata_init()
{
  assert(sizeof(struct navdata_measure_t) == NAVDATA_PACKET_SIZE);

  // Check if the FD isn't already initialized
  if (navdata.fd <= 0) {
    navdata.fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY); //O_NONBLOCK doesn't work

    if (navdata.fd < 0) {
      printf("[navdata] Unable to open navdata board connection(/dev/ttyO1)\n");
      return FALSE;
    }

    // Update the settings of the UART connection
    fcntl(navdata.fd, F_SETFL, 0); //read calls are non blocking
    //set port options
    struct termios options;
    //Get the current options for the port
    tcgetattr(navdata.fd, &options);
    //Set the baud rates to 460800
    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);

    options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
    options.c_iflag = 0; //clear input options
    options.c_lflag = 0; //clear local options
    options.c_oflag &= ~OPOST; //clear output options (raw output)

    //Set the new options for the port
    tcsetattr(navdata.fd, TCSANOW, &options);
  }

  // Reset available flags
  navdata_available = FALSE;
  navdata.baro_calibrated = FALSE;
  navdata.imu_available = FALSE;
  navdata.baro_available = FALSE;
  navdata.imu_lost = FALSE;

  // Set all statistics to 0
  navdata.checksum_errors = 0;
  navdata.lost_imu_frames = 0;
  navdata.totalBytesRead = 0;
  navdata.packetsRead = 0;
  navdata.last_packet_number = 0;

  // Stop acquisition
  navdata_cmd_send(NAVDATA_CMD_STOP);

  // Read the baro calibration(blocking)
  if (!navdata_baro_calib()) {
    printf("[navdata] Could not acquire baro calibration!\n");
    return FALSE;
  }
  navdata.baro_calibrated = TRUE;

  // Start acquisition
  navdata_cmd_send(NAVDATA_CMD_START);

  // Set navboard gpio control
  gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);
  gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);

  // Start navdata reading thread
  pthread_t navdata_thread;
  if (pthread_create(&navdata_thread, NULL, navdata_read, NULL) != 0) {
    printf("[navdata] Could not create navdata reading thread!\n");
    return FALSE;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "ARDRONE_NAVDATA", send_navdata);
#endif

  // Set to initialized
  navdata.is_initialized = TRUE;
  return TRUE;
}


/**
 * Main reading thread
 * This is done asynchronous because the navdata board doesn't support NON_BLOCKING
 */
static void *navdata_read(void *data __attribute__((unused)))
{
  /* Buffer insert index for reading/writing */
  static uint8_t buffer_idx = 0;

  while (TRUE) {

    // Wait until we are notified to read next data,
    // i.e. buffer has been copied in navdata_update
    pthread_mutex_lock(&navdata_mutex);
    while (navdata_available) {
      pthread_cond_wait(&navdata_cond, &navdata_mutex);
    }
    pthread_mutex_unlock(&navdata_mutex);

    // Read new bytes
    int newbytes = read(navdata.fd, navdata_buffer + buffer_idx, NAVDATA_PACKET_SIZE - buffer_idx);

    // When there was no signal interrupt
    if (newbytes > 0) {
      buffer_idx += newbytes;
      navdata.totalBytesRead += newbytes;
    }

    // If we got a full packet
    if (buffer_idx >= NAVDATA_PACKET_SIZE) {
      // check if the start byte is correct
      if (navdata_buffer[0] != NAVDATA_START_BYTE) {
        uint8_t *pint = memchr(navdata_buffer, NAVDATA_START_BYTE, buffer_idx);

        // Check if we found the start byte in the read data
        if (pint != NULL) {
          memmove(navdata_buffer, pint, NAVDATA_PACKET_SIZE - (pint - navdata_buffer));
          buffer_idx = pint - navdata_buffer;
        } else {
          buffer_idx = 0;
        }
        fprintf(stderr, "[navdata] sync error, startbyte not found, resetting...\n");
        continue;
      }

      /* full packet read with startbyte at the beginning, reset insert index */
      buffer_idx = 0;

      // Calculating the checksum
      uint16_t checksum = 0;
      for (int i = 2; i < NAVDATA_PACKET_SIZE - 2; i += 2) {
        checksum += navdata_buffer[i] + (navdata_buffer[i + 1] << 8);
      }

      struct navdata_measure_t *new_measurement = (struct navdata_measure_t *)navdata_buffer;

      // Check if the checksum is ok
      if (new_measurement->chksum != checksum) {
        fprintf(stderr, "[navdata] Checksum error [calculated: %d] [packet: %d] [diff: %d]\n",
                checksum, new_measurement->chksum, checksum - new_measurement->chksum);
        navdata.checksum_errors++;
        continue;
      }

      // set flag that we have new valid navdata
      pthread_mutex_lock(&navdata_mutex);
      navdata_available = TRUE;
      pthread_mutex_unlock(&navdata_mutex);
    }
  }

  return NULL;
}


/**
 * Update the navdata (event loop)
 */
void navdata_update()
{
  // Check if initialized
  if (!navdata.is_initialized) {
    navdata_init();
    mag_freeze_check();
    return;
  }

  pthread_mutex_lock(&navdata_mutex);
  // If we got a new navdata packet
  if (navdata_available) {

    // Copy the navdata packet
    memcpy(&navdata.measure, navdata_buffer, NAVDATA_PACKET_SIZE);

    // reset the flag
    navdata_available = FALSE;
    // signal that we copied the buffer and new packet can be read
    pthread_cond_signal(&navdata_cond);
    pthread_mutex_unlock(&navdata_mutex);

    // Check if we missed a packet (our counter and the one from the navdata)
    navdata.last_packet_number++;
    if (navdata.last_packet_number != navdata.measure.nu_trame) {
      fprintf(stderr, "[navdata] Lost frame: %d should have been %d\n",
              navdata.measure.nu_trame, navdata.last_packet_number);
      navdata.lost_imu_frames++;
    }
    navdata.last_packet_number = navdata.measure.nu_trame;

    // Invert byte order so that TELEMETRY works better
    uint8_t tmp;
    uint8_t *p = (uint8_t *) & (navdata.measure.pressure);
    tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;
    p = (uint8_t *) & (navdata.measure.temperature_pressure);
    tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;

    baro_update_logic();
    mag_freeze_check();

#ifdef USE_SONAR
    // Check if there is a new sonar measurement and update the sonar
    if (navdata.measure.ultrasound >> 15) {
      float sonar_meas = (float)((navdata.measure.ultrasound & 0x7FFF) - SONAR_OFFSET) * SONAR_SCALE;
      AbiSendMsgAGL(AGL_SONAR_ARDRONE2_ID, sonar_meas);
    }
#endif

    navdata.imu_available = TRUE;
    navdata.packetsRead++;
  }
  else {
    // no new packet available, still unlock mutex again
    pthread_mutex_unlock(&navdata_mutex);
  }
}

/**
 * Sends a one byte command
 */
static void navdata_cmd_send(uint8_t cmd)
{
  full_write(navdata.fd, &cmd, 1);
}


/**
 * Try to receive the baro calibration from the navdata board
 */
static bool_t navdata_baro_calib(void)
{
  // Start baro calibration acquisition
  navdata_cmd_send(NAVDATA_CMD_BARO_CALIB);

  // Receive the calibration (blocking)
  uint8_t calibBuffer[22];
  if (full_read(navdata.fd, calibBuffer, sizeof calibBuffer) < 0) {
    printf("[navdata] Could not read calibration data.");
    return FALSE;
  }

  //Convert the read bytes
  navdata.bmp180_calib.ac1 = calibBuffer[0]  << 8 | calibBuffer[1];
  navdata.bmp180_calib.ac2 = calibBuffer[2]  << 8 | calibBuffer[3];
  navdata.bmp180_calib.ac3 = calibBuffer[4]  << 8 | calibBuffer[5];
  navdata.bmp180_calib.ac4 = calibBuffer[6]  << 8 | calibBuffer[7];
  navdata.bmp180_calib.ac5 = calibBuffer[8]  << 8 | calibBuffer[9];
  navdata.bmp180_calib.ac6 = calibBuffer[10] << 8 | calibBuffer[11];
  navdata.bmp180_calib.b1  = calibBuffer[12] << 8 | calibBuffer[13];
  navdata.bmp180_calib.b2  = calibBuffer[14] << 8 | calibBuffer[15];
  navdata.bmp180_calib.mb  = calibBuffer[16] << 8 | calibBuffer[17];
  navdata.bmp180_calib.mc  = calibBuffer[18] << 8 | calibBuffer[19];
  navdata.bmp180_calib.md  = calibBuffer[20] << 8 | calibBuffer[21];

  return TRUE;
}

/**
 * Check if the magneto is frozen
 * Unknown why this bug happens.
 */
static void mag_freeze_check(void)
{
  // Thanks to Daren.G.Lee for initial fix on 20140530
  static int16_t LastMagValue = 0;
  static int MagFreezeCounter = 0;

  if (LastMagValue == navdata.measure.mx) {
    MagFreezeCounter++;

    // has to have at least 30 times the same value to consider it a frozen magnetometer value
    if (MagFreezeCounter > 30) {
      fprintf(stderr, "mag freeze detected, resetting!\n");

      // set imu_lost flag
      navdata.imu_lost = TRUE;
      navdata.lost_imu_frames++;

      // Stop acquisition
      navdata_cmd_send(NAVDATA_CMD_STOP);

      // Reset the hardware of the navboard
      gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);
      usleep(20000);
      gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);

      // Wait for 40ms for it to boot
      usleep(40000);

      // Start the navdata again and reset the counter
      navdata_cmd_send(NAVDATA_CMD_START);
      MagFreezeCounter = 0; // reset counter back to zero
    }
  } else {
    navdata.imu_lost = FALSE;
    // Reset counter if value _does_ change
    MagFreezeCounter = 0;
  }
  // set last value
  LastMagValue = navdata.measure.mx;
}

/**
 * Handle the baro(pressure/temperature) logic
 * Sometimes the temperature and pressure are switched because of a bug in
 * the navdata board firmware.
 */
static void baro_update_logic(void)
{
  static int32_t lastpressval = 0;
  static uint16_t lasttempval = 0;
  static int32_t lastpressval_nospike = 0;
  static uint16_t lasttempval_nospike = 0;
  static uint8_t temp_or_press_was_updated_last =
    0; // 0 = press, so we now wait for temp, 1 = temp so we now wait for press

  static int sync_errors = 0;
  static int spike_detected = 0;

  if (temp_or_press_was_updated_last == 0) { // Last update was press so we are now waiting for temp
    // temp was updated
    temp_or_press_was_updated_last = TRUE;

    // This means that press must remain constant
    if (lastpressval != 0) {
      // If pressure was updated: this is a sync error
      if (lastpressval != navdata.measure.pressure) {
        // wait for temp again
        temp_or_press_was_updated_last = FALSE;
        sync_errors++;
        //printf("Baro-Logic-Error (expected updated temp, got press)\n");
      }
    }
  } else {
    // press was updated
    temp_or_press_was_updated_last = FALSE;

    // This means that temp must remain constant
    if (lasttempval != 0) {
      // If temp was updated: this is a sync error
      if (lasttempval != navdata.measure.temperature_pressure) {
        // wait for press again
        temp_or_press_was_updated_last = TRUE;
        sync_errors++;
        //printf("Baro-Logic-Error (expected updated press, got temp)\n");

      } else {
        // We now got valid pressure and temperature
        navdata.baro_available = TRUE;
      }
    }
  }

  // Detected a pressure switch
  if (lastpressval != 0 && lasttempval != 0
      && ABS(lastpressval - navdata.measure.pressure) > ABS(lasttempval - navdata.measure.pressure)) {
    navdata.baro_available = FALSE;
  }

  // Detected a temprature switch
  if (lastpressval != 0 && lasttempval != 0
      && ABS(lasttempval - navdata.measure.temperature_pressure) > ABS(lastpressval - navdata.measure.temperature_pressure)) {
    navdata.baro_available = FALSE;
  }

  lasttempval = navdata.measure.temperature_pressure;
  lastpressval = navdata.measure.pressure;

  /*
   * It turns out that a lot of navdata boards have a problem (probably interrupt related)
   * in which reading I2C data and writing uart output data is interrupted very long (50% of 200Hz).
   * Afterwards, the 200Hz loop tries to catch up lost time but reads the baro too fast swapping the
   * pressure and temperature values by exceeding the minimal conversion time of the bosh baro. The
   * normal Parrot firmware seems to be perfectly capable to fly with this, either with excessive use of
   * the sonar or with software filtering or spike detection. Paparazzi with its tightly coupled baro-altitude
   * had problems. Since this problems looks not uncommon a detector was made. A lot of evidence is grabbed
   * with a logic analyzer on the navboard I2C and serial output. The UART CRC is still perfect, the baro
   * temp-press-temp-press logic is still perfect, so not easy to detect. Temp and pressure are swapped,
   * and since both can have almost the same value, the size of the spike is not predictable. However at
   * every spike of at least 3 broken boards the press and temp are byte-wise exactly the same due to
   * reading them too quickly (trying to catch up for delay that happened earlier due to still non understood
   * reasons. As pressure is more likely to quickly change, a small (yet unlikely) spike on temperature together with
   * press==temp yields very good results as a detector, although theoretically not perfect.

  #samp press temp.
  50925 39284 34501
  50926 39287 34501
  50927 39287 34501
  50928 39283 34501     // *press good -> baro
  50929 39283 34501
  50930 39285 34501     // *press good -> baro
  50931 39285 34500
  50932 34500 34500     // press read too soon from chip (<4.5ms) -> ADC register still previous temp value
  50933 34500 36618     // press not updated, still wrong. Temp is weird: looks like the average of both
  50934 39284 36618     // new press read, but temp still outdated
  50935 39284 34501
  50936 39284 34501     // *press good -> baro
  50937 39284 34500
  50938 39281 34500
  50939 39281 34500
  50940 39280 34500
  50941 39280 34502
  50942 39280 34502
  50943 39280 34501

   */

  // if press and temp are same and temp has jump: neglect the next frame
  if (navdata.measure.temperature_pressure ==
      navdata.measure.pressure) { // && (abs((int32_t)navdata.temperature_pressure - (int32_t)lasttempval) > 40))
    // dont use next 3 packets
    spike_detected = 3;
  }

  if (spike_detected > 0) {
    // disable kalman filter use
    navdata.baro_available = FALSE;

    // override both to last good
    navdata.measure.pressure = lastpressval_nospike;
    navdata.measure.temperature_pressure = lasttempval_nospike;

    // Countdown
    spike_detected--;
  } else { // both are good
    lastpressval_nospike = navdata.measure.pressure;
    lasttempval_nospike = navdata.measure.temperature_pressure;
  }
}
