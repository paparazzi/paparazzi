/*
 * Copyright (C) 2013 Dino Hensen, Vincent van Hoek
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
#include <fcntl.h>     // for O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>   // for baud rates and options
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>

#include "std.h"
#include "navdata.h"
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "mcu_periph/gpio.h"

#define NAVDATA_PACKET_SIZE 60
#define NAVDATA_START_BYTE 0x3a

#define ARDRONE_GPIO_PORT         0x32524
#define ARDRONE_GPIO_PIN_NAVDATA  177

static inline bool_t acquire_baro_calibration(void);
static void navdata_cropbuffer(int cropsize);

navdata_port nav_port;
static int nav_fd = 0;
measures_t navdata;

static int imu_lost = 0;
static int imu_lost_counter = 0;

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

static void navdata_write(const uint8_t *buf, size_t count)
{
  if (full_write(nav_fd, buf, count) < 0) {
    perror("navdata_write: Write failed");
  }
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_navdata(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ARDRONE_NAVDATA(trans, dev, AC_ID,
                                &navdata.taille,
                                &navdata.nu_trame,
                                &navdata.ax,
                                &navdata.ay,
                                &navdata.az,
                                &navdata.vx,
                                &navdata.vy,
                                &navdata.vz,
                                &navdata.temperature_acc,
                                &navdata.temperature_gyro,
                                &navdata.ultrasound,
                                &navdata.us_debut_echo,
                                &navdata.us_fin_echo,
                                &navdata.us_association_echo,
                                &navdata.us_distance_echo,
                                &navdata.us_curve_time,
                                &navdata.us_curve_value,
                                &navdata.us_curve_ref,
                                &navdata.nb_echo,
                                &navdata.sum_echo,
                                &navdata.gradient,
                                &navdata.flag_echo_ini,
                                &navdata.pressure,
                                &navdata.temperature_pressure,
                                &navdata.mx,
                                &navdata.my,
                                &navdata.mz,
                                &navdata.chksum,
                                &nav_port.checksum_errors);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  if (ahrs.status == AHRS_UNINIT) { mde = 2; }
  if (imu_lost) { mde = 5; }
  uint16_t val = imu_lost_counter;
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &mde, &val);
}

#endif

bool_t navdata_init()
{
  if (nav_fd <= 0) {
    nav_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (nav_fd == -1) {
      perror("navdata_init: Unable to open /dev/ttyO1 - ");
      return FALSE;
    }
  }

  fcntl(nav_fd, F_SETFL, 0); //read calls are non blocking
  //set port options
  struct termios options;
  //Get the current options for the port
  tcgetattr(nav_fd, &options);
  //Set the baud rates to 460800
  cfsetispeed(&options, B460800);
  cfsetospeed(&options, B460800);

  options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
  options.c_iflag = 0; //clear input options
  options.c_lflag = 0; //clear local options
  options.c_oflag &= ~OPOST; //clear output options (raw output)

  //Set the new options for the port
  tcsetattr(nav_fd, TCSANOW, &options);

  // stop acquisition
  uint8_t cmd = 0x02;
  navdata_write(&cmd, 1);

  // read some potential dirt
  // wait 10 milliseconds
  char tmp[100];
  for (int i = 0; i < 12; i++) {
    uint16_t dirt = read(nav_fd, tmp, sizeof tmp);
    (void) dirt;

    usleep(1000);
  }

  baro_calibrated = FALSE;
  if (!acquire_baro_calibration()) {
    return FALSE;
  }

  // start acquisition
  cmd = 0x01;
  navdata_write(&cmd, 1);

  navdata_imu_available = FALSE;
  navdata_baro_available = FALSE;

  nav_port.checksum_errors = 0;
  nav_port.lost_imu_frames = 0;
  nav_port.bytesRead = 0;
  nav_port.totalBytesRead = 0;
  nav_port.packetsRead = 0;
  nav_port.isInitialized = TRUE;
  nav_port.last_packet_number = 0;

  // set navboard gpio control
  gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);
  gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "ARDRONE_NAVDATA", send_navdata);
  register_periodic_telemetry(DefaultPeriodic, "STATE_FILTER_STATUS", send_filter_status);
#endif

  return TRUE;
}

static inline bool_t acquire_baro_calibration(void)
{
  // start baro calibration acquisition
  uint8_t cmd = 0x17; // send cmd 23
  navdata_write(&cmd, 1);

  // wait 20ms to retrieve data
  for (int i = 0; i < 22; i++) {
    usleep(1000);
  }

  uint8_t calibBuffer[22];

  if (full_read(nav_fd, calibBuffer, sizeof calibBuffer) < 0) {
    perror("acquire_baro_calibration: read failed");
    return FALSE;
  }

  baro_calibration.ac1 = calibBuffer[0]  << 8 | calibBuffer[1];
  baro_calibration.ac2 = calibBuffer[2]  << 8 | calibBuffer[3];
  baro_calibration.ac3 = calibBuffer[4]  << 8 | calibBuffer[5];
  baro_calibration.ac4 = calibBuffer[6]  << 8 | calibBuffer[7];
  baro_calibration.ac5 = calibBuffer[8]  << 8 | calibBuffer[9];
  baro_calibration.ac6 = calibBuffer[10] << 8 | calibBuffer[11];
  baro_calibration.b1  = calibBuffer[12] << 8 | calibBuffer[13];
  baro_calibration.b2  = calibBuffer[14] << 8 | calibBuffer[15];
  baro_calibration.mb  = calibBuffer[16] << 8 | calibBuffer[17];
  baro_calibration.mc  = calibBuffer[18] << 8 | calibBuffer[19];
  baro_calibration.md  = calibBuffer[20] << 8 | calibBuffer[21];

  printf("Calibration AC1: %d\n", baro_calibration.ac1);
  printf("Calibration AC2: %d\n", baro_calibration.ac2);
  printf("Calibration AC3: %d\n", baro_calibration.ac3);
  printf("Calibration AC4: %d\n", baro_calibration.ac4);
  printf("Calibration AC5: %d\n", baro_calibration.ac5);
  printf("Calibration AC6: %d\n", baro_calibration.ac6);

  printf("Calibration B1: %d\n", baro_calibration.b1);
  printf("Calibration B2: %d\n", baro_calibration.b2);

  printf("Calibration MB: %d\n", baro_calibration.mb);
  printf("Calibration MC: %d\n", baro_calibration.mc);
  printf("Calibration MD: %d\n", baro_calibration.md);

  baro_calibrated = TRUE;
  return TRUE;
}

void navdata_read()
{
  int newbytes = read(nav_fd, nav_port.buffer + nav_port.bytesRead, NAVDATA_BUFFER_SIZE - nav_port.bytesRead);

  // because non-blocking read returns -1 when no bytes available
  if (newbytes > 0) {
    nav_port.bytesRead += newbytes;
    nav_port.totalBytesRead += newbytes;
  }
}


static void mag_freeze_check(void)
{
  // Thanks to Daren.G.Lee for initial fix on 20140530
  static int16_t LastMagValue = 0;
  static int MagFreezeCounter = 0;

  if (LastMagValue == navdata.mx) {
    MagFreezeCounter++;

    // has to have at least 30 times the same value to consider it a frozen magnetometer value
    if (MagFreezeCounter > 30) {
      //printf("Magetometer is frozen. Lastvalue X: %d , currentvalue X: %d resetting...", LastMagValue, navdata.mx);
      // set imu_lost flag
      imu_lost = 1;
      imu_lost_counter++;

      // stop acquisition
      uint8_t cmd = 0x02;
      navdata_write(&cmd, 1);
      // do the navboard reset via GPIOs
      gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);
      // a delay added, otherwise gpio_set sometime does not work
      usleep(20000);
      gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);

      //uint8_t mde = 5;
      //uint16_t val = 0;
      //DOWNLINK_SEND_STATE_FILTER_STATUS(DefaultChannel, DefaultDevice, &mde, &val);

      // wait 40ms to retrieve data
      // using 40 times a 1ms wait in case the usleep function
      // is interupted by a signal
      for (int i = 0; i < 40; i++) {
        usleep(1000);
      }

      // restart acquisition
      cmd = 0x01;

      // Weird, not having one more a delay and fix does not work... thus pragmatic fix
      usleep(5000);

      /* Due to the Ardrone2 NAVBoard design, one time restarting does not work
       * in all cases, but multiple attempts do.
       */
      for (int i = 0; i < 10; i++) {
        usleep(1000);
        navdata_write(&cmd, 1);
      }

      MagFreezeCounter = 0; // reset counter back to zero
    }
  } else {
    imu_lost = 0;
    // Reset counter if value _does_ change
    MagFreezeCounter = 0;
  }
  // set last value
  LastMagValue = navdata.mx;
}

static void baro_update_logic(void)
{
  static int32_t lastpressval = 0;
  static uint16_t lasttempval = 0;
  static int32_t lastpressval_nospike = 0;
  static uint16_t lasttempval_nospike = 0;
  static uint8_t temp_or_press_was_updated_last = 0; // 0 = press, so we now wait for temp, 1 = temp so we now wait for press

  static int sync_errors = 0;
  static int spike_detected = 0;

  if (temp_or_press_was_updated_last == 0) { // Last update was press so we are now waiting for temp
    // temp was updated
    temp_or_press_was_updated_last = TRUE;

    // This means that press must remain constant
    if (lastpressval != 0) {
      // If pressure was updated: this is a sync error
      if (lastpressval != navdata.pressure) {
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
      if (lasttempval != navdata.temperature_pressure) {
        // wait for press again
        temp_or_press_was_updated_last = TRUE;
        sync_errors++;
        //printf("Baro-Logic-Error (expected updated press, got temp)\n");

      } else {
        // We now got valid pressure and temperature
        navdata_baro_available = TRUE;
      }
    }
  }

  // Detected a pressure switch
  if (lastpressval != 0 && lasttempval != 0 && ABS(lastpressval - navdata.pressure) > ABS(lasttempval - navdata.pressure)) {
    navdata_baro_available = FALSE;
  }

  // Detected a temprature switch
  if (lastpressval != 0 && lasttempval != 0 && ABS(lasttempval - navdata.temperature_pressure) > ABS(lastpressval - navdata.temperature_pressure)) {
    navdata_baro_available = FALSE;
  }

  lasttempval = navdata.temperature_pressure;
  lastpressval = navdata.pressure;

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
  if (navdata.temperature_pressure == navdata.pressure) { // && (abs((int32_t)navdata.temperature_pressure - (int32_t)lasttempval) > 40))
    // dont use next 3 packets
    spike_detected = 3;
  }

  if (spike_detected > 0) {
    // disable kalman filter use
    navdata_baro_available = FALSE;

    // override both to last good
    navdata.pressure = lastpressval_nospike;
    navdata.temperature_pressure = lasttempval_nospike;

    // Countdown
    spike_detected--;
  } else { // both are good
    lastpressval_nospike = navdata.pressure;
    lasttempval_nospike = navdata.temperature_pressure;
  }

// printf("%d %d %d\r\n", navdata.temperature_pressure, navdata.pressure, spike_detected);
// printf(",%d,%d",spike_detected,spikes);
}

void navdata_update()
{
  // Check if initialized
  if (!nav_port.isInitialized) {
    navdata_init();
    mag_freeze_check();
    return;
  }

  // Start reading
  navdata_read();

  // while there is something interesting to do...
  while (nav_port.bytesRead >= NAVDATA_PACKET_SIZE) {
    if (nav_port.buffer[0] == NAVDATA_START_BYTE) {
      assert(sizeof navdata == NAVDATA_PACKET_SIZE);
      memcpy(&navdata, nav_port.buffer, NAVDATA_PACKET_SIZE);

      // Calculating the checksum
      uint16_t checksum = 0;
      for (int i = 2; i < NAVDATA_PACKET_SIZE - 2; i += 2) {
        checksum += nav_port.buffer[i] + (nav_port.buffer[i + 1] << 8);
      }

      // When checksum is incorrect
      if (navdata.chksum != checksum) {
        printf("Checksum error [calculated: %d] [packet: %d] [diff: %d]\n", checksum , navdata.chksum, checksum - navdata.chksum);
        nav_port.checksum_errors++;
      }

      nav_port.last_packet_number++;
      if (nav_port.last_packet_number != navdata.nu_trame) {
        //printf("Lost Navdata frame: %d should have been %d\n",navdata.nu_trame, nav_port.last_packet_number);
        nav_port.lost_imu_frames++;
      }
      nav_port.last_packet_number = navdata.nu_trame;
      //printf("%d\r",navdata.nu_trame);

      // When checksum is correct
      if(navdata.chksum == checksum) {
        // Invert byte order so that TELEMETRY works better
        uint8_t tmp;
        uint8_t *p = (uint8_t *) & (navdata.pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;
        p = (uint8_t *) & (navdata.temperature_pressure);
        tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;

//        printf("%d,%d,%d",navdata.nu_trame, navdata.pressure, navdata.temperature_pressure);

        baro_update_logic();

//        printf(",%d,%d,%d\n", navdata.pressure, navdata.temperature_pressure, (int)navdata_baro_available);

        mag_freeze_check();

#ifdef USE_SONAR
        // Check if there is a new sonar measurement and update the sonar
        if (navdata.ultrasound >> 15) {
          float sonar_meas = (float)((navdata.ultrasound & 0x7FFF) - SONAR_OFFSET) * SONAR_SCALE;
          AbiSendMsgAGL(AGL_SONAR_ARDRONE2_ID, &sonar_meas);
        }
#endif

        navdata_imu_available = TRUE;
        nav_port.packetsRead++;
      }

      // Crop the buffer
      navdata_cropbuffer(NAVDATA_PACKET_SIZE);
    } else {
      // find start byte, copy all data from startbyte to buffer origin, update bytesread
      uint8_t *pint;
      pint = memchr(nav_port.buffer, NAVDATA_START_BYTE, nav_port.bytesRead);

      if (pint != NULL) {
        navdata_cropbuffer(pint - nav_port.buffer);
      } else {
        // if the start byte was not found, it means there is junk in the buffer
        nav_port.bytesRead = 0;
      }
    }
  }

}

static void navdata_cropbuffer(int cropsize)
{
  if (nav_port.bytesRead - cropsize < 0) {
    // TODO think about why the amount of bytes read minus the cropsize gets below zero
    printf("BytesRead(=%d) - Cropsize(=%d) may not be below zero. port->buffer=%p\n", nav_port.bytesRead, cropsize, nav_port.buffer);
    return;
  }

  memmove(nav_port.buffer, nav_port.buffer + cropsize, NAVDATA_BUFFER_SIZE - cropsize);
  nav_port.bytesRead -= cropsize;
}
