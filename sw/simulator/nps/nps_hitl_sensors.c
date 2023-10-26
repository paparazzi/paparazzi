/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 */

#include "nps_ins.h"
#include <sys/time.h>
#include "nps_fdm.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/poll.h>
#include "nps_sensors.h"
#include "nps_main.h"
#include "paparazzi.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprz_transport.h"
#include "arch/linux/serial_port.h"

#ifndef AP_DEV
#warning "[hitl] Please define AP_DEV in your airframe file"
#define AP_DEV "/dev/ttyUSB0"
#endif
#ifndef AP_BAUD
#define AP_BAUD B230400
PRINT_CONFIG_MSG_VALUE("[hitl] Using default baudrate for AP_DEV (B230400)", AP_BAUD)
#endif

#define NPS_HITL_DEBUG 0
#if NPS_HITL_DEBUG
#define DEBUG_PRINT printf
#else
#define DEBUG_PRINT(...) {}
#endif

void *nps_sensors_loop(void *data __attribute__((unused)));
void *nps_ap_data_loop(void *data __attribute__((unused)));
pthread_t th_sensors; // send sensors to AP
pthread_t th_ap_data; // receive commands from AP

/* implement pprzlink_device interface */
#define PPRZLINK_BUFFER_SIZE 256
struct linkdev {
  /** Receive buffer */
  uint8_t rx_buf[PPRZLINK_BUFFER_SIZE];
  uint16_t rx_idx;
  /** Transmit buffer */
  uint8_t tx_buf[PPRZLINK_BUFFER_SIZE];
  uint16_t tx_idx;
  //volatile uint8_t tx_running;
  /** Generic device interface */
  struct link_device device;
  /** transport */
  struct pprz_transport pprz_tp;
  /** Serial port */
  struct SerialPort *port;
  struct pollfd fds[1];
};

static struct linkdev dev;

static int check_free_space(struct linkdev *d, long *fd __attribute__((unused)), uint16_t len)
{
  int space = PPRZLINK_BUFFER_SIZE - d->tx_idx;
  return space >= len ? space : 0;
}

static void put_byte(struct linkdev *d, long fd __attribute__((unused)), uint8_t data)
{
  d->tx_buf[d->tx_idx++] = data;
}

static void put_buffer(struct linkdev *d, long fd __attribute__((unused)), const uint8_t *data, uint16_t len)
{
  memcpy(&(d->tx_buf[d->tx_idx]), data, len);
  d->tx_idx += len;
}

static void send_message(struct linkdev *d, long fd __attribute__((unused)))
{
  int ret = 0;
  do {
    ret = write((int)(d->port->fd), d->tx_buf, d->tx_idx);
  } while (ret < 1 && errno == EAGAIN); //FIXME: max retry
  if (ret < 1) {
    DEBUG_PRINT("[hitl] put_byte: write failed [%d: %s]\n", ret, strerror(errno));
  }
  d->tx_idx = 0;
}

static uint8_t getch(struct linkdev *d)
{
  // this function should only be called when bytes are available
  unsigned char c = 'B';
  ssize_t ret = 0;
  ret = read(d->port->fd, &c, 1);
  if (ret > 0) {
    d->rx_buf[d->rx_idx] = c;
    if (d->rx_idx < PPRZLINK_BUFFER_SIZE) {
      d->rx_idx++;
    } else {
      DEBUG_PRINT("[hitl] rx buffer overflow\n");
    }
  } else {
    DEBUG_PRINT("[hitl] rx read error\n");
  }
  return c;
}

static int char_available(struct linkdev *d)
{
  int ret = poll(d->fds, 1, 1000);
  if (ret > 0) {
    if (d->fds[0].revents & POLLHUP) {
      printf("[hitl] lost connection. Exiting\n");
      exit(1);
    }
    if (d->fds[0].revents & POLLIN) {
      return true;
    }
  } else if (ret == -1) {
    DEBUG_PRINT("[hitl] poll failed\n");
  }
  return false;
}

/// END pprzlink_dev

void nps_hitl_impl_init(void)
{
  pthread_create(&th_sensors, NULL, nps_sensors_loop, NULL);
  pthread_create(&th_ap_data, NULL, nps_ap_data_loop, NULL);

  dev.device.periph = (void *) (&dev);
  dev.device.check_free_space = (check_free_space_t) check_free_space;
  dev.device.put_byte = (put_byte_t) put_byte;
  dev.device.put_buffer = (put_buffer_t) put_buffer;
  dev.device.send_message = (send_message_t) send_message;
  dev.device.char_available = (char_available_t) char_available;
  dev.device.get_byte = (get_byte_t) getch;
  pprz_transport_init(&dev.pprz_tp);

  // open serial port
  dev.port = serial_port_new();
  int ret = serial_port_open_raw(dev.port, AP_DEV, AP_BAUD);
  if (ret != 0) {
    printf("[hitl] Error opening %s code %d\n", AP_DEV, ret);
    serial_port_free(dev.port);
  }

  // poll
  if (dev.port != NULL) {
    dev.fds[0].fd = dev.port->fd;
    dev.fds[0].events = POLLIN;
  }
}

void *nps_sensors_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = (1. / PERIODIC_FREQUENCY) * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  while (TRUE) {
    // lock mutex
    pthread_mutex_lock(&fdm_mutex);

    // start timing
    clock_get_current_time(&requestStart);

    uint8_t id = AC_ID;

    if (nps_sensors_gyro_available()) {
      float gx = (float)sensors.gyro.value.x;
      float gy = (float)sensors.gyro.value.y;
      float gz = (float)sensors.gyro.value.z;
      float ax = (float)sensors.accel.value.x;
      float ay = (float)sensors.accel.value.y;
      float az = (float)sensors.accel.value.z;
      float mx = (float)sensors.mag.value.x;
      float my = (float)sensors.mag.value.y;
      float mz = (float)sensors.mag.value.z;
      uint8_t id = AC_ID;
      pprz_msg_send_HITL_IMU(&dev.pprz_tp.trans_tx, &dev.device, 0,
          &gx, &gy, &gz,
          &ax, &ay, &az,
          &mx, &my, &mz,
          &id);
    }

    if (nps_sensors_gps_available()) {
      float gps_lat = (float)DegOfRad(sensors.gps.lla_pos.lat);
      float gps_lon = (float)DegOfRad(sensors.gps.lla_pos.lon);
      float gps_alt = (float)sensors.gps.lla_pos.alt;
      float gps_hmsl = (float)sensors.gps.hmsl;
      float gps_vx = (float)sensors.gps.ecef_vel.x;
      float gps_vy = (float)sensors.gps.ecef_vel.y;
      float gps_vz = (float)sensors.gps.ecef_vel.z;
      float gps_time = (float)nps_main.sim_time;
      uint8_t gps_fix = 3; // GPS fix
      pprz_msg_send_HITL_GPS(&dev.pprz_tp.trans_tx, &dev.device, 0,
          &gps_lat, &gps_lon, &gps_alt, &gps_hmsl,
          &gps_vx, &gps_vy, &gps_vz,
          &gps_time, &gps_fix, &id);
    }

    uint8_t air_data_flag = 0;
    float baro = -1.f;
    float airspeed = -1.f;
    float aoa = 0.f;
    float sideslip = 0.f;
    if (nps_sensors_baro_available()) {
      SetBit(air_data_flag, 0);
      baro = (float) sensors.baro.value;
    }
    if (nps_sensors_airspeed_available()) {
      SetBit(air_data_flag, 1);
      airspeed = (float) sensors.airspeed.value;
    }
    if (nps_sensors_aoa_available()) {
      SetBit(air_data_flag, 2);
      aoa = (float) sensors.aoa.value;
    }
    if (nps_sensors_sideslip_available()) {
      SetBit(air_data_flag, 3);
      sideslip = (float) sensors.sideslip.value;
    }
    if (air_data_flag != 0) {
      pprz_msg_send_HITL_AIR_DATA(&dev.pprz_tp.trans_tx, &dev.device, 0,
          &baro, &airspeed, &aoa, &sideslip, &air_data_flag, &id);
    }

    // unlock mutex
    pthread_mutex_unlock(&fdm_mutex);

    clock_get_current_time(&requestEnd);

    // Calculate time it took
    task_ns = (requestEnd.tv_sec - requestStart.tv_sec) * 1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    // task took less than one period, sleep for the rest of time
    if (task_ns < period_ns) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = period_ns - task_ns;
      nanosleep(&waitFor, NULL);
    } else {
      // task took longer than the period
#ifdef PRINT_TIME
      printf("SENSORS: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}

void *nps_ap_data_loop(void *data __attribute__((unused)))
{
  struct timespec waitFor;

  uint8_t msg_buffer[PPRZLINK_BUFFER_SIZE];
  bool msg_available = false;

  bool first_command = true;

  uint8_t cmd_len = 0;
  pprz_t  cmd_buf[NPS_COMMANDS_NB];

  while (TRUE) {

    pprz_check_and_parse(&dev.device, &dev.pprz_tp, msg_buffer, &msg_available);
    if (msg_available) {
      // reset rx index to zero for next message
      dev.rx_idx = 0;
      //Parse message
      uint8_t sender_id = SenderIdOfPprzMsg(msg_buffer);
      uint8_t msg_id = IdOfPprzMsg(msg_buffer);

      if (sender_id != AC_ID) {
        printf("[hitl] receiving message from wrong id (%d)\n", sender_id);
        return(NULL); // wrong A/C ?
      }
      /* parse telemetry messages coming from the correct AC_ID */
      switch (msg_id) {
        case DL_COMMANDS:
          // parse commands message
          cmd_len = DL_COMMANDS_values_length(msg_buffer);
          memcpy(&cmd_buf, DL_COMMANDS_values(msg_buffer), cmd_len * sizeof(int16_t));
          pthread_mutex_lock(&fdm_mutex);
          // update commands
          for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
            nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
          }
          // hack: invert pitch to fit most JSBSim models
          nps_autopilot.commands[COMMAND_PITCH] = -(double)cmd_buf[COMMAND_PITCH] / MAX_PPRZ;
          if (first_command) {
            printf("[hitl] receiving COMMANDS message\n");
            first_command = false;
          }
          pthread_mutex_unlock(&fdm_mutex);
          break;
        case DL_MOTOR_MIXING:
          // parse actuarors message
          cmd_len = DL_MOTOR_MIXING_values_length(msg_buffer);
          // check for out-of-bounds access
          if (cmd_len > NPS_COMMANDS_NB) {
            cmd_len = NPS_COMMANDS_NB;
          }
          memcpy(&cmd_buf, DL_MOTOR_MIXING_values(msg_buffer), cmd_len * sizeof(int16_t));
          pthread_mutex_lock(&fdm_mutex);
          // update commands
          for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
            nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
          }
          if (first_command) {
            printf("[hitl] receiving MOTOR_MIXING message\n");
            first_command = false;
          }
          pthread_mutex_unlock(&fdm_mutex);
          break;
        default:
          break;
      }

      msg_available = false;
    }

    // wait before next loop
    waitFor.tv_sec = 0;
    waitFor.tv_nsec = 1000;
    nanosleep(&waitFor, NULL);

  }
  return(NULL);
}


