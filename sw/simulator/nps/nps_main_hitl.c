/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "termios.h"
#include <fcntl.h>
#include <unistd.h>


#include "paparazzi.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/pprz_transport.h"
#include "generated/airframe.h"


#include "nps_main.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"
#include "mcu_periph/sys_time.h"

#include "nps_ins.h"

void *nps_ins_data_loop(void *data __attribute__((unused)));
void *nps_ap_data_loop(void *data __attribute__((unused)));

pthread_t th_ins_data; // sends INS packets to the autopilot
pthread_t th_ap_data; // receives commands from the autopilot

#define NPS_MAX_MSG_SIZE 512

int main(int argc, char **argv)
{
  nps_main_init(argc, argv);

  if (nps_main.fg_host) {
    pthread_create(&th_flight_gear, NULL, nps_flight_gear_loop, NULL);
  }
  pthread_create(&th_display_ivy, NULL, nps_main_display, NULL);
  pthread_create(&th_main_loop, NULL, nps_main_loop, NULL);
  pthread_create(&th_ins_data, NULL, nps_ins_data_loop, NULL);
  pthread_create(&th_ap_data, NULL, nps_ap_data_loop, NULL);
  pthread_join(th_main_loop, NULL);

  return 0;
}

void nps_radio_and_autopilot_init(void)
{
  enum NpsRadioControlType rc_type;
  char *rc_dev = NULL;
  if (nps_main.norc) {
    rc_type = NORC;
  } else if (nps_main.js_dev) {
    rc_type = JOYSTICK;
    rc_dev = nps_main.js_dev;
  } else if (nps_main.spektrum_dev) {
    rc_type = SPEKTRUM;
    rc_dev = nps_main.spektrum_dev;
  } else {
    rc_type = SCRIPT;
  }
  nps_autopilot_init(rc_type, nps_main.rc_script, rc_dev);
}

void nps_update_launch_from_dl(uint8_t value)
{
  nps_autopilot.launch = value;
  printf("Launch value=%u\n",nps_autopilot.launch);
}

void nps_main_run_sim_step(void)
{
  nps_atmosphere_update(SIM_DT);

  nps_fdm_run_step(nps_autopilot.launch, nps_autopilot.commands, NPS_COMMANDS_NB);

  nps_sensors_run_step(nps_main.sim_time);
}

void *nps_ins_data_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = (1. / INS_FREQUENCY) * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  nps_ins_init(); // initialize ins variables and pointers

  // configure port
  int fd = open(STRINGIFY(INS_DEV), O_WRONLY | O_NOCTTY | O_SYNC);//open(INS_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    printf("INS THREAD: data loop error opening port %i\n", fd);
    return(NULL);
  }

  struct termios new_settings;
  tcgetattr(fd, &new_settings);
  memset(&new_settings, 0, sizeof(new_settings));
  new_settings.c_iflag = 0;
  new_settings.c_cflag = 0;
  new_settings.c_lflag = 0;
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 0;
  cfsetispeed(&new_settings, (speed_t)INS_BAUD);
  cfsetospeed(&new_settings, (speed_t)INS_BAUD);
  tcsetattr(fd, TCSANOW, &new_settings);

  struct NpsFdm fdm_ins;

  while (TRUE) {
    // lock mutex
    pthread_mutex_lock(&fdm_mutex);

    // start timing
    clock_get_current_time(&requestStart);

    // make a copy of fdm struct to speed things up
    memcpy(&fdm_ins, &fdm, sizeof(fdm));

    // unlock mutex
    pthread_mutex_unlock(&fdm_mutex);

    // fetch data
    nps_ins_fetch_data(&fdm_ins);

    // send ins data here
    static uint16_t idx;
    idx = nps_ins_fill_buffer();

    static int wlen;
    wlen = write(fd, ins_buffer, idx);
    if (wlen != idx) {
      printf("INS THREAD: Warning - sent only %u bytes to the autopilot, instead of expected %u\n", wlen, idx);
    }

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
      printf("INS THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}



void *nps_ap_data_loop(void *data __attribute__((unused)))
{
  // configure port
  int fd = open(STRINGIFY(AP_DEV), O_RDONLY | O_NOCTTY);
  if (fd < 0) {
    printf("AP data loop error opening port %i\n", fd);
    return(NULL);
  }

  struct termios new_settings;
  tcgetattr(fd, &new_settings);
  memset(&new_settings, 0, sizeof(new_settings));
  new_settings.c_iflag = 0;
  new_settings.c_cflag = 0;
  new_settings.c_lflag = 0;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 5;
  cfsetispeed(&new_settings, (speed_t)AP_BAUD);
  cfsetospeed(&new_settings, (speed_t)AP_BAUD);
  tcsetattr(fd, TCSANOW, &new_settings);

  int rdlen;
  uint8_t buf[NPS_MAX_MSG_SIZE];
  uint8_t cmd_len;
  pprz_t  cmd_buf[NPS_COMMANDS_NB];

  struct pprz_transport pprz_tp_logger;

  pprz_transport_init(&pprz_tp_logger);
  uint32_t rx_msgs = 0;
  uint32_t rx_commands = 0;
  uint32_t rx_motor_mixing = 0;
  uint8_t show_stats = 1;

  while (TRUE) {
    if ((rx_msgs % 100 == 0) && show_stats && (rx_msgs > 0) ) {
      printf("AP data loop received %i messages.\n", rx_msgs);
      printf("From those, %i were COMMANDS messages, and %i were MOTOR_MIXING messages.\n",
          rx_commands, rx_motor_mixing);
      show_stats = 0;
    }

    // receive messages from the autopilot
    // Note: read function might wait until the buffer is full, in which case
    // it could contain several messages. That might lead to delay in processing,
    // for example if we send COMMANDS at 100Hz, and each read() will hold 10 COMMANDS
    // it means a delay of 100ms before the message is processed.
    rdlen = read(fd, buf, sizeof(buf) - 1);

    for (int i = 0; i < rdlen; i++) {
      // parse data
      parse_pprz(&pprz_tp_logger, buf[i]);

      // if msg_available then read
      if (pprz_tp_logger.trans_rx.msg_received) {
        rx_msgs++;
        for (int k = 0; k < pprz_tp_logger.trans_rx.payload_len; k++) {
          buf[k] = pprz_tp_logger.trans_rx.payload[k];
        }
        //Parse message
        uint8_t sender_id = SenderIdOfPprzMsg(buf);
        uint8_t msg_id = IdOfPprzMsg(buf);

        // parse telemetry messages coming from other AC
        if (sender_id != AC_ID) {
          switch (msg_id) {
            default: {
              break;
            }
          }
        } else {
          /* parse telemetry messages coming from the correct AC_ID */
          switch (msg_id) {
            case DL_COMMANDS:
              // parse commands message
              rx_commands++;
              cmd_len = DL_COMMANDS_values_length(buf);
              memcpy(&cmd_buf, DL_COMMANDS_values(buf), cmd_len * sizeof(int16_t));
              pthread_mutex_lock(&fdm_mutex);
              // update commands
              for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
                nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
              }
              // hack: invert pitch to fit most JSBSim models
              nps_autopilot.commands[COMMAND_PITCH] = -(double)cmd_buf[COMMAND_PITCH] / MAX_PPRZ;
              pthread_mutex_unlock(&fdm_mutex);
              break;
            case DL_MOTOR_MIXING:
              // parse actuarors message
              rx_motor_mixing++;
              cmd_len = DL_MOTOR_MIXING_values_length(buf);
              // check for out-of-bounds access
              if (cmd_len > NPS_COMMANDS_NB) {
                cmd_len = NPS_COMMANDS_NB;
              }
              memcpy(&cmd_buf, DL_MOTOR_MIXING_values(buf), cmd_len * sizeof(int16_t));
              pthread_mutex_lock(&fdm_mutex);
              // update commands
              for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
                nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
              }
              pthread_mutex_unlock(&fdm_mutex);
              break;
            default:
              break;
          }
        }
        pprz_tp_logger.trans_rx.msg_received = false;
      }
    }
  }
  return(NULL);
}




void *nps_main_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = SIM_DT * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  // check the sim time difference from the realtime
  // fdm.time - simulation time
  struct timespec startTime;
  struct timespec realTime;
  clock_get_current_time(&startTime);
  double start_secs = ntime_to_double(&startTime);
  double real_secs = 0;
  double real_time = 0;
  static int guard;

  while (TRUE) {
    clock_get_current_time(&requestStart);

    pthread_mutex_lock(&fdm_mutex);

    // check the current simulation time
    clock_get_current_time(&realTime);
    real_secs = ntime_to_double(&realTime);
    real_time = real_secs - start_secs; // real time elapsed

    guard = 0;
    while ((real_time - fdm.time) > SIM_DT) {
      nps_main_run_sim_step();
      nps_main.sim_time = fdm.time;
      guard++;
      if (guard > 2) {
        //If we are too much behind, catch up incrementaly
        break;
      }
    }
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
      printf("MAIN THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}


