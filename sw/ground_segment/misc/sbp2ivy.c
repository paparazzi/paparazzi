/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * \file sbp2ivy.c
 * \brief SBP GPS packets to Ivy for DGPS and RTK
 *
 * This communicates with an SBP GPS receiver like an
 * Swift-Nav Piksi. Tjis then forwards the Observed messages
 * over the Ivy bus to inject them for DGPS and RTK positioning.
 */

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <libsbp/sbp.h>
#include <libsbp/observation.h>

#include "std.h"
#include "serial_port.h"

/** Used variables **/
struct SerialPort *serial_port;
sbp_state_t sbp_state;
sbp_msg_callbacks_node_t sbp_obs_node;
sbp_msg_callbacks_node_t sbp_obs_dep_a_node;
sbp_msg_callbacks_node_t base_pos_node;

/** Default values **/
uint8_t ac_id = 0;
char *serial_device   = "/dev/ttyUSB0";
uint32_t serial_baud  = B115200;

/** Debugging options */
bool verbose = FALSE;
#define printf_debug    if(verbose == TRUE) printf

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255:2010";
#endif

/*
 * Read bytes from the Piksi UART connection
 * This is a wrapper functions used in the libsbp library
 */
static uint32_t sbp_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  int ret = read(serial_port->fd, buff, n);
  if(ret > 0)
    return ret;
  else
    return 0;
}

static void ivy_send_message(uint8_t packet_id, uint8_t len, uint8_t msg[]) {
  char gps_packet[512], number[5];
  uint8_t i;

  snprintf(gps_packet, 512, "0 GPS_INJECT %d %d %d", ac_id, packet_id, msg[0]); //AC_ID
  for(i = 1; i < len; i++) {
    snprintf(number, 5, ",%d", msg[i]);
    strcat(gps_packet, number);
  }

  IvySendMsg("%s", gps_packet);
  printf_debug("Ivy send: %s\n", gps_packet);
}

/*
 * Callback for the OBS observation message to send it trough GPS_INJECT
 */
static void sbp_obs_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
    ivy_send_message(SBP_MSG_OBS, len, msg);
  }
  printf_debug("Parsed OBS callback\n");
}

/*
 * Callback for the old OBS observation message to send it trough GPS_INJECT
 */
static void sbp_obs_dep_a_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
    ivy_send_message(SBP_MSG_OBS_DEP_A, len, msg);
  }
  printf_debug("Parsed OBS_DEP_A callback\n");
}

/*
 * Callback for the BASE_POS observation message to send it trough GPS_INJECT
 */
static void sbp_base_pos_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
    ivy_send_message(SBP_MSG_BASE_POS, len, msg);
  }
  printf_debug("Parsed BASE_POS callback\n");
}

/**
 * Parse the tty data when bytes are available
 */
static gboolean parse_device_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
  sbp_process(&sbp_state, &sbp_read);
  return TRUE;
}

/** Print the program help */
void print_usage(int argc __attribute__((unused)), char ** argv) {
  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h, --help                Display this help\n"
    "   -v, --verbose             Verbosity enabled\n\n"

    "   -d <device>               The GPS device(default: /dev/ttyUSB0)\n\n";
  fprintf(stderr, usage, argv[0]);
}

int main(int argc, char** argv)
{
  // Parse the options from cmdline
  char c;
  while ((c = getopt (argc, argv, "hvd:b:i:")) != EOF) {
    switch (c) {
    case 'h':
      print_usage(argc, argv);
      exit(EXIT_SUCCESS);
      break;
    case 'v':
      verbose = TRUE;
      break;
    case 'd':
      serial_device = optarg;
      break;
    case 'b':
      serial_baud = atoi(optarg);
      break;
    case 'i':
      ac_id = atoi(optarg);
      break;
    case '?':
      if (optopt == 'd' || optopt == 'b' || optopt == 'i')
        fprintf (stderr, "Option -%c requires an argument.\n", optopt);
      else if (isprint (optopt))
        fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
      print_usage(argc, argv);
      exit(EXIT_FAILURE);
    default:
      abort();
    }
  }

  // Create the Ivy Client
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit("natnet2ivy", "natnet2ivy READY", 0, 0, 0, 0);
  IvyStart(ivy_bus);

  // Start the tty device
  printf_debug("Opening tty device %s...\n", serial_device);
  serial_port = serial_port_new();
  int ret = serial_port_open_raw(serial_port, serial_device, serial_baud);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    serial_port_free(serial_port);
    exit(EXIT_FAILURE);
  }

  // Setup SBP callbacks
  printf_debug("Setup SBP callbacks...\n");
  sbp_state_init(&sbp_state);
  sbp_register_callback(&sbp_state, SBP_MSG_OBS, &sbp_obs_callback, NULL, &sbp_obs_node);
  sbp_register_callback(&sbp_state, SBP_MSG_OBS_DEP_A, &sbp_obs_dep_a_callback, NULL, &sbp_obs_dep_a_node);
  sbp_register_callback(&sbp_state, SBP_MSG_BASE_POS, &sbp_base_pos_callback, NULL, &base_pos_node);

  // Add IO watch for tty connection
  printf_debug("Adding IO watch...\n");
  GIOChannel *sk = g_io_channel_unix_new(serial_port->fd);
  g_io_add_watch(sk, G_IO_IN, parse_device_data, NULL);

  // Run the main loop
  printf_debug("Started sbp2ivy for aircraft id %d!\n", ac_id);
  g_main_loop_run(ml);

  return 0;
}
