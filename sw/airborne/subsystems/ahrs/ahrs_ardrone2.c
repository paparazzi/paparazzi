/*
 * Copyright (C) 2012-2013 Freek van Tienen
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
 * @file subsystems/ahrs/ahrs_ardrone2.c
 * AHRS implementation for ardrone2-sdk based on AT-commands.
 *
 * Uses AT-Commands to communicate with ardrone api to retrieve AHRS data
 * and also sets battery level.
 */

#ifdef ARDRONE2_DEBUG
# include <errno.h>
# include <stdio.h>
#endif

#include "ahrs_ardrone2.h"
#include "state.h"
#include "math/pprz_algebra_float.h"
#include "boards/ardrone/at_com.h"
#include "subsystems/electrical.h"

#ifdef USE_GPS_ARDRONE2
#include "subsystems/gps/gps_ardrone2.h"
#endif

struct AhrsARDrone ahrs_impl;
struct AhrsAligner ahrs_aligner;
unsigned char buffer[4096]; //Packet buffer

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_ahrs_ad2(void) {
  DOWNLINK_SEND_AHRS_ARDRONE2(DefaultChannel, DefaultDevice,
      &ahrs_impl.state,
      &ahrs_impl.control_state,
      &ahrs_impl.eulers.phi,
      &ahrs_impl.eulers.theta,
      &ahrs_impl.eulers.psi,
      &ahrs_impl.speed.x,
      &ahrs_impl.speed.y,
      &ahrs_impl.speed.z,
      &ahrs_impl.accel.x,
      &ahrs_impl.accel.y,
      &ahrs_impl.accel.z,
      &ahrs_impl.altitude,
      &ahrs_impl.battery);
}
#endif

void ahrs_init(void) {
  init_at_com();

  //Set navdata_demo to FALSE and flat trim the ar drone
  at_com_send_config("general:navdata_demo", "FALSE");
  at_com_send_ftrim();

  ahrs.status = AHRS_RUNNING;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "AHRS_ARDRONE2", send_ahrs_ad2);
#endif
}

void ahrs_align(void) {

}

#ifdef ARDRONE2_DEBUG
static void dump(const void *_b, size_t s) {
  const unsigned char *b = _b;
  size_t n;

  for(n = 0; n < s; ++n) {
    printf("%02x ", b[n]);
    if (n%16 == 15)
      printf("\n");
  }
  if (n%16 != 0)
    printf("\n");
}
#endif

void ahrs_propagate(void) {
  int l;

  //Recieve the main packet
  l = at_com_recieve_navdata(buffer);
  navdata_t* main_packet = (navdata_t*) &buffer;

#ifdef ARDRONE2_DEBUG
  if (l < 0)
    printf("errno = %d\n", errno);
#endif

  //When this isn't a valid packet return
  if(l < 0 || main_packet->header != NAVDATA_HEADER)
    return;

#ifdef ARDRONE2_DEBUG
  printf("Read %d\n", l);
  dump(buffer, l);
#endif

  //Set the state
  ahrs_impl.state = main_packet->ardrone_state;

  //Init the option
  navdata_option_t* navdata_option = (navdata_option_t*)&(main_packet->options[0]);
  bool_t full_read = FALSE;

  //The possible packets
  navdata_demo_t* navdata_demo;
  navdata_gps_t* navdata_gps;
  navdata_phys_measures_t* navdata_phys_measures;

  //Read the navdata until packet is fully readed
  while(!full_read && navdata_option->size > 0) {
#ifdef ARDRONE2_DEBUG
    printf ("tag = %d\n", navdata_option->tag);
#endif
    //Check the tag for the right option
    switch(navdata_option->tag) {
    case 0: //NAVDATA_DEMO
      navdata_demo = (navdata_demo_t*) navdata_option;

      //Set the AHRS state
      ahrs_impl.control_state = navdata_demo->ctrl_state >> 16;
      ahrs_impl.eulers.phi = navdata_demo->phi;
      ahrs_impl.eulers.theta = navdata_demo->theta;
      ahrs_impl.eulers.psi = navdata_demo->psi;
      ahrs_impl.speed.x = navdata_demo->vx / 1000;
      ahrs_impl.speed.y = navdata_demo->vy / 1000;
      ahrs_impl.speed.z = navdata_demo->vz / 1000;
      ahrs_impl.altitude = navdata_demo->altitude / 10;
      ahrs_impl.battery = navdata_demo->vbat_flying_percentage;

      //Set the ned to body eulers
      struct FloatEulers angles;
      angles.theta = navdata_demo->theta/180000.*M_PI;
      angles.psi = navdata_demo->psi/180000.*M_PI;
      angles.phi = navdata_demo->phi/180000.*M_PI;
      stateSetNedToBodyEulers_f(&angles);

      //Update the electrical supply
      electrical.vsupply = navdata_demo->vbat_flying_percentage;
      break;
    case 3: //NAVDATA_PHYS_MEASURES
      navdata_phys_measures = (navdata_phys_measures_t*) navdata_option;

      //Set the AHRS accel state
      INT32_VECT3_SCALE_2(ahrs_impl.accel, navdata_phys_measures->phys_accs, 9.81, 1000)
      break;
#ifdef USE_GPS_ARDRONE2
    case 27: //NAVDATA_GPS
# ifdef ARDRONE2_DEBUG
      dump(navdata_option, navdata_option->size);
# endif
      navdata_gps = (navdata_gps_t*) navdata_option;

      // Send the data to the gps parser
      gps_ardrone2_parse(navdata_gps);
      break;
#endif
    case 0xFFFF: //CHECKSUM
      //TODO: Check the checksum
      full_read = TRUE;
      break;
    default:
#ifdef ARDRONE2_DEBUG
      printf("NAVDATA UNKNOWN TAG: %d %d\n", navdata_option->tag, navdata_option->size);
#endif
      break;
    }
    navdata_option = (navdata_option_t*) ((uint32_t)navdata_option + navdata_option->size);
  }
}

void ahrs_update_accel(void) {

}

void ahrs_update_mag(void) {

}

void ahrs_update_gps(void) {

}

void ahrs_aligner_init(void) {

}

void ahrs_aligner_run(void) {

}
