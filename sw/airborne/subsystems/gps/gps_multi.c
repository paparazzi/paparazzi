/*
 * Copyright (C) 2015 Michael Sierra <sierramichael.a@gmail.com>
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

/**
 * @file subsystems/gps/gps_multi.c
 *
 * multigps implementation
 */

#include "subsystems/gps/gps_multi.h"
//#include "subsystems/gps.h"
#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "subsystems/abi.h"
#include "subsystems/settings.h"
#include "mcu_periph/sys_time.h"
#include "generated/settings.h"

#if GPS_USE_LATLONG
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#endif

#ifndef PrimaryGpsImpl
#warning "PrimaryGpsImpl not set!"
#else
PRINT_CONFIG_VAR(PrimaryGpsImpl)
#endif
#ifndef SecondaryGpsImpl
#warning "SecondaryGpsImpl not set!"
#else
PRINT_CONFIG_VAR(SecondaryGpsImpl)
#endif

#define __PrimaryGpsRegister(_x) _x ## _gps_register()
#define _PrimaryGpsRegister(_x) __PrimaryGpsRegister(_x)
#define PrimaryGpsRegister() _PrimaryGpsRegister(PrimaryGpsImpl)

#define __SecondaryGpsRegister(_x) _x ## _gps_register()
#define _SecondaryGpsRegister(_x) __SecondaryGpsRegister(_x)
#define SecondaryGpsRegister() _SecondaryGpsRegister(SecondaryGpsImpl)

#define gps_primary GpsInstances[PRIMARY_GPS_INSTANCE].gps_s
#define gps_secondary GpsInstances[SECONDARY_GPS_INSTANCE].gps_s

#define TIME_TO_SWITCH 10000 //ten s in ms

static uint8_t current_gps_impl = 0;
static uint32_t time_since_last_gps_switch;
static uint32_t time_since_last_piksi_heartbeat;
uint8_t multi_gps_mode;


/* gps structs */
struct GpsInstance {
  //uint32_t type;
  MultiGpsInit init;
  MultiGpsEvent event;
  struct GpsState *gps_s;
  struct GpsTimeSync *timesync;
};

struct GpsInstance GpsInstances[GPS_MAX_INSTANCES];

/*
 * register gps structs for callback
 */
void gps_register_impl(MultiGpsInit init, MultiGpsEvent event, struct GpsState *gps_s, struct GpsTimeSync *timesync, int8_t instance)
{
  GpsInstances[instance].init = init;
  GpsInstances[instance].event = event;
  GpsInstances[instance].gps_s = gps_s;
  GpsInstances[instance].timesync = timesync;

  GpsInstances[instance].init();
}

/*
 * set up telemetry to monitor two gps at the same time
 */
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_multi_gps_int_primary(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MULTI_GPS_INT_PRIMARY(trans, dev, AC_ID,
                        &gps_primary->ecef_pos.x, &gps_primary->ecef_pos.y, &gps_primary->ecef_pos.z,
                        &gps_primary->lla_pos.lat, &gps_primary->lla_pos.lon, &gps_primary->lla_pos.alt,
                        &gps_primary->hmsl,
                        &gps_primary->ecef_vel.x, &gps_primary->ecef_vel.y, &gps_primary->ecef_vel.z,
                        &gps_primary->pacc, &gps_primary->sacc,
                        &gps_primary->tow,
                        &gps_primary->pdop,
                        &gps_primary->num_sv,
                        &gps_primary->fix);
  // send SVINFO for available satellites that have new data
  //send_svinfo_available(trans, dev);
}

static void send_multi_gps_int_secondary(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MULTI_GPS_INT_SECONDARY(trans, dev, AC_ID,
                        &gps_secondary->ecef_pos.x, &gps_secondary->ecef_pos.y, &gps_secondary->ecef_pos.z,
                        &gps_secondary->lla_pos.lat, &gps_secondary->lla_pos.lon, &gps_secondary->lla_pos.alt,
                        &gps_secondary->hmsl,
                        &gps_secondary->ecef_vel.x, &gps_secondary->ecef_vel.y, &gps_secondary->ecef_vel.z,
                        &gps_secondary->pacc, &gps_secondary->sacc,
                        &gps_secondary->tow,
                        &gps_secondary->pdop,
                        &gps_secondary->num_sv,
                        &gps_secondary->fix);
  // send SVINFO for available satellites that have new data
  //send_svinfo_available(trans, dev);
}

static void send_piksi_heartbeat(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_PIKSI_HEARTBEAT(trans, dev, AC_ID,
                        &time_since_last_piksi_heartbeat);
}

#endif

/* initialize all gps units */
void gps_impl_init(void) {
  time_since_last_gps_switch = get_sys_time_msec();

  multi_gps_mode = MULTI_GPS_MODE;

  uint8_t i;
  for ( i = 0 ; i < GPS_MAX_INSTANCES ; i++) {
    GpsInstances[i].init = NULL;
    GpsInstances[i].event = NULL;
    GpsInstances[i].gps_s = NULL;
    GpsInstances[i].timesync = NULL;
  }

#ifdef PrimaryGpsImpl
  PrimaryGpsRegister();
#endif
#ifdef SecondaryGpsImpl
  SecondaryGpsRegister();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "MULTI_GPS_INT_PRIMARY", send_multi_gps_int_primary);
  register_periodic_telemetry(DefaultPeriodic, "MULTI_GPS_INT_SECONDARY", send_multi_gps_int_secondary);

#ifdef PIKSI_HEARTBEAT_MSG
  register_periodic_telemetry(DefaultPeriodic, "PIKSI_HEARTBEAT", send_piksi_heartbeat);
#endif
#endif

}

void piksi_heartbeat(void)
{
  time_since_last_piksi_heartbeat = get_sys_time_msec();
}


/*
 * publish gps state to gps.c/h and Abi
 */
static void gps_multi_publish(struct GpsState *gps_s)
{
  uint32_t now_ts = get_sys_time_usec();

  gps = *gps_s;
  AbiSendMsgGPS(GPS_UBX_ID, now_ts, &gps);
}

/*
 * switching
 * switch only after a set amount of time so as to not confuse ins
 */
 static uint8_t gps_multi_switch(void)
 {
  time_since_last_gps_switch = get_sys_time_msec();
  if (multi_gps_mode == GPS_MODE_PRIMARY) {
    return PRIMARY_GPS_INSTANCE;
  } else if (multi_gps_mode == GPS_MODE_SECONDARY) {
    return SECONDARY_GPS_INSTANCE;
  } else {
#ifdef GPS_PRIMARY_PIKSI
  if (GpsInstances[PRIMARY_GPS_INSTANCE].gps_s->fix > GpsInstances[SECONDARY_GPS_INSTANCE].gps_s->fix) {
    return PRIMARY_GPS_INSTANCE;
  } else {
    return SECONDARY_GPS_INSTANCE;
  }
#endif
#ifdef GPS_SECONDARY_PIKSI
  if (GpsInstances[SECONDARY_GPS_INSTANCE].gps_s->fix > GpsInstances[PRIMARY_GPS_INSTANCE].gps_s->fix) {
    return SECONDARY_GPS_INSTANCE;
  } else {
    return PRIMARY_GPS_INSTANCE;
  }
#endif
  }
 }

/* 
 * handle gps switching and updating gps instances 
 */
void gps_multi_event(void) {
  // run each gps event
  uint8_t i;
  for ( i = 0 ; i < GPS_MAX_INSTANCES ; i++) {
    GpsInstances[i].event();
  }
  //switch gps if one has better fix, depending on mode, or if one looses fix altogether
  if ((get_sys_time_msec() - time_since_last_gps_switch) > TIME_TO_SWITCH || 
    multi_gps_mode != GPS_MODE_AUTO || 
    GpsInstances[PRIMARY_GPS_INSTANCE].gps_s->fix == GPS_FIX_NONE ||
    GpsInstances[SECONDARY_GPS_INSTANCE].gps_s->fix == GPS_FIX_NONE )
  {
    current_gps_impl = gps_multi_switch();
  }
  // update main gps state
  gps_multi_publish(GpsInstances[current_gps_impl].gps_s);
}
