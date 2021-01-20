/*
 * $Id$
 *  
 * Copyright (C) 2009  Martin Mueller
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

/** \file gpsd_ivy.c
 *  \brief GPSd forwarder
 *
 *   This receives position information through gpsd and forwards it
 * through the ivy bus to the gcs to display the ground station location
 * on the map
 */

/* 
  <message name="FLIGHT_PARAM" id="11">
    <field name="ac_id"  type="string"/>
    <field name="roll"   type="float" unit="deg"/>
    <field name="pitch"  type="float" unit="deg"/>
    <field name="heading" type="float" unit="deg"/>
    <field name="lat"    type="float" unit="deg"/>
    <field name="long"   type="float" unit="deg"/>
    <field name="speed"  type="float" unit="m/s"/>
    <field name="course" type="float" unit="deg" format="%.1f"/>
    <field name="alt"    type="float" unit="m"/>
    <field name="climb"  type="float" unit="m/s"/>
    <field name="agl"    type="float" unit="m"/>
    <field name="unix_time"    type="float" unit="s (Unix time)"/>
    <field name="itow"   type="uint32" unit="ms"/>
  </message>
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <glib.h>
#include <unistd.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "gps.h"

#define MSG_DEST	"ground"
#define MSG_NAME 	"FLIGHT_PARAM"
#define MSG_ID		"GCS"

#define TIMEOUT_PERIOD 200

struct gps_data_t *gpsdata;

static void update_gps(struct gps_data_t *gpsdata,
                       char *message,
                       size_t len)
{
    static double fix_time = 0;
    double fix_track = 0;
    double fix_speed = 0;
    double fix_altitude = 0;
    double fix_climb = 0;

    if ((isnan(gpsdata->fix.latitude) == 0) &&
        (isnan(gpsdata->fix.longitude) == 0) &&
        (isnan(gpsdata->fix.time) == 0) &&
        (gpsdata->fix.mode >= MODE_2D) &&
        (gpsdata->fix.time != fix_time))
    {
        if (isnan(gpsdata->fix.track) != 0) fix_track = gpsdata->fix.track;
        if (isnan(gpsdata->fix.speed) != 0) fix_speed = gpsdata->fix.speed;

        if (gpsdata->fix.mode >= MODE_3D)
        {
            if (isnan(gpsdata->fix.altitude) != 0) fix_altitude = gpsdata->fix.altitude;
            if (isnan(gpsdata->fix.climb) != 0) fix_climb = gpsdata->fix.climb;
        }

    	IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d",
                MSG_DEST,
                MSG_NAME,
                MSG_ID, // ac_id
                0.0, // roll,
                0.0, // pitch,
                0.0, // heading
                gpsdata->fix.latitude,
                gpsdata->fix.longitude,
                fix_speed,
                fix_track, // course
                fix_altitude,
                fix_climb,
                0.0, // agl
                gpsdata->fix.time,
                0); // itow

        fix_time = gpsdata->fix.time;
    }
}

static gboolean gps_periodic(gpointer data __attribute__ ((unused)))
{
        if (gps_waiting (gpsdata, 150)) {
            if (gps_read (gpsdata) == -1) {
		    perror("gps read error");
	    } else {
		    update_gps(gpsdata, NULL, 0);
	    }
	}
	return true;
}

int main(int argc, char *argv[])
{
    char *port = DEFAULT_GPSD_PORT;
    int ret = 0;
    GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
    

    gpsdata = malloc(sizeof(struct gps_data_t));

    ret = gps_open(NULL, port, gpsdata);
    if (ret != 0) {
	    perror("error connecting to gpsd");
	    return 1;
    }

    gps_stream(gpsdata, WATCH_ENABLE, NULL);
    
    IvyInit ("GPSd2Ivy", "GPSd2Ivy READY", NULL, NULL, NULL, NULL);
    IvyStart("224.255.255.255:2010");

    g_timeout_add(TIMEOUT_PERIOD, gps_periodic, NULL);

    g_main_loop_run(ml);

    (void) gps_stream(gpsdata, WATCH_DISABLE, NULL);
    (void) gps_close (gpsdata);

    free(gpsdata);

    return ret;
}

