/*
 * Copyright (C) 2009  Martin Mueller
 *               2019  Freek van Tienen <freek.v.tienen@gmail.com>
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

/** \file gpsd2ivy.c
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
    <field name="airspeed" type="float" unit="m/s"/>
  </message>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <glib.h>
#include <unistd.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "gps.h"

#define MSG_DEST	"ground"
#define MSG_NAME    "FLIGHT_PARAM"
#define MSG_ID		"GCS"

#define TIMEOUT_PERIOD 10

#if GPSD_API_MAJOR_VERSION <= 6
#define TIME_T double
#define TIME_INIT 0.
// note that this test is stupid
#define IS_TIME_EQUAL(_a, _b) (_a == _b)
#define TIME_IN_SEC(_a) (_a)
#define GPS_READ(_a) gps_read(_a)
#else
#define TIME_T timespec_t
#define TIME_INIT {0}
#define IS_TIME_EQUAL(_a, _b) ((_a.tv_sec == _b.tv_sec) && (_a.tv_nsec == _b.tv_nsec))
#define TIME_IN_SEC(_a) ((double)(_a.tv_sec + _a.tv_nsec * 1e-9))
#define GPS_READ(_a) gps_read(_a, NULL, 0)
#endif

struct gps_data_t *gpsdata;
gboolean verbose;
char* server;
char* port;
char* ivy_bus;
char* ac;
char* wp;

static void update_gps(struct gps_data_t *gpsdata,
                       char *message,
                       size_t len)
{
    static TIME_T fix_time = TIME_INIT;
    double fix_track = 0;
    double fix_speed = 0;
    double fix_altitude = 0;
    double fix_climb = 0;

    if ((isnan(gpsdata->fix.latitude) == 0) &&
        (isnan(gpsdata->fix.longitude) == 0) &&
        (gpsdata->fix.mode >= MODE_2D) &&
        !IS_TIME_EQUAL(gpsdata->fix.time, fix_time))
    {
        if (!isnan(gpsdata->fix.track))
            fix_track = gpsdata->fix.track;
        if (!isnan(gpsdata->fix.speed))
            fix_speed = gpsdata->fix.speed;

        if (gpsdata->fix.mode >= MODE_3D)
        {
            if (!isnan(gpsdata->fix.altitude))
                fix_altitude = gpsdata->fix.altitude;
            if (!isnan(gpsdata->fix.climb))
                fix_climb = gpsdata->fix.climb;
        }

        if (verbose)
            printf("sending gps info viy Ivy: lat %f, lon %f, speed %g, course %g, alt %g, climb %g\n",
                   gpsdata->fix.latitude, gpsdata->fix.longitude, fix_speed, fix_track, fix_altitude, fix_climb);

        IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f",
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
                TIME_IN_SEC(gpsdata->fix.time),
                0, // itow
                0.0); // airspeed


        if(strcmp(ac, "NONE") != 0) {
            IvySendMsg("%s TARGET_POS %s %s %d %d %d %f %f %f", "0", "0", ac, (int)(gpsdata->fix.latitude * 1e7), (int)(gpsdata->fix.longitude * 1e7), (int)(fix_altitude* 1000), fix_speed, fix_climb, fix_track);
            if (verbose)
                printf("sending TARGET_POS for aircraft %s\n", ac);
        }
        if(strcmp(ac, "NONE") != 0 && strcmp(wp, "NONE") != 0) {
		    IvySendMsg("%s MOVE_WP %s %s %d %d %d", "0", wp, ac, (int)(gpsdata->fix.latitude * 1e7), (int)(gpsdata->fix.longitude * 1e7), (int)(20. * 1000));
            if (verbose)
                printf("sending waypoint %s for aircraft %s\n", wp, ac);
        }

        fix_time = gpsdata->fix.time;
    }
    else
    {
        if (verbose) {
            printf("ignoring gps data: lat %f, lon %f, mode %d, time %f\n", gpsdata->fix.latitude,
                   gpsdata->fix.longitude, gpsdata->fix.mode, TIME_IN_SEC(gpsdata->fix.time));
        }
    }
}

static gboolean gps_periodic(gpointer data __attribute__ ((unused)))
{
    if (gps_waiting (gpsdata, TIMEOUT_PERIOD)) {
        if (GPS_READ (gpsdata) == -1) {
            perror("gps read error");
        } else {
            update_gps(gpsdata, NULL, 0);
        }
    }
    return TRUE;
}

gboolean parse_args(int argc, char** argv)
{
    verbose = FALSE;
    server = "localhost";
    port = DEFAULT_GPSD_PORT;
    ac = "NONE";
    wp = "NONE";
#ifdef __APPLE__
    ivy_bus = "224.255.255.255";
#else
    ivy_bus = "127.255.255.255";
#endif

    static const char* usage =
        "Usage: %s [options]\n"
        " Options :\n"
        "   -h --help                              Display this help\n"
        "   -v --verbose                           Print verbose information\n"
        "   --server <gpsd server>                 e.g. localhost\n"
        "   --port <gpsd port>                     e.g. 2947\n"
        "   --ivy_bus <ivy bus>                    e.g. 127.255.255.255\n"
        "   --ac <ac_id>                           e.g. 17\n"
        "   --wp <wp_id>                           e.g. 2\n";

    while (1) {

        static struct option long_options[] = {
            {"ivy_bus", 1, NULL, 0},
            {"server", 1, NULL, 0},
            {"port", 1, NULL, 0},
            {"ac", 1, NULL, 0},
            {"wp", 1, NULL, 0},
            {"help", 0, NULL, 'h'},
            {"verbose", 0, NULL, 'v'},
            {0, 0, 0, 0}
        };
        int option_index = 0;
        int c = getopt_long(argc, argv, "vh",
                            long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
            case 0:
                switch (option_index) {
                    case 0:
                        ivy_bus = strdup(optarg); break;
                    case 1:
                        server = strdup(optarg); break;
                    case 2:
                        port = strdup(optarg); break;
                    case 3:
                        ac = strdup(optarg); break;
                    case 4:
                        wp = strdup(optarg); break;
                    default:
                        break;
                }
                break;

            case 'v':
                verbose = TRUE;
                break;

            case 'h':
                fprintf(stderr, usage, argv[0]);
                exit(0);

            default: /* ’?’ */
                printf("?? getopt returned character code 0%o ??\n", c);
                fprintf(stderr, usage, argv[0]);
                exit(EXIT_FAILURE);
        }
    }
    return TRUE;
}

int main(int argc, char** argv)
{
    int ret = 0;

    if (!parse_args(argc, argv))
        return 1;

    GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

    gpsdata = malloc(sizeof(struct gps_data_t));

    printf("Connecting to gpsd server %s, port %s\n", server, port);

    ret = gps_open(server, port, gpsdata);
    if (ret != 0) {
        perror("error connecting to gpsd");
        return 1;
    }

    gps_stream(gpsdata, WATCH_ENABLE, NULL);

    IvyInit ("GPSd2Ivy", "GPSd2Ivy READY", NULL, NULL, NULL, NULL);
    IvyStart(ivy_bus);

    g_timeout_add(TIMEOUT_PERIOD, gps_periodic, NULL);

    g_main_loop_run(ml);

    (void) gps_stream(gpsdata, WATCH_DISABLE, NULL);
    (void) gps_close (gpsdata);

    free(gpsdata);

    return ret;
}
