
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
#include <sys/select.h>
#include <sys/socket.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "gps.h"

#define MSG_DEST	"ground"
#define MSG_NAME 	"FLIGHT_PARAM"
#define MSG_ID		"GCS"

#define TIMEOUT_PERIOD 200

static struct gps_data_t *gpsdata;
fd_set rfds;

static void update_gps(struct gps_data_t *gpsdata,
                       char *message,
                       size_t len,
                       int level)
{
	IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d",
                MSG_DEST,
                MSG_NAME,
                MSG_ID, //ac_id
                0.0, // roll
                0.0, // pitch
                gpsdata->fix.track, // heading
                gpsdata->fix.latitude,
                gpsdata->fix.longitude,
                gpsdata->fix.speed,
                0.0, // course
                gpsdata->fix.altitude,
                gpsdata->fix.climb,
                0.0, // agl
                gpsdata->fix.time,
                0); // itow
}

static gboolean gps_periodic(gpointer data __attribute__ ((unused)))
{
    struct timeval timeout;
    int ret;

    FD_ZERO(&rfds);
    FD_SET(gpsdata->gps_fd, &rfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    ret = select(gpsdata->gps_fd + 1, &rfds, NULL, NULL, &timeout);

    if (ret == -1)
    {
        perror("socket error\n");
        exit(2);
    }
    else if (ret) gps_poll(gpsdata);

    return 1;
}

int main(void)
{
    char *server = NULL, *port = DEFAULT_GPSD_PORT;
    GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

    gpsdata = gps_open(server, port);

    if (!gpsdata) perror("error connecting to gpsd");

    gps_set_raw_hook(gpsdata, update_gps);

    gps_query(gpsdata, "w+x\n");
  
    IvyInit ("GPSd2Ivy", "GPSd2Ivy READY", NULL, NULL, NULL, NULL);
    IvyStart("127.255.255.255");
  
    g_timeout_add(TIMEOUT_PERIOD, gps_periodic, NULL);
  
    g_main_loop_run(ml);

    return 0;
}

