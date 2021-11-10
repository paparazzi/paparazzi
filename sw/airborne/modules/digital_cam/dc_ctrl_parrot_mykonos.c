/*
 * Copyright (C) 2018 OpenUAS
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
 * NOTES: For now only the front cam is used, feel free to extend.
 * Reason is that bottom_cam is normally in use for mapping or Optic flow and the like.
 * See: https://github.com/Parrot-Developers/disco-opensource/
 */

#include "modules/digital_cam/dc_ctrl_parrot_mykonos.h"
#include "generated/modules.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
// Include Standard Camera Control Interface
#include "modules/digital_cam/dc.h"

/** Time in seconds to start/stop recording or take a picture */
#define DC_CTRL_PARROT_MYKONOS_RECORD_DELAY 0.05

/** delay in milli-seconds before logging after a shot in ms */
#define DC_CTRL_PARROT_MYKONOS_LOG_DELAY 50

/** Get timer from delay based on periodic freq from modules.h */
#define DC_CTRL_PARROT_MYKONOS_TIMER_OF_DELAY(_delay) ((uint32_t)(_delay * DC_CTRL_PARROT_MYKONOS_PERIODIC_FREQ))

/** autoshoot timer delay based on periodic freq from modules.h */
#ifndef DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DELAY
#define DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DELAY 5.0
#endif
#define DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_TIMER_OF_DELAY(_delay) ((uint32_t)(_delay * DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_FREQ))

/** Send report */
#if DC_CTRL_PARROT_MYKONOS_SYNC_SEND

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "modules/gps/gps.h"

#include "subsystems/datalink/telemetry.h" //? or just downlink.h should be enough

static inline void dc_ctrl_parrot_mykonos_send_shot_position(void)
{
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = stateGetHorizontalSpeedNorm_f() * 10;

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &dc_ctrl_parrot_mykonos.photo_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);
}
#endif


/*
  The pimpctl command is available per default on a Disco and can be used for
  triggering various camera related tasks:

  Possible arguments for pimpctl command
  list-cameras                        = print the list of available cameras
  stream-start front <address> <port> = start streaming video
  stream-stop front                   = stop streaming video
  take-picture <camera-name>          = ehh, take a photo indeed
  recording-start <camera-name>       = start recording video
  recording-stop <camera-name>        = stop recording video
  set-controller <camera-name> <controller_id>  = set a new camera controller
                                   0 -> USER FLAT (yaw axis is free)
                                   1 -> USER ABSOLUTE (fully stabilized)
                                   2 -> USER FPV (fixed relatively to the drone))
                                   3 -> AUTOPILOT
  cam-orientation <camera-name> <y> <p> <r>       = set camera orientation (in degree CCW)
                                   y -> yaw pitch rotatio (psi euler angle)
                                   p -> pitch rotation (theta euler angle)
                                   r -> roll rotation (phi euler angle)
*/

#if DC_CTRL_PARROT_MYKONOS_LOG
#include "state.h"
#include "modules/gps/gps.h"
#endif

struct Dc_Ctrl_Parrot_Mykonos dc_ctrl_parrot_mykonos;

void dc_ctrl_parrot_mykonos_init(void)
{
  // Call common DC init
  dc_init();

  dc_ctrl_parrot_mykonos.status = DC_CTRL_PARROT_MYKONOS_NONE;
  dc_ctrl_parrot_mykonos.timer = 0;
  dc_ctrl_parrot_mykonos.photo_nr = 0;
  dc_ctrl_parrot_mykonos.autoshoot = 0;
  dc_ctrl_parrot_mykonos.log_delay = 0;

#ifndef SITL
  int ret __attribute__((unused));
  //ret = system("kk"); //No need kill original AP proceess since new  AP is already running and that killed original process

  //TIP: With media-ctl much more can be done for the parameters of the camera only basics are set.
  ret = system("media-ctl -l \'\"mt9f002 0-0010\":0->\"avicam.0\":0[1]\'");
  ret = system("media-ctl -l \'\"avicam_dummy_dev.0\":0->\"avicam.0\":0[0]\'");//No bottomcam, used internaly in AP

  ret = system("prestart dxowrapperd");
  ret = system("prestart pimp"); // pimp = Parrot IMaging Process
  //ret = system("pimpctl list-cameras"); //TODO look for 1 or more then define a variable to use or not
#if DC_CTRL_PARROT_MYKONOS_STREAM_AT_STARTUP
  dc_ctrl_parrot_mykonos_command(DC_CTRL_PARROT_MYKONOS_STREAM_START);
#endif

#else
  //Start your local processes so simulated flight with sim shooting can be perfromed
  //E.g. using a tile map interface of a pre-recorded stream or a 3D generated image
  //FIXME: make a complete example, he, it's opensouce and involves your work indeed...
  //ret = system("whateveryouwantotstartlocallyaddithere");
#endif
}

void dc_ctrl_parrot_mykonos_periodic(void)
{
  //Nice 'n ugly use of True/False
  if (dc_ctrl_parrot_mykonos.timer) {
    dc_ctrl_parrot_mykonos.timer--;
  } else {
	  dc_ctrl_parrot_mykonos_command(DC_CTRL_PARROT_MYKONOS_SHOOT);
  }
  // test log delay if set
  if (dc_ctrl_parrot_mykonos.log_delay) {
#ifndef SITL
    if (get_sys_time_msec() > dc_ctrl_parrot_mykonos.log_delay) { //FIXME: Could also happen in SITL...
#endif
#if DC_CTRL_PARROT_MYKONOS_LOG
      dc_ctrl_parrot_mykonos_log_shot_position();
#endif

#if DC_CTRL_PARROT_MYKONOS_SYNC_SEND
      dc_ctrl_parrot_mykonos_send_shot_position();
#endif
      // increment photo number
      dc_ctrl_parrot_mykonos.photo_nr++;
      // unset log delay
      dc_ctrl_parrot_mykonos.log_delay = 0;
#ifndef SITL
    }
#endif
  }

  // Common DC Periodic task
  dc_periodic();
}

//FIXME muiltiple cams and same DC cam API do no go together,
// so for the time being not used here, also one needs to add SITL options

/* Command the Camera
 * Intermidate function so the universal PPRZ Camra API can be used
 * No need to change flightplan, script or otherwhise
 * should work on both Fixedwing and Rotorcraft
*/

#ifdef SITL
void dc_send_command(uint8_t cmd)
{
	 //Nothing yet, empty framework here so sim compiles
	switch (cmd) {
    default:
      break;
	}

  // call command send_command function
  dc_send_command_common(cmd);
}
#endif


/* Execute the Shoot, Record and Stream commands */
void dc_ctrl_parrot_mykonos_command(enum dc_ctrl_parrot_mykonos_status cmd)
{
  int ret __attribute__((unused));

  dc_ctrl_parrot_mykonos.status = cmd;
  switch (cmd) {
    case DC_CTRL_PARROT_MYKONOS_RECORD_START:
#ifndef SITL
      ret = system("pimpctl recording-start front");
#else
      //ret = system("addyourlocalsitlcommandshere");
#endif
      break;
    case DC_CTRL_PARROT_MYKONOS_RECORD_STOP:
      dc_ctrl_parrot_mykonos.timer = DC_CTRL_PARROT_MYKONOS_TIMER_OF_DELAY(DC_CTRL_PARROT_MYKONOS_RECORD_DELAY);
#ifndef SITL
      ret = system("pimpctl recording-stop front");
#else
      //ret = system("addyourlocalsitlcommandshere");
#endif
      break;
    case DC_CTRL_PARROT_MYKONOS_SHOOT:
#ifndef SITL
      ret = system("pimpctl take-picture front");
#else
      //ret = system("addyourlocalsitlcommandshere");
#endif

      break;
    case DC_CTRL_PARROT_MYKONOS_STREAM_START:
      /*
       * Note that while .0 as destination IP works multicasting it introduces latency.
      To avoid this use the real target IP of the video viewer device e.g. the GCS
      quick and dirty; get ip from latest lease:	./data/lib/misc/dhcp_eth0.leases
      and look for the name of your host you want to target.

      TIP: Example to view stream on Host PC:

    	gst-launch-1.0 udpsrc port=55004 ! "application/x-rtp, payload=96" ! rtph264depay ! avdec_h264 ! autovideosink

      But there are many ways to Rome...

    	 */
#ifndef SITL
      ret = system("pimpctl stream-start front 192.168.42.0 55004");//FIXME: Option to target only IP, for less delay
#else
//ret = system("addyourlocalsitlcommandshere");
#endif
      break;
    case DC_CTRL_PARROT_MYKONOS_STREAM_STOP:
#ifndef SITL
      ret = system("pimpctl stream-stop front 192.168.42.0 55004");
#else
//ret = system("addyourlocalsitlcommandshere");
#endif
      break;
    case DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_START:
      dc_ctrl_parrot_mykonos.timer = DC_CTRL_PARROT_MYKONOS_TIMER_OF_DELAY(DC_CTRL_PARROT_MYKONOS_RECORD_DELAY);
#ifndef SITL
      ret = system("pimpctl take-picture front");
#else
//ret = system("addyourlocalsitlcommandshere");
#endif
      dc_ctrl_parrot_mykonos.log_delay = get_sys_time_msec() + DC_CTRL_PARROT_MYKONOS_LOG_DELAY;
      dc_ctrl_parrot_mykonos.last_shot_pos = *stateGetPositionEnu_f();
      break;
    case DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_STOP:
      //nix
      break;
    default:
      break;
  }
}

void dc_ctrl_parrot_mykonos_autoshoot(void)
{
// Wait a minimum time between two shots
  if (dc_ctrl_parrot_mykonos.autoshoot) {
    dc_ctrl_parrot_mykonos.autoshoot--;
  } else {
    // test distance if needed
    // or take picture if first of the sequence
#ifdef DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DIST
    struct EnuCoor_f pos = *stateGetPositionEnu_f();
    struct FloatVect2 d_pos;
    d_pos.x = pos.x - dc_ctrl_parrot_mykonos.last_shot_pos.x;
    d_pos.y = pos.y - dc_ctrl_parrot_mykonos.last_shot_pos.y;
    if (VECT2_NORM2(d_pos) > (DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DIST * DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DIST)
        || dc_ctrl_parrot_mykonos.status == DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_START) {
#endif
      // take a picture
      dc_ctrl_parrot_mykonos_command(DC_CTRL_PARROT_MYKONOS_SHOOT);
      // reset timer
      dc_ctrl_parrot_mykonos.autoshoot = DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_TIMER_OF_DELAY(DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DELAY);
#ifdef DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_DIST
    }
#endif
  }
}

void dc_ctrl_parrot_mykonos_autoshoot_start(void)
{
  // Start taking a picture immediately
  dc_ctrl_parrot_mykonos.autoshoot = 0;
  dc_ctrl_parrot_mykonos.status = DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_START;
}
