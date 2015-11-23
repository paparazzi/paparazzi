/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/stereocam/droplet/stereocam_droplet.c"
 * @author C. DW
 *
 */

#include "modules/stereocam/droplet/stereocam_droplet.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

#include "firmwares/rotorcraft/navigation.h"



// Serial Port
#include "mcu_periph/uart.h"
PRINT_CONFIG_VAR(STEREO_UART)

// define coms link for stereocam
#define STEREO_PORT   (&((STEREO_UART).device))
struct link_device *xdev = STEREO_PORT;

#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)
#define StereoSend1(c) STEREO_PORT->put_byte(STEREO_PORT->periph, c)
#define StereoUartSend1(c) StereoSend1(c)
#define StereoSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) StereoSend1(_dat[i]); };
#define StereoUartSetBaudrate(_b) uart_periph_set_baudrate(STEREO_PORT, _b);
#define StereoChAvailable()(xdev->char_available(xdev->periph))






// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "led.h"





// Module data
struct AvoidNavigationStruct {
  uint8_t mode; ///< 0 = straight, 1 =  right, 2 = left, ...
  uint8_t stereo_bin[8];
  uint8_t timeout;
};

struct AvoidNavigationStruct avoid_navigation_data;







static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c)
{
  // Protocol is one byte only: store last instance
  avoid_navigation_data.stereo_bin[0] = c;
  avoid_navigation_data.timeout = 20;
}


void stereocam_droplet_init(void)
{
  // Do nothing
  avoid_navigation_data.mode = 0;
  avoid_navigation_data.timeout = 0;
}
void stereocam_droplet_periodic(void)
{

  static float heading = 0;

  // Read Serial
  while (StereoChAvailable()) {
    stereo_parse(StereoGetch());
  }

  if (avoid_navigation_data.timeout <= 0)
    return;

  avoid_navigation_data.timeout --;

  // Results
  DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, avoid_navigation_data.stereo_bin);

  volatile bool_t once = TRUE;
  // Move waypoint with constant speed in current direction
  if (
    (avoid_navigation_data.stereo_bin[0] == 97) ||
    (avoid_navigation_data.stereo_bin[0] == 100)
  ) {
    once = TRUE;
    struct EnuCoor_f enu;
    enu.x = waypoint_get_x(WP_GOAL);
    enu.y = waypoint_get_y(WP_GOAL);
    enu.z = waypoint_get_alt(WP_GOAL);
    float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
    float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
    enu.x += (sin_heading * 1.3 / 20);
    enu.y += (cos_heading * 1.3 / 20);
    waypoint_set_enu(WP_GOAL, &enu);
  } else if (avoid_navigation_data.stereo_bin[0] == 98) {
    // STOP!!!
    if (once) {
      NavSetWaypointHere(WP_GOAL);
      once = FALSE;
    }
  } else {
    once = TRUE;
  }


  switch (avoid_navigation_data.stereo_bin[0]) {
    case 99:     // Turn
      heading += 4;
      if (heading > 360) { heading = 0; }
      nav_set_heading_rad(RadOfDeg(heading));
      break;
    default:    // do nothing
      break;
  }

#ifdef STEREO_LED
  if (obstacle_detected) {
    LED_ON(STEREO_LED);
  } else {
    LED_OFF(STEREO_LED);
  }
#endif

}



