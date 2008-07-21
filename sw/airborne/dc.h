#ifndef DC_H
#define DC_H

#include "std.h"
#include "led.h"
#include "airframe.h"
#include "ap_downlink.h"
#include "estimator.h"
#include "gps.h"




extern uint8_t dc_timer;

extern uint8_t dc_periodic_shutter;
/* In s. If non zero, period of automatic calls to dc_shutter() */
extern uint8_t dc_shutter_timer;
/* In s. Related counter */

extern uint8_t dc_utm_threshold;
/* In m. If non zero, automatic shots when greater than utm_north % 100 */



#define SHUTTER_DELAY 2  /* 4Hz -> 0.5s */

static inline uint8_t dc_shutter( void ) {
  dc_timer = SHUTTER_DELAY; 
  LED_OFF(DC_SHUTTER_LED);

  int16_t phi = DegOfRad(estimator_phi);
  int16_t theta = DegOfRad(estimator_theta);
  DOWNLINK_SEND_DC_SHOT( &gps_utm_east, &gps_utm_north, &gps_utm_zone, &gps_course, &estimator_z, &phi, &theta);

  return 0;
}

static inline uint8_t dc_zoom( void ) {
  dc_timer = SHUTTER_DELAY; 
  LED_OFF(DC_ZOOM_LED);
  return 0;
}

#define dc_Shutter(_) { dc_shutter(); }
#define dc_Zoom(_) { dc_zoom(); }
#define dc_Periodic(s) { dc_periodic_shutter = s; dc_shutter_timer = s; }


#define dc_init() { /* initialized as leds */ dc_periodic_shutter = 0; } /* Output */

/* 4Hz */
static inline void dc_periodic( void ) {
  if (dc_timer) {
    dc_timer--;
  } else {
    LED_ON(DC_SHUTTER_LED);
    LED_ON(DC_ZOOM_LED);
  }

  if (dc_periodic_shutter) {
    RunOnceEvery(4, 
    {
      if (dc_shutter_timer) {
	dc_shutter_timer--;
      } else {
	dc_shutter();
	dc_shutter_timer = dc_periodic_shutter;
      }
    });
  }
}

static inline void dc_shot_on_utm_north_close_to_100m_grid( void ) {
  if (dc_utm_threshold && !dc_timer) {
    uint32_t dist_to_100m_grid = (gps_utm_north) / 100 % 100;
    if (dist_to_100m_grid < dc_utm_threshold || 100 - dist_to_100m_grid < dc_utm_threshold)
      dc_shutter();
  }
}

#endif // DC_H
