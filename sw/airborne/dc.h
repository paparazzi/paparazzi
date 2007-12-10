#ifndef DC_H
#define DC_H

#include <inttypes.h>

extern uint8_t dc_timer;

#ifdef SITL

#define dc_Shutter(_) { }
#define dc_Zoom(_) { }

#else /* SITL */

#include "LPC21xx.h"
#include "std.h"
#include "led.h"
#include "airframe.h"

#define SHUTTER_DELAY 2  /* 4Hz */

static inline uint8_t dc_shutter() {
  dc_timer = SHUTTER_DELAY; 
  LED_OFF(DC_SHUTTER_LED);
  return 0;
}

static inline uint8_t dc_zoom() {
  dc_timer = SHUTTER_DELAY; 
  LED_OFF(DC_ZOOM_LED);
  return 0;
}

#define dc_Shutter(_) { dc_shutter(); }
#define dc_Zoom(_) { dc_zoom(); }

#define dc_init() { /* initialized as leds */ } /* Output */

/* 4Hz */
#define dc_periodic() { if (dc_timer) { dc_timer--; } else { LED_ON(DC_SHUTTER_LED); LED_ON(DC_ZOOM_LED); } }

#endif /* ! SITL */

#endif
