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

#define SHUTTER_DELAY 2  /* 4Hz */
#define SHUTTER_BANK 0 /* Grey */
#define SHUTTER_PIN 16

#define ZOOM_BANK 1
#define ZOOM_PIN 20

static inline uint8_t dc_shutter() {
  dc_timer = SHUTTER_DELAY; 
  IO0SET = _BV(SHUTTER_PIN);
  return 0;
}

static inline uint8_t dc_zoom() {
  dc_timer = SHUTTER_DELAY; 
  IO1SET = _BV(ZOOM_PIN);
  return 0;
}

#define dc_Shutter(_) { dc_shutter(); }
#define dc_Zoom(_) { dc_zoom(); }

#define dc_init() { IO0DIR |= _BV(SHUTTER_PIN); IO0CLR = _BV(SHUTTER_PIN); ClearBit(PINSEL2, 3); IO1DIR |= _BV(ZOOM_PIN); IO1CLR = _BV(ZOOM_PIN); } /* Output */

/* 4Hz */
#define dc_periodic() { if (dc_timer) { dc_timer--; } else { IO0CLR = _BV(SHUTTER_PIN); IO1CLR = _BV(ZOOM_PIN); } }

#endif /* ! SITL */

#endif
