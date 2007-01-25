#ifndef ANT_TRACKER_H
#define ANT_TRACKER_H

#include "std.h"

#define ANT_TRACK_MANUAL 0
#define ANT_TRACK_AUTO   1

extern uint8_t ant_track_mode;
extern float ant_track_azim;
extern float ant_track_elev;
extern uint8_t ant_track_id;

#include "led.h"

#define ant_tracker_SetId(i) { ant_track_id = i; }
#define ant_tracker_SetMode(i) \
  {			       \
    ant_track_mode = i;	       \
    if(ant_track_mode)	       \
      LED_ON(2);	       \
    else		       \
      LED_OFF(2);	       \
  }


extern void ant_tracker_init( void );
extern void ant_tracker_periodic( void );
#endif /* ANT_TRACKER_H */
