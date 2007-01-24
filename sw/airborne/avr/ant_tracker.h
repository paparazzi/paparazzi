#ifndef ANT_TRACKER_H
#define ANT_TRACKER_H

#include "std.h"

#define ANT_TRACK_MANUAL 0
#define ANT_TRACK_AUTO   1

extern uint8_t ant_track_mode;
extern float ant_track_azim;
extern float ant_track_elev;
extern uint8_t ant_track_id;

#endif /* ANT_TRACKER_H */
