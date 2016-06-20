/** \file potential.h
 *  \brief flying with potential field to avoid collision
 *
 */


#ifndef POTENTIAL_H
#define POTENTIAL_H

#include "firmwares/fixedwing/nav.h"
#include "modules/multi/traffic_info.h"

struct force_ {
  float east;
  float north;
  float alt;
  float speed;
  float climb;
};

extern struct force_ potential_force;

extern float force_pos_gain;
extern float force_speed_gain;
extern float force_climb_gain;

extern void potential_init(void);

extern int potential_task(void);

#endif /* POTENTIAL_H */
