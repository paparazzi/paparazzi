/** \file formation.h
 *  \brief Formation flight library
 *
 */


#ifndef FORMATION_H
#define FORMATION_H

#include "nav.h"
#include "traffic_info.h"

#define FORM_MODE_GLOBAL 0
#define FORM_MODE_COURSE 1

extern float coef_form_alt,coef_form_pos,coef_form_speed,coef_form_course;
extern float form_prox;
extern int form_mode;
extern uint8_t leader_id;

enum slot_status {UNSET, ACTIVE, IDLE};

struct slot_ {
    enum slot_status status;
    float east;
    float north;
    float alt;
};

struct slot_ formation[NB_ACS];

int formation_init(void);

int add_slot(uint8_t _id, float slot_e, float slot_n, float slot_a);

#define UpdateSlot(_id, _se, _sn, _sa) { \
  formation[_id].east = _se; \
  formation[_id].north = _sn; \
  formation[_id].alt = _sa; \
}

int start_formation(void);

int stop_formation(void);

int formation_flight(void);

#ifdef FORMATION 
#define FormationFlight() formation_flight()
#else
#define FormationFlight() {}
#endif

#endif /* FORMATION_H */
