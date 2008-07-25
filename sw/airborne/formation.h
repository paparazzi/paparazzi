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

enum slot_status {UNSET, ACTIVE, IDLE, LOST};

struct slot_ {
    enum slot_status status;
    float east;
    float north;
    float alt;
};

extern struct slot_ formation[NB_ACS];

extern int formation_init(void);

extern int add_slot(uint8_t _id, float slot_e, float slot_n, float slot_a);

#define UpdateSlot(_id, _se, _sn, _sa) { \
  formation[the_acs_id[_id]].east = _se; \
  formation[the_acs_id[_id]].north = _sn; \
  formation[the_acs_id[_id]].alt = _sa; \
}

#define UpdateFormationStatus(_id,_status) { formation[the_acs_id[_id]].status = _status; }

extern int start_formation(void);

extern int stop_formation(void);

extern int formation_flight(void);

extern void formation_pre_call(void);

#endif /* FORMATION_H */
