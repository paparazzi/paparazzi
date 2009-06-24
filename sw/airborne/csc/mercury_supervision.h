#ifndef BOOZ2_SUPERVISION_H
#define BOOZ2_SUPERVISION_H

#include "airframe.h"

/* #if defined SUPERVISION_FRONT_ROTOR_CW */
/* #define TRIM_FRONT ( SUPERVISION_TRIM_E-SUPERVISION_TRIM_R) */
/* #define TRIM_RIGHT (-SUPERVISION_TRIM_A+SUPERVISION_TRIM_R) */
/* #define TRIM_BACK  (-SUPERVISION_TRIM_E-SUPERVISION_TRIM_R) */
/* #define TRIM_LEFT  ( SUPERVISION_TRIM_A+SUPERVISION_TRIM_R) */
/* #define SUPERVISION_MIX(_mot_cmd, _da, _de, _dr, _dt) {		\ */
/*     _mot_cmd[PROP_FRONT] = _dt + _de - _dr + TRIM_FRONT;	\ */
/*     _mot_cmd[PROP_RIGHT] = _dt - _da + _dr + TRIM_RIGHT;	\ */
/*     _mot_cmd[PROP_BACK]  = _dt - _de - _dr + TRIM_BACK;	\ */
/*     _mot_cmd[PROP_LEFT]  = _dt + _da + _dr + TRIM_LEFT;	\ */
/*   } */
/* #else */
#define TRIM_FRONT ( SUPERVISION_TRIM_E+SUPERVISION_TRIM_R)
#define TRIM_RIGHT (-SUPERVISION_TRIM_A-SUPERVISION_TRIM_R)
#define TRIM_BACK  (-SUPERVISION_TRIM_E+SUPERVISION_TRIM_R)
#define TRIM_LEFT  ( SUPERVISION_TRIM_A-SUPERVISION_TRIM_R)
#define SUPERVISION_MIX(_mot_cmd, _da, _de, _dr, _dt) {			\
    _mot_cmd[PROP_UPPER_LEFT]  = _dt + _da - _de  + _dr  + TRIM_FRONT; \
    _mot_cmd[PROP_LOWER_RIGHT] = _dt - _da + _de  + _dr  + TRIM_RIGHT;	\
    _mot_cmd[PROP_LOWER_LEFT]  = _dt + _da + _de  - _dr  + TRIM_BACK;	\
    _mot_cmd[PROP_UPPER_RIGHT] = _dt - _da - _de  - _dr  + TRIM_LEFT;	\
  }
//#endif

#define SUPERVISION_FIND_MAX_MOTOR(_mot_cmd, _max_mot) {	\
    _max_mot = (-32767-1); /* INT16_MIN;*/			\
    if (_mot_cmd[PROP_UPPER_LEFT] > _max_mot)			\
      max_mot = _mot_cmd[PROP_UPPER_LEFT];				\
    if (_mot_cmd[PROP_LOWER_RIGHT] > _max_mot)			\
      max_mot = _mot_cmd[PROP_LOWER_RIGHT];				\
    if (_mot_cmd[PROP_LOWER_LEFT] > _max_mot)			\
      max_mot = _mot_cmd[PROP_LOWER_LEFT];				\
    if (_mot_cmd[PROP_UPPER_RIGHT] > _max_mot)			\
      max_mot = _mot_cmd[PROP_UPPER_RIGHT];				\
  }

#define SUPERVISION_FIND_MIN_MOTOR(_mot_cmd, _min_mot) {	\
    _min_mot = (32767); /*INT16_MAX;*/				\
    if (_mot_cmd[PROP_UPPER_LEFT] < _min_mot)			\
      min_mot = _mot_cmd[PROP_UPPER_LEFT];				\
    if (_mot_cmd[PROP_LOWER_RIGHT] < _min_mot)			\
      min_mot = _mot_cmd[PROP_LOWER_RIGHT];				\
    if (_mot_cmd[PROP_LOWER_LEFT] < _min_mot)			\
      min_mot = _mot_cmd[PROP_LOWER_LEFT];				\
    if (_mot_cmd[PROP_UPPER_RIGHT] < _min_mot)			\
      min_mot = _mot_cmd[PROP_UPPER_RIGHT];				\
  }

#define SUPERVISION_OFFSET_MOTORS(_mot_cmd, _offset) {	\
    _mot_cmd[PROP_UPPER_LEFT] += _offset;			\
    _mot_cmd[PROP_LOWER_RIGHT] += _offset;			\
    _mot_cmd[PROP_LOWER_LEFT]  += _offset;			\
    _mot_cmd[PROP_UPPER_RIGHT]  += _offset;			\
  }

#define SUPERVISION_BOUND_MOTORS(_mot_cmd) {				\
    Bound(_mot_cmd[PROP_UPPER_LEFT], SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR); \
    Bound(_mot_cmd[PROP_LOWER_RIGHT], SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR); \
    Bound(_mot_cmd[PROP_LOWER_LEFT] , SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR); \
    Bound(_mot_cmd[PROP_UPPER_RIGHT] , SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR); \
  }


#define BOOZ2_SUPERVISION_RUN(_out, _in,_motors_on) {			\
    if (_motors_on) {							\
      SUPERVISION_MIX(_out, _in[COMMAND_ROLL], _in[COMMAND_PITCH], _in[COMMAND_YAW], _in[COMMAND_THRUST]); \
      pprz_t min_mot;							\
      SUPERVISION_FIND_MIN_MOTOR(_out, min_mot);			\
      if (min_mot < SUPERVISION_MIN_MOTOR) {				\
	pprz_t offset = -(min_mot - SUPERVISION_MIN_MOTOR);		\
	SUPERVISION_OFFSET_MOTORS(_out, offset) ;			\
      }									\
      pprz_t max_mot;							\
      SUPERVISION_FIND_MAX_MOTOR(_out, max_mot);			\
      if (max_mot > SUPERVISION_MAX_MOTOR) {				\
	pprz_t offset = -(max_mot - SUPERVISION_MAX_MOTOR);		\
	SUPERVISION_OFFSET_MOTORS(_out, offset) ;			\
      }									\
      SUPERVISION_BOUND_MOTORS(_out);					\
    }									\
    else {								\
      _out[PROP_UPPER_LEFT] = 0;						\
      _out[PROP_LOWER_RIGHT] = 0;						\
      _out[PROP_LOWER_LEFT]  = 0;						\
      _out[PROP_UPPER_RIGHT]  = 0;						\
    }									\
  }




#endif /* BOOZ2_SUPERVISION_H */
