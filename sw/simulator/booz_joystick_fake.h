#ifndef BOOZ_JOYSTICK_FAKE_H
#define BOOZ_JOYSTICK_FAKE_H

#define BREAK_MTT() {				\
    int foo = sim_time/10;			\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.0 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.4 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_PITCH]    = 1500 - 0.0 * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.4 * (2050-950);		\
      break;								\
    }									\
}

#define WALK_OVAL() {							\
    ppm_pulses[RADIO_ROLL]       = 1500 + 0. * (2050-950);		\
    int foo = sim_time/10;						\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0. * (2050-950);		\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0.2 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_PITCH]    = 1500 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.2 * (2050-950);		\
      break;								\
    }									\
}

#define HOVER() { \
    ppm_pulses[RADIO_THROTTLE] = 1223 + 0.6 * (2050-1223);		\
    ppm_pulses[RADIO_MODE]     = 1500 - 0.5 * (2050-950);		\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0. * (2050-950);		\
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
  }


#define TOUPIE() {							\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0. * (2050-950);		\
    int foo = sim_time/20;						\
    switch(foo%4) {							\
    case 0:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0.2 * (2050-950);		\
      break;								\
    case 2:								\
      ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
      break;								\
    case 3:								\
      ppm_pulses[RADIO_YAW]      = 1493 + -0.2 * (2050-950);		\
      break;								\
    }									\
  }


#define CIRCLE() {				                        \
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
    double alpha = 0.01 * sim_time;					\
    ppm_pulses[RADIO_PITCH]    = 1498 + cos(alpha) * 250.;		\
    ppm_pulses[RADIO_ROLL]     = 1500 + sin(alpha) * 250.;		\
  }

#define ATTITUDE_ROLL_STEPS() {						\
    ppm_pulses[RADIO_THROTTLE] = 1223 + 0.66 * (2050-1223);		\
    ppm_pulses[RADIO_MODE]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0. * (2050-950);		\
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
    int foo = sim_time/10;						\
    switch(foo%2) {							\
    case 0:								\
      ppm_pulses[RADIO_ROLL]      = 1500 + 0.2 * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_ROLL]      = 1500 - 0.2 * (2050-950);		\
      break;								\
    }									\
  }

#define ATTITUDE_PITCH_STEPS() {						\
    ppm_pulses[RADIO_THROTTLE] = 1223 + 0.66 * (2050-1223);		\
    ppm_pulses[RADIO_MODE]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_YAW]      = 1493 + 0. * (2050-950);		\
    int foo = sim_time/10;						\
    switch(foo%2) {							\
    case 0:								\
      ppm_pulses[RADIO_PITCH]      = 1498 + 0.2 * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_PITCH]      = 1498 - 0.2 * (2050-950);		\
      break;								\
    }									\
  }

#define ATTITUDE_YAW_STEPS() {						\
    ppm_pulses[RADIO_THROTTLE] = 1223 + 0.66 * (2050-1223);		\
    ppm_pulses[RADIO_MODE]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0. * (2050-950);		\
    ppm_pulses[RADIO_PITCH]    = 1500 + 0. * (2050-950);		\
    int foo = sim_time/10;						\
    switch(foo%2) {							\
    case 0:								\
      ppm_pulses[RADIO_YAW]    = 1493 + 0.2 * (2050-950);		\
      break;								\
    case 1:								\
      ppm_pulses[RADIO_YAW]  = 1493 - 0.2 * (2050-950);			\
      break;								\
    }									\
  }



#endif /* BOOZ_JOYSTICK_FAKE_H */
