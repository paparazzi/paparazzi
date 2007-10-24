#ifndef LIGHT_H
#define LIGHT_H

#ifdef USE_LIGHT

#include <inttypes.h>

#ifdef LIGHT_PIN_1
#error "LIGHT_PIN_1 is now LIGHT_LED_1"
#endif

/** 0 =< <= 61 . 0 == off, 61 == always on */
extern uint8_t light_duration_1;
extern uint8_t light_duration_2;

#ifdef LIGHT_LED_1
#define LightPeriodicTask1(_1Hz) { \
  if (_1Hz < light_duration_1) { \
    LED_OFF(LIGHT_LED_1) \
  } else \
    LED_ON(LIGHT_LED_1) \
}
#else /* LIGHT_LED_1 */
#define LightPeriodicTask1(_1Hz) {}
#endif /* LIGHT_LED_1 */


#ifdef LIGHT_LED_2
#define LightPeriodicTask2(_1Hz) { \
  if (_1Hz < light_duration_2) { \
    LED_OFF(LIGHT_LED_2) \
  } else \
    LED_ON(LIGHT_LED_2) \
}
#else /* LIGHT_LED_1 */
#define LightPeriodicTask2(_1Hz) {}
#endif /* LIGHT_LED_1 */


#define LightInit() { \
  light_duration_1 = 0; \
  light_duration_2 = 0; \
}


/* Called at 60Hz
   Argument is a counter which goes from 0 to 59 */
#define LightPeriodicTask(_1Hz) { \
  LightPeriodicTask1(_1Hz); \
  LightPeriodicTask2(_1Hz); \
}

#endif // USE_LIGHT

#endif // LIGHT_H
