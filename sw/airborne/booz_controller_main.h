#ifndef BOOZ_CONTROLLER_MAIN_H
#define BOOZ_CONTROLLER_MAIN_H

#ifdef SITL
#define STATIC_INLINE
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void booz_controller_main_init( void );
STATIC_INLINE void booz_controller_main_periodic_task( void );
STATIC_INLINE void booz_controller_main_event_task( void );

#endif /* BOOZ_CONTROLLER_MAIN_H */
