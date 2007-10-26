#ifndef MAIN_BOOZ_H
#define MAIN_BOOZ_H

#ifdef SITL
#define STATIC_INLINE
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void booz_main_init( void );
STATIC_INLINE void booz_main_periodic_task( void );
STATIC_INLINE void booz_main_event_task( void );

#endif /* MAIN_BOOZ_H */
