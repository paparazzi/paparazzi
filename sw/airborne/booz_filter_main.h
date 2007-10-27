#ifndef BOOZ_FILTER_MAIN_H
#define BOOZ_FILTER_MAIN_H

#ifdef SITL
#define STATIC_INLINE
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void booz_filter_main_init( void );
STATIC_INLINE void booz_filter_main_periodic_task( void );
STATIC_INLINE void booz_filter_main_event_task( void );

#endif /* BOOZ_FILTER_MAIN_H */
