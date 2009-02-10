#ifndef BOOZ2_MAIN_H
#define BOOZ2_MAIN_H

#ifdef SITL
#define STATIC_INLINE extern
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void booz2_main_init( void );
STATIC_INLINE void booz2_main_periodic( void );
STATIC_INLINE void booz2_main_event( void );

#endif /* BOOZ2_MAIN_H */
