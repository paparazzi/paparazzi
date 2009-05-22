#ifndef CSC_MAIN_H
#define CSC_MAIN_H

#ifdef SITL
#define STATIC_INLINE extern
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void csc_main_init( void );
STATIC_INLINE void csc_main_periodic( void );
STATIC_INLINE void csc_main_event( void );

#endif /* CSC_MAIN_H */
