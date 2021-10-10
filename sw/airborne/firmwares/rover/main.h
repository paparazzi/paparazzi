// Empty main loop functions... rovers only use main_ap...

#ifndef MAIN_H
#define MAIN_H

#ifdef SITL
#define STATIC_INLINE extern
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void main_init(void);
STATIC_INLINE void main_event(void);
STATIC_INLINE void handle_periodic_tasks(void);

STATIC_INLINE void main_periodic(void);
STATIC_INLINE void telemetry_periodic(void);
STATIC_INLINE void failsafe_check(void);


#endif /* MAIN_H */
