#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#define SYS_TICS_OF_USEC(x) (x)
#define SIGNED_SYS_TICS_OF_USEC(x) (x)

static inline void sys_time_init( void ) {}

#define sys_time_periodic() TRUE

#endif /* SYS_TIME_HW_H */
