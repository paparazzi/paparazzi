#ifndef RUST_ALLOC_CHIBIOS_H
#define RUST_ALLOC_CHIBIOS_H

#if !USE_CHIBIOS_RTOS
#error "This module requires ChibiOS"
#endif
extern void rust_function(void);

#endif /* RUST_ALLOC_CHIBIOS_H */
