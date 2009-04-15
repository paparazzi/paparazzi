#ifndef RDYB_GPIO_H
#define RDYB_GPIO_H

#include <inttypes.h>

extern void gpio_init( void );
extern void gpio_toggle( uint8_t pin );
extern void gpio_set(uint8_t pin, uint8_t on);

#endif /* RDYB_GPIO_H */
