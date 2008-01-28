#ifndef TL_NAV_H
#define TL_NAV_H

#include "common_nav.h"

void tl_nav_init(void);

/** To be called at 4Hz */
void tl_nav_periodic_task(void);


#endif // TL_NAV_H
