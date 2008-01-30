#ifndef TL_NAV_H
#define TL_NAV_H

#include "common_nav.h"

extern float tl_nav_goto_h_pgain;
extern float tl_nav_goto_h_dgain;
extern float tl_nav_goto_x_sp;
extern float tl_nav_goto_y_sp;
extern float y_unit_err_body, x_unit_err_body;

void tl_nav_init(void);

/** To be called at 4Hz */
void tl_nav_periodic_task(void);

#endif // TL_NAV_H
