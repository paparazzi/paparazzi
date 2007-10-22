#ifndef BOOZ_FM_UTILS_H
#define BOOZ_FM_UTILS_H

#include <matrix.h>

typedef void (*ode_fun)(VEC* x, VEC* u, VEC* xdot);

extern void rk4(ode_fun f, VEC* x, VEC* u, double dt);

#endif /* BOOZ_FM_UTILS_H */
