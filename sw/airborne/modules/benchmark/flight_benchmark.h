#ifndef FLIGHTBENCHMARK_H
#define FLIGHTBENCHMARK_H

#include <inttypes.h>

void flight_benchmark_init(void);
void flight_benchmark_periodic(void);
void flight_benchmark_reset(void);

extern float ToleranceAispeed;
extern float ToleranceAltitude;
extern float TolerancePosition;
extern bool benchm_reset;
extern bool benchm_go;

#endif /* FLIGHTBENCHMARK_H */
