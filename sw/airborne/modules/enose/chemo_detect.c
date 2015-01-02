#include "chemo_detect.h"
#include "enose.h"

#define DETECT_PERIOD 8 /* *4Hz */
#define THRESHOLD 150

uint16_t chemo_sensor;

void chemo_init(void)
{
  chemo_sensor = 0;
}

void chemo_periodic(void)
{
#ifdef ENOSE
  static uint16_t vals[ENOSE_NB_SENSOR][DETECT_PERIOD];
  static int idx;

  /* Detection on the first sensor */
  int dval = enose_val[0] - vals[0][idx];
  if (dval < -THRESHOLD) {
    chemo_sensor = -dval;
  } else {
    chemo_sensor = 0;
  }

  int i;
  for (i = 0; i < ENOSE_NB_SENSOR; i++) {
    vals[i][idx] = enose_val[i];
  }

  idx++;
  if (idx > DETECT_PERIOD) {
    idx = 0;
  }
#endif /* ENOSE */
}
