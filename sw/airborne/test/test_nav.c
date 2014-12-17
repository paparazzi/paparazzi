#include <assert.h>
#include "firmwares/fixedwing/nav.h"

#define FloatEqual_3(f1, f2) (fabs(f1 - f2) < 1e-3)

int main(void)
{
  float angle = 370.;
  NormCourse(angle);
  assert(FloatEqual_3(angle, 10.));

  /* No loop for large numbers */
  angle = 360. * 257 + 10.;
  NormCourse(angle);
  assert(angle > 360);

  assert(CloseDegAngles(180, 175 + 10 * 360));
  assert(CloseDegAngles(175 + 10 * 360, 180));
  assert(CloseDegAngles(355, 4 + 5 * 360));
  assert(CloseDegAngles(4 + 5 * 360, 355));

  return 0;
}
