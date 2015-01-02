


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"


int main(int argc, char **argv)
{

  struct FloatEulers ea = { RadOfDeg(45.), RadOfDeg(30.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("ea", ea);

  struct FloatEulers eb = { RadOfDeg(42.), RadOfDeg(45.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("eb", eb);

  struct FloatRMat ra;
  float_rmat_of_eulers(&ra, &ea);
  struct FloatRMat rb;
  float_rmat_of_eulers(&rb, &eb);
  struct FloatRMat rc;
  float_rmat_comp(&rc, &ra, &rb);

  DISPLAY_FLOAT_RMAT_AS_EULERS_DEG("rc ", rc);

  struct FloatQuat qa;
  float_quat_of_eulers(&qa, &ea);
  struct FloatQuat qb;
  float_quat_of_eulers(&qb, &eb);
  struct FloatQuat qc;
  float_quat_comp(&qc, &qa, &qb);


  DISPLAY_FLOAT_QUAT_AS_EULERS_DEG("qc ", qc);

}
