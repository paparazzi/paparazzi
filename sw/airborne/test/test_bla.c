


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"


int main(int argc, char** argv) {

  struct FloatEulers ea = { RadOfDeg(45.), RadOfDeg(30.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("ea", ea);

  struct FloatEulers eb = { RadOfDeg(42.), RadOfDeg(45.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("eb", eb);

  struct FloatRMat ra;
  FLOAT_RMAT_OF_EULERS(ra, ea);
  struct FloatRMat rb;
  FLOAT_RMAT_OF_EULERS(rb, eb);
  struct FloatRMat rc;
  FLOAT_RMAT_COMP(rc, ra, rb);

  DISPLAY_FLOAT_RMAT_AS_EULERS_DEG("rc ", rc);

  struct FloatQuat qa;
  FLOAT_QUAT_OF_EULERS(qa, ea);
  struct FloatQuat qb;
  FLOAT_QUAT_OF_EULERS(qb, eb);
  struct FloatQuat qc;
  FLOAT_QUAT_COMP(qc, qa, qb);


  DISPLAY_FLOAT_QUAT_AS_EULERS_DEG("qc ", qc);

}
