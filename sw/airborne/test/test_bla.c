


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"


static float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = RadOfDeg(lat_now);
	const double lat_next_rad = RadOfDeg(lat_next);

	const double cos_lat_next = cos(lat_next_rad);
	const double d_lon = RadOfDeg(lon_next - lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	const float y = (sin(d_lon) * cos_lat_next);
	const float x = (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon));

  float a = atan2f(y, x);
  FLOAT_ANGLE_NORMALIZE(a);

	return a;
}

static void
get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e)
{
  double CONSTANTS_RADIUS_OF_EARTH = 6371000.0;

	const double lat_now_rad = RadOfDeg(lat_now);
	const double lat_next_rad = RadOfDeg(lat_next);
	const double d_lon = RadOfDeg(lon_next) - RadOfDeg(lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = (CONSTANTS_RADIUS_OF_EARTH * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon)));
	*v_e = (CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad));
}


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

  float v_n, v_e;
  get_vector_to_next_waypoint(0.+1.e-7, 0.0+1.e-7, 0., 0.0, &v_n, &v_e);
  printf("new func: %f, %f\n", v_e, v_n);

}
