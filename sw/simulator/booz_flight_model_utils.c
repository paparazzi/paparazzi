#include "booz_flight_model_utils.h"

#include <math.h>
#include "6dof.h"

void rk4(ode_fun f, VEC* x, VEC* u, double dt) {

  static VEC *v1=VNULL, *v2=VNULL, *v3=VNULL, *v4=VNULL;
  static VEC *temp=VNULL;

  v1   = v_resize(v1,x->dim);
  v2   = v_resize(v2,x->dim);
  v3   = v_resize(v3,x->dim);
  v4   = v_resize(v4,x->dim);
  temp = v_resize(temp,x->dim);

  //  MEM_STAT_REG(v1,TYPE_VEC);
  //  MEM_STAT_REG(v2,TYPE_VEC);
  //  MEM_STAT_REG(v3,TYPE_VEC);
  //  MEM_STAT_REG(v4,TYPE_VEC);
  //  MEM_STAT_REG(temp,TYPE_VEC);

  f(x, u, v1);
  v_mltadd(x,v1,0.5*dt,temp);    /* temp = x + .5*dt*v1   */
  f(temp, u, v2);
  v_mltadd(x,v2,0.5*dt,temp);    /* temp = x + .5*dt*v2   */
  f(temp,u, v3);
  v_mltadd(x,v3,dt,temp);        /* temp = x + dt*v3      */
  f(temp, u, v4);

  /* now add: v1+2*v2+2*v3+v4 */
  v_copy(v1,temp);              /* temp = v1              */
  v_mltadd(temp,v2,2.0,temp);   /* temp = v1+2*v2         */
  v_mltadd(temp,v3,2.0,temp);   /* temp = v1+2*v2+2*v3    */
  v_add(temp,v4,temp);          /* temp = v1+2*v2+2*v3+v4 */

  /* adjust x */
  v_mltadd(x,temp,dt/6.0,x);    /* x = x+(h/6) * temp     */
  
}

MAT* dcm_of_eulers (VEC* eulers, MAT* dcm ) {
  
  dcm = m_resize(dcm, 3,3);

  double sinPHI   = sin(eulers->ve[EULER_PHI]);
  double cosPHI   = cos(eulers->ve[EULER_PHI]);
  double sinTHETA = sin(eulers->ve[EULER_THETA]);
  double cosTHETA = cos(eulers->ve[EULER_THETA]);
  double sinPSI   = sin(eulers->ve[EULER_PSI]);
  double cosPSI   = cos(eulers->ve[EULER_PSI]);

  dcm->me[0][0] = cosTHETA * cosPSI;
  dcm->me[0][1] = cosTHETA * sinPSI;
  dcm->me[0][2] = -sinTHETA;
  dcm->me[1][0] = sinPHI * sinTHETA * cosPSI - cosPHI * sinPSI;
  dcm->me[1][1] = sinPHI * sinTHETA * sinPSI + cosPHI * cosPSI;
  dcm->me[1][2] = sinPHI * cosTHETA;
  dcm->me[2][0] = cosPHI * sinTHETA * cosPSI + sinPHI * sinPSI;
  dcm->me[2][1] = cosPHI * sinTHETA * sinPSI - sinPHI * cosPSI;
  dcm->me[2][2] = cosPHI * cosTHETA;

  return dcm;
}

VEC* out_prod( VEC* a, VEC* b, VEC* out) {
  if ( a->dim != 3 || b->dim != 3 )
    error(E_SIZES,"out_prod");
  if ( out==(VEC *)NULL || out->dim != a->dim )
    out = v_resize(out,a->dim);
  out->ve[0] = a->ve[1]*b->ve[2] - a->ve[2]*b->ve[1];
  out->ve[1] = a->ve[2]*b->ve[0] - a->ve[0]*b->ve[2];
  out->ve[2] = a->ve[0]*b->ve[1] - a->ve[1]*b->ve[0];
  return out;
}
