#include "booz_flight_model_utils.h"

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
  v_mltadd(x,temp,dt/6.0,x);     /* x = x+(h/6)*temp */
  
}
