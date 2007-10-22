#ifndef BOOZ_FLIGHT_MODEL_PARAMS_H
#define BOOZ_FLIGHT_MODEL_PARAMS_H

/* drag coefficient of the body                    */
#define C_d_body .2
/* thrust aerodynamic coefficient                  */
#define C_t 1.
/* moment aerodynamic coefficient                  */
#define C_q 0.1
/* propeller radius in m                           */
#define PROP_RADIUS 0.125
/* propeller area in m2                            */
#define PROP_AREA (M_PI * PROP_RADIUS * PROP_RADIUS)
/* air density in kg/m3                            */
#define RHO 1.225
/* gravity in m/s2                                 */
#define G 9.81
/* mass in kg                                      */
#define MASS 0.5                 
/* inertia on x axis in m2                         */
#define Ix .5
/* inertia on y axis in m2                         */
#define Iy .5
/* inertia on z axis in m2                         */
#define Iz 1.
/* lenght between centers of vehicle and prop in m */
#define L  0.25
/* height between cg and prop plane in m           */
#define H 0.04


/* motors parameters 

  from http://cherokee.stanford.edu/~starmac/docs/DynamicsSummary

  omega_dot = -1/thau * omega - Kq * omega^2 + Kv/thau * V

*/
#define BAT_VOLTAGE 11.

#define THAU   100. 
#define Kq       0.01
#define Kv   10000.



#endif /* BOOZ_FLIGHT_MODEL_PARAMS_H */
