#ifndef BOOZ_FLIGHT_MODEL_PARAMS_H
#define BOOZ_FLIGHT_MODEL_PARAMS_H

/* body drag coefficient                           */
#define C_d_body .0075
/* propeller thrust aerodynamic coefficient        */
#define C_t 0.297
/* propeller moment aerodynamic coefficient        */
#define C_q 0.0276
/* propeller radius in m                           */
#define PROP_RADIUS 0.125
/* propeller area in m2                            */
#define PROP_AREA 0.005
/* air density in kg/m3                            */
#define RHO 1.225
/* gravity in m/s2                                 */
#define G 9.81
/* mass in kg                                      */
#define MASS 0.724
/* inertia on x axis in kg * m2                    */
#define Ix .007
/* inertia on y axis in kg * m2                    */
#define Iy .0073
/* inertia on z axis in kg * m2                    */
#define Iz .0137
/* lenght between centers of vehicle and prop in m */
#define L  0.25

/* motors parameters

  from http://cherokee.stanford.edu/~starmac/docs/DynamicsSummary

  omega_dot = -1/thau * omega - Kq * omega^2 + Kv/thau * V

*/
#define BAT_VOLTAGE 11.

#define THAU     0.05
#define Kq       0.12
#define Kv     304.



#endif /* BOOZ_FLIGHT_MODEL_PARAMS_H */
