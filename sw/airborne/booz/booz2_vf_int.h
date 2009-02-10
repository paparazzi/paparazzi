#ifndef BOOZ2_VF_INT_H
#define BOOZ2_VF_INT_H

#include "std.h"
#include "booz_geometry_int.h"

extern void booz2_vfi_init( int32_t z0, int32_t zd0, int32_t bias0 );
extern void booz2_vfi_propagate( int32_t accel_reading );

/* z_meas : altitude measurement in meter       */
/* Q23.8 : accuracy 0.004m range 8388km         */
extern void booz2_vfi_update( int32_t z_meas );
#define B2_VFI_MEAS_Z_FRAC IPOS_FRAC

/* propagate frequency : 512 Hz */
#define B2_VFI_F_UPDATE_FRAC 9
#define B2_VFI_F_UPDATE   (1<<B2_VFI_F_UPDATE_RES)

/* vertical acceleration in m/s^2                */
/* Q21.10 : accuracy 0.001m/s^2, range 2097km/s2 */
extern int32_t b2_vfi_zdd;
#define B2_VFI_ZDD_FRAC IACCEL_RES

/* vertical accelerometer bias in m/s^2          */
/* Q21.10 : accuracy 0.001m/s^2, range 2097km/s2 */
extern int32_t b2_vfi_abias;
#define B2_VFI_BIAS_FRAC IACCEL_RES

/* vertical speed in m/s                         */
/* Q12.19 : accuracy 0.000002 , range 4096m/s2   */
extern int32_t b2_vfi_zd;
#define B2_VFI_ZD_FRAC (B2_VFI_ZDD_FRAC + B2_VFI_F_UPDATE_FRAC)

/* altitude in m                                 */
/* Q35.28 : accuracy 3.7e-9 , range 3.4e10m      */
extern int64_t b2_vfi_z;
#define B2_VFI_Z_FRAC   (B2_VFI_ZD_FRAC + B2_VFI_F_UPDATE_FRAC)

/* Kalman filter state                           */
#define B2_VFI_S_Z    0
#define B2_VFI_S_ZD   1
#define B2_VFI_S_AB   2
#define B2_VFI_S_SIZE 3
/* Kalman filter covariance                      */
/* Q3.28                                         */
extern int32_t b2_vfi_P[B2_VFI_S_SIZE][B2_VFI_S_SIZE];
#define B2_VFI_P_FRAC  28




#endif /* BOOZ2_VF_INT_H */
