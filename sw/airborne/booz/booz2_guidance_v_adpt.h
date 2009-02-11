#ifndef BOOZ2_GUIDANCE_V_ADPT
#define BOOZ2_GUIDANCE_V_ADPT

extern int32_t b2_gv_adapt_X;
extern int32_t b2_gv_adapt_P;

#ifdef B2_GUIDANCE_V_C

/* */
int32_t b2_gv_adapt_X;
#define B2_GV_ADAPT_X_FRAC 18

/* Q5.26 */
int32_t b2_gv_adapt_P;
#define B2_GV_ADAPT_P_FRAC 18

#define B2_GV_ADAPT_X0_F 0.15
#define B2_GV_ADAPT_X0 BOOZ_INT_OF_FLOAT(B2_GV_ADAPT_X0_F, B2_GV_ADAPT_X_FRAC)
#define B2_GV_ADAPT_P0_F 0.5
#define B2_GV_ADAPT_P0 BOOZ_INT_OF_FLOAT(B2_GV_ADAPT_P0_F, B2_GV_ADAPT_P_FRAC)

#define B2_GV_ADAPT_SYS_NOISE_F 0.00005
#define B2_GV_ADAPT_SYS_NOISE  BOOZ_INT_OF_FLOAT(B2_GV_ADAPT_SYS_NOISE_F, B2_GV_ADAPT_P_FRAC)
#define B2_GV_ADAPT_MEAS_NOISE_F 2.0
#define B2_GV_ADAPT_MEAS_NOISE BOOZ_INT_OF_FLOAT(B2_GV_ADAPT_MEAS_NOISE_F, B2_GV_ADAPT_P_FRAC)


static inline void b2_gv_adapt_init(void) {
  b2_gv_adapt_X = B2_GV_ADAPT_X0;
  b2_gv_adapt_P = B2_GV_ADAPT_P0;
}

/*
  zdd_meas : IACCEL_RES ... shall we or shall we not unbias ? :)
  thrust_applied : controller input [2-200]
*/
#define K_FRAC 12
static inline void b2_gv_adapt_run(int32_t zdd_meas, int32_t thrust_applied) {
  if (thrust_applied == 0) return;
  /* propagate covariance */
  b2_gv_adapt_P =  b2_gv_adapt_P + B2_GV_ADAPT_SYS_NOISE;
  int32_t meas = (((int32_t)BOOZ_INT_OF_FLOAT(9.81, IACCEL_RES) - zdd_meas)<<(B2_GV_ADAPT_X_FRAC - IACCEL_RES)) / thrust_applied;
  int32_t residual = meas - b2_gv_adapt_X;
  int32_t E = b2_gv_adapt_P + B2_GV_ADAPT_MEAS_NOISE;
  int32_t K = (b2_gv_adapt_P<<K_FRAC) / E;
  b2_gv_adapt_P = b2_gv_adapt_P - ((K * b2_gv_adapt_P)>>K_FRAC);
  b2_gv_adapt_X = b2_gv_adapt_X + ((K * residual)>>K_FRAC);
}


#endif /* B2_GUIDANCE_V_C */


#endif /* BOOZ2_GUIDANCE_V_ADPT */
