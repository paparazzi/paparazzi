#include <stdio.h>

#include "booz_geometry_mixed.h"
#define B2_GUIDANCE_V_C
#include "booz2_guidance_v_ref.h"

#define NB_STEP 10000
#define DT (1./B2_GV_FREQ)

double z_sp, z_ref, zd_ref, zdd_ref;

void set_ref(double z, double zd, double zdd) {
  z_ref = z;
  zd_ref = zd;
  zdd_ref = zdd;
}

void update_ref(void) {

  z_ref  += (zd_ref  * DT);
  zd_ref += (zdd_ref * DT);
  zdd_ref = -2.*B2_GV_ZETA*B2_GV_OMEGA*zd_ref - B2_GV_OMEGA*B2_GV_OMEGA*(z_ref - z_sp);

  Bound(zdd_ref, B2_GV_MIN_ZDD_F, B2_GV_MAX_ZDD_F);

  if (zd_ref <= B2_GV_MIN_ZD_F) {
    zd_ref = B2_GV_MIN_ZD_F;
    if (zdd_ref < 0)
      zdd_ref = 0;
  }
  else if (zd_ref >= B2_GV_MAX_ZD_F) {
    zd_ref = B2_GV_MAX_ZD_F;
    if (zdd_ref > 0)
      zdd_ref = 0;
  }

}

void print_ref(int i) {
  double i2f_z   = BOOZ_FLOAT_OF_INT( b2_gv_z_ref,   B2_GV_Z_REF_FRAC);
  double i2f_zd  = BOOZ_FLOAT_OF_INT( b2_gv_zd_ref,  B2_GV_ZD_REF_FRAC);
  double i2f_zdd = BOOZ_FLOAT_OF_INT( b2_gv_zdd_ref, B2_GV_ZDD_REF_FRAC);
  printf("%f %f %f %f %f %f %f %f\n", 
	 (double)i*DT, z_sp, 
	 z_ref, zd_ref, zdd_ref,
	 i2f_z, i2f_zd, i2f_zdd );
}

int32_t get_sp (int i) {
  //  return BOOZ_INT_OF_FLOAT(i>512 ? -50.0 : 0, IPOS_FRAC);
  return BOOZ_INT_OF_FLOAT((i>512&&i<3072) ? -5.0 : 0, IPOS_FRAC);
}



int main(int argc, char** argv) {
  b2_gv_set_ref(0, 0, 0);
  set_ref(0., 0., 0.);
  int i = 0;
  while (i<NB_STEP) {
    int32_t sp_i = get_sp(i);
    b2_gv_update_ref(sp_i);
    z_sp = BOOZ_FLOAT_OF_INT(sp_i, IPOS_FRAC);
    update_ref();
    print_ref(i);
    i++;
  }

  
  return 0;
}


