/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <stdio.h>

#include "booz_geometry_mixed.h"
#define GUIDANCE_V_C
#include "firmwares/rotorcraft/guidance/guidance_v_ref.h"

#define NB_STEP 10000
#define DT (1./GV_FREQ)

double z_sp, zd_sp, z_ref, zd_ref, zdd_ref;

void set_ref(double z, double zd, double zdd) {
  z_ref = z;
  zd_ref = zd;
  zdd_ref = zdd;
}

void update_ref_from_z(void) {

  z_ref  += (zd_ref  * DT);
  zd_ref += (zdd_ref * DT);
  zdd_ref = -2.*GV_ZETA*GV_OMEGA*zd_ref - GV_OMEGA*GV_OMEGA*(z_ref - z_sp);

  Bound(zdd_ref, GV_MIN_ZDD_F, GV_MAX_ZDD_F);

  if (zd_ref <= GV_MIN_ZD_F) {
    zd_ref = GV_MIN_ZD_F;
    if (zdd_ref < 0)
      zdd_ref = 0;
  }
  else if (zd_ref >= GV_MAX_ZD_F) {
    zd_ref = GV_MAX_ZD_F;
    if (zdd_ref > 0)
      zdd_ref = 0;
  }

}

void update_ref_from_zd(void) {

  z_ref  += (zd_ref  * DT);
  zd_ref += (zdd_ref * DT);
  zdd_ref = -1/GV_REF_THAU_F*(zd_ref - zd_sp);

  Bound(zdd_ref, GV_MIN_ZDD_F, GV_MAX_ZDD_F);

  if (zd_ref <= GV_MIN_ZD_F) {
    zd_ref = GV_MIN_ZD_F;
    if (zdd_ref < 0)
      zdd_ref = 0;
  }
  else if (zd_ref >= GV_MAX_ZD_F) {
    zd_ref = GV_MAX_ZD_F;
    if (zdd_ref > 0)
      zdd_ref = 0;
  }

}

void print_ref(int i) {
  double i2f_z   = BOOZ_FLOAT_OF_INT( gv_z_ref,   GV_Z_REF_FRAC);
  double i2f_zd  = BOOZ_FLOAT_OF_INT( gv_zd_ref,  GV_ZD_REF_FRAC);
  double i2f_zdd = BOOZ_FLOAT_OF_INT( gv_zdd_ref, GV_ZDD_REF_FRAC);
  printf("%f %f %f %f %f %f %f %f %f\n",
     (double)i*DT, z_sp, zd_sp,
     z_ref, zd_ref, zdd_ref,
     i2f_z, i2f_zd, i2f_zdd );
}

int32_t get_sp (int i) {
  //  return BOOZ_INT_OF_FLOAT(i>512 ? -50.0 : 0, IPOS_FRAC);
  return BOOZ_INT_OF_FLOAT((i>512&&i<3072) ? -1.0 : 0, ISPEED_RES);
}



int main(int argc, char** argv) {
  gv_set_ref(0, 0, 0);
  set_ref(0., 0., 0.);
  int i = 0;
  while (i<NB_STEP) {
    int32_t sp_i = get_sp(i);
    //    gv_update_ref_from_z_sp(sp_i);
    gv_update_ref_from_zd_sp(sp_i);
    //    z_sp = BOOZ_FLOAT_OF_INT(sp_i, IPOS_FRAC);
    //    update_ref_from_z();
    zd_sp = BOOZ_FLOAT_OF_INT(sp_i, ISPEED_RES);
    update_ref_from_zd();
    print_ref(i);
    i++;
  }


  return 0;
}
