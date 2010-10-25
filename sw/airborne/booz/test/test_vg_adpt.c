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

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>

#include "std.h"
#include "booz_geometry_mixed.h"
#define GUIDANCE_V_C
#include "firmwares/rotorcraft/guidance/guidance_v_adpt.h"



#define NB_STEP 20000
int n_dat;

double time[NB_STEP];

int32_t z_sp[NB_STEP];
int32_t zd_sp[NB_STEP];
int32_t est_z[NB_STEP];
int32_t est_zd[NB_STEP];
int32_t est_zdd[NB_STEP];
int32_t ref_z[NB_STEP];
int32_t ref_zd[NB_STEP];
int32_t ref_zdd[NB_STEP];
int32_t adp_inv_m[NB_STEP];
int32_t adp_cov[NB_STEP];
int32_t sum_err[NB_STEP];
int32_t ff_cmd[NB_STEP];
int32_t fb_cmd[NB_STEP];
int32_t delta_t[NB_STEP];



int32_t ifX[NB_STEP];
int32_t ifP[NB_STEP];


double ffX[NB_STEP];
double ffm[NB_STEP];
double ffP[NB_STEP];

static void float_filter_init(void) {
  ffX[0] = GV_ADAPT_X0_F;
  ffP[0] = GV_ADAPT_P0_F;
}

static void float_filter_run( int i) {

  int prev = i>0 ? i-1 : i;
  ffX[i] = ffX[prev];
  ffP[i] = ffP[prev];
  if (delta_t[prev] == 0) return;
  ffP[i] = ffP[i] + GV_ADAPT_SYS_NOISE_F;
  ffm[i] = (9.81 - (double)est_zdd[i]/(double)(1<<10)) / (double)delta_t[prev];
  double residual = ffm[i] - ffX[i];
  double E = ffP[i] + GV_ADAPT_MEAS_NOISE_F;
  double K = ffP[i] / E;
  ffP[i] = ffP[i] - K * ffP[i];
  ffX[i] = ffX[i] + K * residual;
}

static int read_data(const char* filename) {

  FILE *fp = NULL;
  fp = fopen(filename, "r");
  if (fp == NULL)
    return -1;

  char * line = NULL;
  size_t len = 0;
  size_t read;
  n_dat = 0;
  while ((read = getline(&line, &len, fp)) != -1 && n_dat< NB_STEP) {
    if (sscanf(line, "%lf %*d VERT_LOOP %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
           &time[n_dat],
           &z_sp[n_dat], &zd_sp[n_dat],
           &est_z[n_dat], &est_zd[n_dat], &est_zdd[n_dat],
           &ref_z[n_dat], &ref_zd[n_dat], &ref_zdd[n_dat],
           &adp_inv_m[n_dat], &adp_cov[n_dat], &sum_err[n_dat],
           &ff_cmd[n_dat], &fb_cmd[n_dat], &delta_t[n_dat]) == 15) n_dat++;
  }
  if (line)
    free(line);
  fclose(fp);
  return 0;

}

void gen_data(void) {
  int i = 0;
  n_dat = NB_STEP;
  while (i<n_dat) {
    long int r = random();
    double noise = 2. * 2. * ((double)r / (double)RAND_MAX - 0.5);
    //    printf("%ld %f\n", r, noise);
    est_zdd[i] = 0 + noise;
    delta_t[i] = 85;
    i++;
  }
}

void dump_res(void) {
 int i = 0;
  while (i<n_dat) {
    printf("%f %d %d %d %f %f %f %d %d\n", time[i], est_zdd[i], delta_t[i], ref_zdd[i], ffX[i], ffP[i], ffm[i], ifX[i], ifP[i]);
    i++;
  }

}

#define FF_CMD_FRAC 18
void test_command(int i) {

  const int32_t inv_m_i =  gv_adapt_X>>(GV_ADAPT_X_FRAC - FF_CMD_FRAC);
  int32_t cmd_i = (BOOZ_INT_OF_FLOAT(9.81, FF_CMD_FRAC) - (ref_zdd[i]<<(FF_CMD_FRAC - IACCEL_RES))) / inv_m_i;

  double inv_m_f = (double)BOOZ_FLOAT_OF_INT(gv_adapt_X, GV_ADAPT_X_FRAC);
  double cmd_f = (9.81 - (double)BOOZ_FLOAT_OF_INT(ref_zdd[i], IACCEL_RES)) / inv_m_f;

  int32_t cmd_i_fixed;

  // = ((int32_t)BOOZ_INT_OF_FLOAT(9.81, IACCEL_RES) - ref_zdd[i] + (inv_m_i_fixed>>1)) / inv_m_i_fixed;

  if (ref_zdd[i] < BOOZ_INT_OF_FLOAT(9.81, IACCEL_RES))
    cmd_i_fixed = (BOOZ_INT_OF_FLOAT(9.81, FF_CMD_FRAC) - (ref_zdd[i]<<(FF_CMD_FRAC - IACCEL_RES)) + (inv_m_i>>1) ) / inv_m_i;
  else
    cmd_i_fixed = (BOOZ_INT_OF_FLOAT(9.81, FF_CMD_FRAC) - (ref_zdd[i]<<(FF_CMD_FRAC - IACCEL_RES)) - (inv_m_i>>1) ) / inv_m_i;

  printf("%d %f %d\n",cmd_i, cmd_f, cmd_i_fixed);
}

int main(int argc, char** argv) {
  //  gen_data();
  read_data("09_02_15__20_45_58.data");
  printf("read %d\n", n_dat);
  gv_adapt_init();
  float_filter_init();
  int i = 0;
  while (i<n_dat) {
    Bound(delta_t[i], 1, 200);
    gv_adapt_run(est_zdd[i], delta_t[i]);
    ifX[i] = gv_adapt_X;
    ifP[i] = gv_adapt_P;
    float_filter_run(i);
    //    test_command(i);
    i++;
  }

  dump_res();

  return 0;
}
