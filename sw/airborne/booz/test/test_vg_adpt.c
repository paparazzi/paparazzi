#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>

#include "std.h"
#include "booz_geometry_mixed.h"
#define B2_GUIDANCE_V_C
#include "booz2_guidance_v_adpt.h"



#define NB_STEP 20000
int n_dat;

double time[NB_STEP];

int32_t measure[NB_STEP];
int32_t thrust[NB_STEP];
int32_t zdd_ref[NB_STEP];

int32_t ifX[NB_STEP];
int32_t ifP[NB_STEP];


double ffX[NB_STEP];
double ffm[NB_STEP];
double ffP[NB_STEP];

static void float_filter_init(void) {
  ffX[0] = B2_GV_ADAPT_X0_F;
  ffP[0] = B2_GV_ADAPT_P0_F;
}

static void float_filter_run( int i) {

  int prev = i>0 ? i-1 : i;
  ffX[i] = ffX[prev];
  ffP[i] = ffP[prev];
  if (thrust[prev] == 0) return;
  ffP[i] = ffP[i] + B2_GV_ADAPT_SYS_NOISE_F;
  ffm[i] = (9.81 - (double)measure[i]/(double)(1<<10)) / (double)thrust[prev];
  double residual = ffm[i] - ffX[i];
  double E = ffP[i] + B2_GV_ADAPT_MEAS_NOISE_F;
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
    if (sscanf(line, "%lf %*d BOOZ2_VERT_LOOP %*d %*d %*d %d %*d %*d %d %*d %d", 
	       &time[n_dat], &measure[n_dat], &zdd_ref[n_dat], &thrust[n_dat]) == 4) n_dat++;
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
    measure[i] = 0 + noise;
    thrust[i] = 85;
    i++;
  }
}

void dump_res(void) {
 int i = 0;
  while (i<n_dat) {
    printf("%f %d %d %d %f %f %f %d %d\n", time[i], measure[i], thrust[i], zdd_ref[i], ffX[i], ffP[i], ffm[i], ifX[i], ifP[i]);
    i++;
  }
  
}


int main(int argc, char** argv) {
  //  gen_data();
  read_data("vert_loop.dat");
  printf("read %d\n", n_dat);
  b2_gv_adapt_init();
  float_filter_init();
  int i = 0;
  while (i<n_dat) {
    b2_gv_adapt_run(measure[i], thrust[i]);
    ifX[i] = b2_gv_adapt_X;
    ifP[i] = b2_gv_adapt_P;
    float_filter_run(i);
    i++;
  }
  
  dump_res();

  return 0;
}
