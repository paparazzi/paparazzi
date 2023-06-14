/* 
 * Main function for testing the control allocation wls package. 
 * 
 * Till Blaha 2022
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "solveActiveSet.h"
#include "setupWLS.h"
#include <time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "test_cases.h"

static void executeSolution(
    int n_v,
    int n_u,
    num_t JG[AS_N_V*AS_N_U],
    num_t Wv[AS_N_V],
    num_t Wu[AS_N_U],
    num_t up[AS_N_U],
    num_t dv[AS_N_V],
    num_t lb[AS_N_U],
    num_t ub[AS_N_U],
    num_t u0[AS_N_U],
    activeSetAlgoChoice choice,
    num_t theta,
    num_t cond_bound,
    int8_t Ws[AS_N_U],
    num_t us[AS_N_U],
    bool compute_errors,
    num_t us_gt[AS_N_U],
    num_t* v_gt_norm,
    num_t* us_gt_norm,
    num_t* v_rel_error,
    num_t* us_rel_error,
    double* exec_time_us,
    int* n_free,
    int* iter
) 
{
  num_t A[AS_N_C*AS_N_U];
  num_t b[AS_N_C];
  num_t gamma;
  int8_t Ws_use[AS_N_U];

  for (int i=0; i<n_u; i++) {
    us[i] = u0[i];
    Ws_use[i] = Ws[i];
  }

  clock_t begin = clock();
  setupWLS_A(
    JG,
    Wv,
    Wu,
    n_v,
    n_u,
    theta,
    cond_bound,
    A,
    &gamma
  );
  setupWLS_b(
    dv,
    up,
    Wv,
    Wu,
    n_v,
    n_u,
    gamma,
    b
  );

  //printf("%f\n", gamma);
#ifdef AS_RECORD_COST
  num_t costs[AS_RECORD_COST_N];
#else
  num_t *costs = 0;
#endif
  solveActiveSet(choice)(
    A,
    b,
    lb,
    ub,
    us,
    Ws_use,
    //Ws, // for "testing" warm starting
    100,
    n_u,
    n_v,
    iter,
    n_free,
    costs
  );
  clock_t end = clock();
  *exec_time_us = (double)(end - begin) / CLOCKS_PER_SEC * 1e6;

  //printf("%d\n", iter);
  /*
  for (int i=0; i<n_u; i++) {
    printf("%f, ", us[i]);
  }
  printf("\n");
  */

  num_t normsq;
  num_t normsq_gt;
  num_t diff;

  if (compute_errors) {
    // verify by solution u
    normsq = 0.0F;
    normsq_gt = 0.0F;
    for (int j=0; j < n_u; j++) {
      diff = (us[j] - us_gt[j]);
      normsq += diff*diff;
      normsq_gt += us_gt[j]*us_gt[j];
    }
    *us_gt_norm = sqrt(normsq_gt);
    *us_rel_error = sqrt(normsq); // IMPORTANT CHANGE --> no more division by normsq_gt to avoid nan's

    // verify by impact on reached pseudocontrol Delta v
    normsq = 0.0F;
    normsq_gt = 0.0F;
    for (int j=0; j < n_v; j++) {
      diff = 0.0F;
      for (int k=0; k < n_u; k++) {
        diff += JG[j + (n_v)*k] * (us[k] - us_gt[k]);
      }
      normsq += diff*diff;
      normsq_gt += dv[j]*dv[j];
    }
    *v_gt_norm = sqrt(normsq_gt);
    *v_rel_error = sqrt(normsq/normsq_gt);
  }
}

static void main_solveActiveSet(int mode, activeSetAlgoChoice choice)
{

  int N;
  switch (mode)
  {
    case 1:
    case 2:
      // verification mode, just one pass over test cases
      N = 1;
      break;

    case 10:
    // timing mode, 1000 passes
      N = 1000;
      break;

    case 20:
      // csv output mode
      N = 500;
      break;
  }

  // get test cases
  TestCase test_cases[N_CASES];
  fill_cases(test_cases);

  // predefine arrays
  int8_t Ws[AS_N_U] = {1, 0, -1, 1, 0, 0};
  //memset(Ws, 0, sizeof(int8_t)*AS_N_U);

  // global settings
  //num_t theta = 1.5e-3 * sqrt(1e-4);  // works: 1.5e-3 * 1e0
  num_t theta = 2.0e-9;
  num_t cond_bound = 4e5; // works: 4.45e5. Optimal 2.5e6 for nederdrone? 2.4e6 for hex?

  num_t max_v_error = 0.0F;
  num_t max_us_error = 0.0F;
  num_t max_perturb = 0.0F;
  num_t avg_perturb = 0.0F;
  num_t us_algo[AS_N_U];
  num_t us_algo_perturb[AS_N_U];
  int idx;
  num_t perturb_frac = 0.01;

  num_t v_gt_norm[2];
  num_t us_gt_norm[2];
  num_t v_rel_error[2];
  num_t us_rel_error[2];

  bool verify_errors = false;
  if (mode < 10)
    verify_errors = true;

  int i = 0;
  bool first_run = false;
  if (mode == 20)
    printf("idx,v_error,u_error,u_pert,exec_time,n_satch,iter\n");

  double exec_time_total_us = 0;
  for (i=0; i < N*N_CASES; i++) {
  //for (int i=179; i < 180; i++) {
    idx = (int) i/N;
    first_run = !(i%N);

    //test_cases[idx].Wv[0] = sqrtf(1000.);
    //test_cases[idx].Wv[1] = sqrtf(1000.);
    //test_cases[idx].Wv[2] = sqrtf(1.);
    //test_cases[idx].Wv[3] = sqrtf(100.);

    double exec_time_us;
    int n_free;
    int iter;
    if (first_run) {
      exec_time_total_us = 0;
    }

    executeSolution(
      test_cases[idx].n_v,
      test_cases[idx].n_u,
      test_cases[idx].JG,
      test_cases[idx].Wv,
      test_cases[idx].Wu,
      test_cases[idx].up,
      test_cases[idx].v,
      test_cases[idx].lb,
      test_cases[idx].ub,
      test_cases[idx].u0,
      choice,
      theta,
      cond_bound,
      Ws,
      us_algo,
      verify_errors || (mode == 20 || first_run),
      test_cases[idx].us,
      v_gt_norm,
      us_gt_norm,
      v_rel_error,
      us_rel_error,
      &exec_time_us,
      &n_free,
      &iter
    );


    exec_time_total_us += exec_time_us;
    if (!((i+1)%N) && mode == 20) {
      printf("%f,%d,%d\n", exec_time_total_us/N, test_cases[idx].n_u-n_free, iter);
    }


    if ((mode < 10) || ((mode == 20) && first_run)) {

      max_v_error = (max_v_error < v_rel_error[0]) ? v_rel_error[0] : max_v_error;
      max_us_error = (max_us_error < us_rel_error[0]) ? us_rel_error[0] : max_us_error;

      // do perturbation
      for (int j=0; j<test_cases[idx].n_v; j++)
        test_cases[idx].v[j] *= (1.0F + perturb_frac);

      executeSolution(
        test_cases[idx].n_v,
        test_cases[idx].n_u,
        test_cases[idx].JG,
        test_cases[idx].Wv,
        test_cases[idx].Wu,
        test_cases[idx].up,
        test_cases[idx].v,
        test_cases[idx].lb,
        test_cases[idx].ub,
        test_cases[idx].u0,
        choice,
        theta,
        cond_bound,
        Ws,
        us_algo_perturb,
        true,
        test_cases[idx].us,
        v_gt_norm+1,
        us_gt_norm+1,
        v_rel_error+1,
        us_rel_error+1,
        &exec_time_us,
        &n_free,
        &iter
      );

      num_t perturbation = 0.0;

      for (int j=0; j<test_cases[idx].n_u; j++)
        perturbation += (us_algo_perturb[j] - us_algo[j])*(us_algo_perturb[j] - us_algo[j]);
      
      perturbation = sqrt(perturbation); // / us_gt_norm[0]; IMPORTANT CHANGE, NO MORE division by gt norm

      max_perturb = (max_perturb < perturbation) ? perturbation  : max_perturb;
      avg_perturb += perturbation / N_CASES;

      #ifdef AS_VERBOSE
      printf("%d u: %.7f | %.7f%%\n", i, us_gt_norm[0], 100*us_rel_error[0]);
      printf("%d v: %.7f | %.7f%%\n", i, v_gt_norm[0], 100*v_rel_error[0]);
      #endif

      if (mode == 20)
        printf("%d,%.12f,%.12f,%.12f,", idx, v_rel_error[0], us_rel_error[0], perturbation);

    }
  }

  if (mode < 10) {
    printf("Max relative pseudocontrol error: %.7f%%\n", 100*max_v_error);
    printf("Max relative actuator error: %.7f%%\n", 100*max_us_error);
    printf("Max relative perturbation ratio: %.7f%%\n", 100*max_perturb);
    printf("Avg relative perturbation ratio: %.7f%%\n", 100*avg_perturb);
  }
}

int main(int argc, char **argv)
{
  if (argc > 1) {
    if (!strcmp(argv[1], "verify")) {
      main_solveActiveSet(1, (activeSetAlgoChoice) atoi(argv[2]));
    } else if (!strcmp(argv[1], "time")){
      main_solveActiveSet(10, (activeSetAlgoChoice) atoi(argv[2]));
    } else if (!strcmp(argv[1], "csv")){
      main_solveActiveSet(20, (activeSetAlgoChoice) atoi(argv[2]));
    } else {
      return 1;
    }
  } else {
    return 2;
  }

  return 0;
}
