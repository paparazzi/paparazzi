#include "calibrator.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define DEG_OF_RAD(a) ((a)/M_PI*180.)
#define RAD_OF_DEG(a) ((a)*M_PI/180.)
#define NORM_ANGLE_RAD(a) {  while ((a) < -M_PI ) (a) += 2 * M_PI;  while ((a) > M_PI ) (a) -= 2 * M_PI; }

#define AC_ID 43
#define IR_ROLL_NEUTRAL 0.
#define IR_360_LATERAL_CORRECTION   1.
#define IR_360_VERTICAL_CORRECTION  1.
#define IR_CORRECTION_RIGHT 1.
#define IR_CORRECTION_LEFT  1.

#define DT 0.25

#define g 9.81

float est_airspeed;
float est_wind_dir;
float est_wind_speed;
float est_wind_east;
float est_wind_north;

float gnd_ir_phi;
float estimator_phi;


float gps_gs_east;
float gps_gs_north;
float gps_gs_angle;
float gps_gs_norm;

float gps_as_east;
float gps_as_north;
float gps_as_angle;
float gps_as_norm;

float gps_phi;

#define NB_POINTS 121
float estimator_phi_by_degres[NB_POINTS];
float alpha = 0.5;

static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]);
static void on_GPS(IvyClientPtr app, void *user_data, int argc, char *argv[]);
static void on_Wind(IvyClientPtr app, void *user_data, int argc, char *argv[]);
static void on_IrSensors(IvyClientPtr app, void *user_data, int argc, char *argv[]);
static void reset_average(void);


static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //guint ac_id = atoi(argv[0]);
  estimator_phi = RAD_OF_DEG(atof(argv[1]));
  //g_message("attitude %d %f", ac_id, estimator_phi);
  int gps_phi_deg = round(DEG_OF_RAD(gps_phi)); 
  if (fabs(gps_phi_deg) <= NB_POINTS/2) {
    unsigned int idx =  gps_phi_deg + NB_POINTS/2;
    estimator_phi_by_degres[idx] = (1-alpha) * estimator_phi_by_degres[idx] + alpha * DEG_OF_RAD(estimator_phi);
  }
}

void on_GPS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  unsigned int ac_id = atoi(argv[0]);
  float course = atof(argv[4]);
  float speed = atof(argv[6]);

  gps_gs_norm = speed / 100.;
  gps_gs_angle = RAD_OF_DEG(90. - course / 10.);
  NORM_ANGLE_RAD( gps_gs_angle );
  
  gps_gs_east  = gps_gs_norm * cos(gps_gs_angle);
  gps_gs_north = gps_gs_norm * sin(gps_gs_angle);

  gps_as_east = gps_gs_east - est_wind_east;
  gps_as_north = gps_gs_north - est_wind_north;

  gps_as_angle = atan2(gps_as_north,gps_as_east); 
  gps_as_norm = sqrt(gps_as_east*gps_as_east + gps_as_north*gps_as_north);

  static float old_psi = 0.;
  float delta_psi = gps_as_angle - old_psi;
  old_psi = gps_as_angle;
  NORM_ANGLE_RAD(delta_psi);

  /* tan(phi) = v^2 / (R*g) */
  /* R = (V * dt) / dpsi */
  
  if (fabs(delta_psi) < 1e-6)
    delta_psi = copysign(1e-6, delta_psi);

  float R = -gps_as_norm * DT / delta_psi;

  gps_phi = atan(gps_as_norm * gps_as_norm / R / g);

  printf("gps %d % 3.1f \t% 3.0f \t%.1f \t%.1f \t%.1f\n", ac_id, DEG_OF_RAD(delta_psi), R, DEG_OF_RAD(gps_phi), DEG_OF_RAD(estimator_phi), DEG_OF_RAD(gnd_ir_phi));
}

void on_Wind(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //guint ac_id = atoi(argv[1]);
  est_wind_dir = atof(argv[2]);
  est_wind_speed = atof(argv[3]);
  est_airspeed = atof(argv[4]);

  float w_dir_rad =RAD_OF_DEG(270. - est_wind_dir);
  NORM_ANGLE_RAD( w_dir_rad );

  est_wind_east = est_wind_speed * cos( w_dir_rad );
  est_wind_north = est_wind_speed * sin( w_dir_rad );

  //g_message("wind %d %f %f %f", ac_id, w_dir, w_speed, ac_aspeed);
  //g_message("wind %f %f %f", w_dir_rad,  est_wind_east,  est_wind_north);
}

void on_IrSensors(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //guint ac_id = atoi(argv[0]);
  float lateral = atof(argv[2]);
  float vertical = atof(argv[3]);
  
  float ir_roll = lateral * IR_360_LATERAL_CORRECTION;
  float ir_top = vertical * IR_360_LATERAL_CORRECTION;

  gnd_ir_phi = atan2(ir_roll, ir_top) - IR_ROLL_NEUTRAL;

  if (gnd_ir_phi >= 0) 
    gnd_ir_phi *= IR_CORRECTION_RIGHT;
  else
    gnd_ir_phi *= IR_CORRECTION_LEFT;

  //  g_message("ir_sensors %d %.0f %.0f (%.1f)", ac_id, lateral, vertical, DEG_OF_RAD(phi));
}

static void reset_average(void) {

  int i;
  for (i=0; i<NB_POINTS; i++) {
    estimator_phi_by_degres[i] = i-NB_POINTS/2;
  }

}


void calibrator_init(void) {

  IvyInit ("IrCalib", "IrCalib READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Attitude, NULL, "^(\\S*) ATTITUDE (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_GPS, NULL, "^(\\S*) GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_Wind, NULL, "^(\\S*) WIND (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_IrSensors, NULL, "^(\\S*) IR_SENSORS (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  reset_average();
}

float* calibrator_get_values(void) {
  return  estimator_phi_by_degres;
}
