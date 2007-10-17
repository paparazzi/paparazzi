#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define DEG_OF_RAD(a) (a/M_PI*180.)
#define RAD_OF_DEG(a) (a*M_PI/180.)

#define AC_ID 43
#define IR_ROLL_NEUTRAL 0.
#define IR_360_LATERAL_CORRECTION   1.
#define IR_360_VERTICAL_CORRECTION  1.
#define IR_CORRECTION_RIGHT 1.
#define IR_CORRECTION_LEFT  1.

#define AIRSPEED 12.6
#define DT 0.25

#define g 9.81


float estimator_phi;
float gps_phi;


gboolean timeout_callback(gpointer data) {

  return TRUE;
}

void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[0]);
  estimator_phi = RAD_OF_DEG(atof(argv[1]));
  
  //  g_message("attitude %d %f", ac_id, estimator_phi);
}

void on_GPS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[0]);
  float course = atof(argv[4]);

  float angle_gspeed_deg = 90. - course / 10.;
  if (angle_gspeed_deg < -180.) angle_gspeed_deg += 360.;
  if (angle_gspeed_deg >  180.) angle_gspeed_deg -= 360.;

  float angle_gspeed_rad = RAD_OF_DEG(angle_gspeed_deg);
  static float old_psi = 0.;
  float delta_psi = angle_gspeed_rad - old_psi;
  old_psi = angle_gspeed_rad;

  /* tan(phi) = v^2 / (R*g) */
  /* R = (V * dt) / dpsi */
  
  if (fabs(delta_psi) < 1e-6)
    delta_psi = copysign(1e-6, delta_psi);
  float R = AIRSPEED * DT / delta_psi;

  gps_phi = - atan(AIRSPEED * AIRSPEED / R / g);



  g_message("gps %d % 3.1f\t % 3.0f \t%.1f \t%.1f", ac_id, angle_gspeed_deg, R, DEG_OF_RAD(gps_phi), DEG_OF_RAD(estimator_phi));
}

void on_Wind(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[1]);
  float w_dir = atof(argv[2]); // deg
  float w_speed = atof(argv[3]);
  float ac_aspeed = atof(argv[4]);

  //  g_message("wind %d %f %f %f", ac_id, w_dir, w_speed, ac_aspeed);
}

void on_IrSensors(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[0]);
  float lateral = atof(argv[2]);
  float vertical = atof(argv[3]);
  
  float ir_roll = lateral * IR_360_LATERAL_CORRECTION;
  float ir_top = vertical * IR_360_LATERAL_CORRECTION;

  float phi = atan2(ir_roll, ir_top) - IR_ROLL_NEUTRAL;

  if (phi >= 0) 
    phi *= IR_CORRECTION_RIGHT;
  else
    phi *= IR_CORRECTION_LEFT;

  //  g_message("ir_sensors %d %.0f %.0f (%.1f)", ac_id, lateral, vertical, DEG_OF_RAD(phi));
}

int main ( int argc, char** argv) {
  
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(500, timeout_callback, NULL);

  IvyInit ("IrCalib", "IrCalib READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Attitude, NULL, "^(\\S*) ATTITUDE (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_GPS, NULL, "^(\\S*) GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_Wind, NULL, "^(\\S*) WIND (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_IrSensors, NULL, "^(\\S*) IR_SENSORS (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}
