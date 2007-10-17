#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[0]);
  float phi = atof(argv[1]);
  
  g_message("attitude %d %f", ac_id, phi);
}

void on_GPS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[0]);
  float course = atof(argv[4]);
  
  g_message("GPS %d %f", ac_id, course);
}

void on_Wind(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint ac_id = atoi(argv[1]);
  float w_dir = atof(argv[2]); // deg
  float w_speed = atof(argv[3]);
  float ac_aspeed = atof(argv[4]);

  g_message("WIND %d %f %f %f", ac_id, w_dir, w_speed, ac_aspeed);

}


int main ( int argc, char** argv) {
  
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);


  IvyInit ("IrCalib", "IrCalib READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Attitude, NULL, "^(\\S*) ATTITUDE (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_GPS, NULL, "^(\\S*) GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_Wind, NULL, "^(\\S*) WIND (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);




  return 0;
}
