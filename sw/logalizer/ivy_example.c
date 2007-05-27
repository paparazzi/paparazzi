#include <glib.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  //guint time_tick = atoi(argv[0]);
  //guint time_sec = atoi(argv[1]);
  //guint throttle = atoi(argv[2]);
  //guint new_mode = atoi(argv[3]);

}


int main ( int argc, char** argv) {
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  IvyInit ("IvyExample", "IvyExample READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);


  return 0;
}
