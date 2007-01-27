#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define TICK_PER_SEC 15000000.
void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint time_tick = atoi(argv[0]);
  guint time_sec = atoi(argv[1]);
  guint throttle = atoi(argv[2]);
  guint mode = atoi(argv[3]);

  float time = (float)time_sec + (float)time_tick / TICK_PER_SEC;
  printf("%f %d\n", time, throttle);

}


int main ( int argc, char** argv) {

  //  g_timeout_add(16, timeout_callback, chan);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  IvyInit ("MotorBench", "MotorBench READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);
  return 0;
}
