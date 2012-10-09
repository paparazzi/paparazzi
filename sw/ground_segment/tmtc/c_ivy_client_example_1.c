#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //  guint ac_id = atoi(argv[0]);
  //  float estimator_phi = atof(argv[1]);
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Example1", "Example1 READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Attitude, NULL, "^(\\S*) ATTITUDE (\\S*) (\\S*) (\\S*)");

  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}

