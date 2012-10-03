#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>



gboolean timeout_callback(gpointer data) {
  int foo = 42;
  IvySendMsg("ME HELLO_WORLD 1234 5678 %d", foo);
  return TRUE;
}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Example2", "Example2 READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_timeout_add(100, timeout_callback, NULL);

  g_main_loop_run(ml);

  return 0;
}

