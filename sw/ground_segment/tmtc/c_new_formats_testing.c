#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>



gboolean timeout_callback(gpointer data) {
  char i1 = 'x';
  char i2 = 'a';
  char i3 = 'v';
  char i4 = 'i';
  IvySendMsg("ME TEST 1 %c,%c,%c,%c", i1,i2,i3,i4);
  return TRUE;
}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("Test", "Test READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_timeout_add(100, timeout_callback, NULL);
  
  g_main_loop_run(ml);

  return 0;
}

