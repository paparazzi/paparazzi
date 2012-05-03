#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>



gboolean timeout_callback(gpointer data) {
  IvySendMsg("ME ISSENDING 1");//XGGDEBUG:DELETE FILE, and makefile sentence
  unsigned long long i1 = 1234567890123456789ULL;
  char i2 = 'x';
  char i3 = 'a';

  int i4[3] = {1234,5432,1111};
  int i6 = -1234;
  IvySendMsg("ME TEST %llu %c,%c %d,%d,%d %d 1", i1,i2,i3,i4[0],i4[1],i4[2],i6);
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

