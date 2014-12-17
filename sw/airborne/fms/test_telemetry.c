
#include <stdio.h>
#include <glib.h>

int main(int argc, char **argv)
{


  GMainLoop *ml = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(ml);

  return 0;
}
