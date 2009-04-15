#include "rdyb_telemetry.h"


extern void telemetry_init(void) {

  IvyInit ("Rdyb", "Rdyb READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

}




