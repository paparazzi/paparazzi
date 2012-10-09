
/*

http://users.softlab.ntua.gr/~ttsiod/gnuplotStreaming.html

*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>
#include <sys/time.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define HEIGHT_SPAN 20000

FILE *Gplt, *Gplh;
int32_t alt = 0;
int32_t temp[HEIGHT_SPAN] = {0};
int32_t humid[HEIGHT_SPAN] = {0};

void on_GPS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
/*
   <message name="GPS" id="8">
     <field name="mode"       type="uint8"  unit="byte_mask"/>
     <field name="utm_east"   type="int32"  unit="cm" alt_unit="m"/>
     <field name="utm_north"  type="int32"  unit="cm" alt_unit="m"/>
     <field name="course"     type="int16"  unit="decideg" alt_unit="deg"/>
     <field name="alt"        type="int32"  unit="cm" alt_unit="m"/>
     <field name="speed"      type="uint16" unit="cm/s" alt_unit="m/s"/>
     <field name="climb"      type="int16"  unit="cm/s" alt_unit="m/s"/>
     <field name="week"       type="uint16" unit="weeks"></field>
     <field name="itow"       type="uint32" unit="ms"/>
     <field name="utm_zone"   type="uint8"/>
     <field name="gps_nb_err" type="uint8"/>
   </message>

7.73 11 GPS 0 55577549 665183336 0 -4310 0 0 1642 345957748 31 0
*/

  int32_t _alt;

  _alt = atoi(argv[5]);
  alt = _alt / 100;
//  if ((_alt/100) < HEIGHT_SPAN) alt = _alt;

//  printf("alt %f\n", (float) _alt/100.);
}

void on_TMP_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
/*
  <message name="TMP_STATUS" id="86">
    <field name="itemp"  type="uint16"/>
    <field name="temp"   type="float" unit="deg_celsius" format="%.2f"/>
  </message>
*/

  float _temp;
  int i;

  _temp = atof(argv[2]);
  if (alt < HEIGHT_SPAN) temp[alt] = _temp * 100;

//  printf("temp %f\n", _temp);
  fprintf(Gplt, "plot '-' w points pt 0 title \"Temp\"\n");
  for (i = 0; i < HEIGHT_SPAN; i++){
    if (temp[i] != 0) fprintf(Gplt, "%f %d\n", temp[i]/100., i);
  }
  fprintf(Gplt,"e\n");
}

void on_SHT_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
/*
  <message name="SHT_STATUS" id="89">
    <field name="ihumid" type="uint16"/>
    <field name="itemp"  type="uint16"/>
    <field name="humid"  type="float" unit="rel_hum" format="%.2f"/>
    <field name="temp"   type="float" unit="deg_celsius" format="%.2f"/>
  </message>
*/

  float _humid;
  int i;

  _humid = atof(argv[3]);
  if (alt < HEIGHT_SPAN) humid[alt] = _humid * 100;

//  printf("humid %f\n", _humid);
  fprintf(Gplh, "plot '-' w points pt 0 title \"Humid\"\n");
  for (i = 0; i < HEIGHT_SPAN; i++){
    if (humid[i] != 0) fprintf(Gplh, "%f %d\n", humid[i]/100., i);
  }
  fprintf(Gplh,"e\n");
}

int main( int argc, char* argv[] )
{
  double xmint, xmaxt, xminh, xmaxh, ymin, ymax;
  GMainLoop *ml;

  ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("IvyPlotProfile", "IvyPlotProfile READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_GPS, NULL, "^(\\S*) GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_TMP_STATUS, NULL, "^(\\S*) TMP_STATUS (\\S*) (\\S*)");
  IvyBindMsg(on_SHT_STATUS, NULL, "^(\\S*) SHT_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
//  IvyBindMsg(on_SHT_STATUS, NULL, "^(\\S*) DPICCO_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  xmint = 5;
  xmaxt = 35;
  xminh = 0;
  xmaxh = 100;
  ymin = 500;
  ymax = 2300;

  Gplt = popen("gnuplot -geometry 300x300 -noraise","w");
  setlinebuf(Gplt);
  fprintf(Gplt, "set xrange[%f:%f]\n", xmint, xmaxt);
  fprintf(Gplt, "set yrange[%f:%f]\n", ymin, ymax);

  Gplh = popen("gnuplot -geometry 300x300 -noraise","w");
  setlinebuf(Gplh);
  fprintf(Gplh, "set xrange[%f:%f]\n", xminh, xmaxh);
  fprintf(Gplh, "set yrange[%f:%f]\n", ymin, ymax);

  g_main_loop_run(ml);

  fclose(Gplt);
  fclose(Gplh);
  return 0;
}
