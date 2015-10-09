#if USE_MISSION_COMMANDS_IN_NPS

#include "nps_ivy.h"
#include "nps_autopilot.h"

#include <stdlib.h>
#include <Ivy/ivy.h>

#include "generated/airframe.h"

#ifdef RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#include NPS_SENSORS_PARAMS

#define MSG_SIZE 128
extern uint8_t dl_buffer[MSG_SIZE];

/* mission specific Datalink Ivy functions */
static void on_DL_MISSION_GOTO_WP(IvyClientPtr app __attribute__((unused)),
                                  void *user_data __attribute__((unused)),
                                  int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_GOTO_WP_LLA(IvyClientPtr app __attribute__((unused)),
                                      void *user_data __attribute__((unused)),
                                      int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_CIRCLE(IvyClientPtr app __attribute__((unused)),
                                 void *user_data __attribute__((unused)),
                                 int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_CIRCLE_LLA(IvyClientPtr app __attribute__((unused)),
                                     void *user_data __attribute__((unused)),
                                     int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_SEGMENT(IvyClientPtr app __attribute__((unused)),
                                  void *user_data __attribute__((unused)),
                                  int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_SEGMENT_LLA(IvyClientPtr app __attribute__((unused)),
                                      void *user_data __attribute__((unused)),
                                      int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_PATH(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[]);


static void on_DL_MISSION_PATH_LLA(IvyClientPtr app __attribute__((unused)),
                                   void *user_data __attribute__((unused)),
                                   int argc __attribute__((unused)), char *argv[]);


static void on_DL_GOTO_MISSION(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[]);


static void on_DL_NEXT_MISSION(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[]);


static void on_DL_END_MISSION(IvyClientPtr app __attribute__((unused)),
                              void *user_data __attribute__((unused)),
                              int argc __attribute__((unused)), char *argv[]);

void nps_ivy_mission_commands_init(void)
{

  IvyBindMsg(on_DL_MISSION_GOTO_WP, NULL, "^(\\S*) MISSION_GOTO_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_GOTO_WP_LLA, NULL, "^(\\S*) MISSION_GOTO_WP_LLA (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_CIRCLE, NULL, "^(\\S*) MISSION_CIRCLE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_CIRCLE_LLA, NULL,
             "^(\\S*) MISSION_CIRCLE_LLA (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_SEGMENT, NULL,
             "^(\\S*) MISSION_SEGMENT (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_SEGMENT_LLA, NULL,
             "^(\\S*) MISSION_SEGMENT_LLA (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_PATH, NULL,
             "^(\\S*) MISSION_PATH (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MISSION_PATH_LLA, NULL,
             "^(\\S*) MISSION_PATH_LLA (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_GOTO_MISSION, NULL, "^(\\S*) GOTO_MISSION (\\S*) (\\S*)");
  IvyBindMsg(on_DL_NEXT_MISSION, NULL, "^(\\S*) NEXT_MISSION (\\S*)");
  IvyBindMsg(on_DL_END_MISSION, NULL, "^(\\S*) END_MISSION (\\S*)");

}

#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_MISSION_GOTO_WP(IvyClientPtr app __attribute__((unused)),
                                  void *user_data __attribute__((unused)),
                                  int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t i = 0;
  float dummy;
  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  for (i = 1; i < 5 ; i++) { //target components
    dummy = (float)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }

  mission_parse_GOTO_WP();
}

static void on_DL_MISSION_GOTO_WP_LLA(IvyClientPtr app __attribute__((unused)),
                                      void *user_data __attribute__((unused)),
                                      int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  uint8_t i = 0;
  int32_t dummy;
  for (i = 1; i < 4 ; i++) { //target components (lat, lon, alt in int)
    dummy = (int32_t)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }
  float d = (float)(atof(argv[6]));
  memcpy(&dl_buffer[i * 4], &d, 4); // duration

  mission_parse_GOTO_WP_LLA();
}

static void on_DL_MISSION_CIRCLE(IvyClientPtr app __attribute__((unused)),
                                 void *user_data __attribute__((unused)),
                                 int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t i = 0;
  float dummy;
  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  for (i = 1; i < 6 ; i++) { //target components
    dummy = (float)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }

  mission_parse_CIRCLE();
}

static void on_DL_MISSION_CIRCLE_LLA(IvyClientPtr app __attribute__((unused)),
                                     void *user_data __attribute__((unused)),
                                     int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  uint8_t i = 0;
  int32_t dummy;
  for (i = 1; i < 4 ; i++) { //target components (lat, lon, alt in int)
    dummy = (int32_t)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }
  float d = (float)(atof(argv[6])); // radius in m
  memcpy(&dl_buffer[4 * 4], &d, 4);
  d = (float)(atof(argv[7])); // duration
  memcpy(&dl_buffer[5 * 4], &d, 4);

  mission_parse_CIRCLE_LLA();
}

static void on_DL_MISSION_SEGMENT(IvyClientPtr app __attribute__((unused)),
                                  void *user_data __attribute__((unused)),
                                  int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t i = 0;
  float dummy;
  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  for (i = 1; i < 7 ; i++) { //target components
    dummy = (float)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }

  mission_parse_SEGMENT();
}

static void on_DL_MISSION_SEGMENT_LLA(IvyClientPtr app __attribute__((unused)),
                                      void *user_data __attribute__((unused)),
                                      int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  uint8_t i = 0;
  int32_t dummy;
  for (i = 1; i < 6 ; i++) { //target components
    dummy = (int32_t)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }
  float d = (float)(atof(argv[8]));
  memcpy(&dl_buffer[i * 4], &d, 4);

  mission_parse_SEGMENT_LLA();
}

static void on_DL_MISSION_PATH(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t i = 0;
  float dummy;
  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  for (i = 1; i < 13 ; i++) { //target components
    dummy = (float)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }
  dl_buffer[i * 4] = (uint8_t)(atoi(argv[2 + i])); //path nb

  mission_parse_PATH();
}

static void on_DL_MISSION_PATH_LLA(IvyClientPtr app __attribute__((unused)),
                                   void *user_data __attribute__((unused)),
                                   int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //insert mode

  uint8_t i = 0;
  int32_t dummy;
  for (i = 1; i < 12 ; i++) { //target components
    dummy = (int32_t)(atof(argv[2 + i]));
    memcpy(&dl_buffer[i * 4], &dummy, 4);
  }
  float d = (float)(atof(argv[2 + 12])); // duration
  memcpy(&dl_buffer[i * 4], &d, 4);
  dl_buffer[13 * 4] = (uint8_t)(atoi(argv[2 + 13])); //path nb

  mission_parse_PATH_LLA();
}

static void on_DL_GOTO_MISSION(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id
  dl_buffer[3] = (uint8_t)(atoi(argv[2])); //mission_id

  mission_parse_GOTO_MISSION();
}

static void on_DL_NEXT_MISSION(IvyClientPtr app __attribute__((unused)),
                               void *user_data __attribute__((unused)),
                               int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id

  mission_parse_NEXT_MISSION();
}

static void on_DL_END_MISSION(IvyClientPtr app __attribute__((unused)),
                              void *user_data __attribute__((unused)),
                              int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  dl_buffer[2] = (uint8_t)(atoi(argv[1])); //ac_id

  mission_parse_END_MISSION();
}


#endif /* USE_MISSION_COMMANDS_IN_NPS */
