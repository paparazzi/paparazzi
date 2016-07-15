/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file NpsIvy.c
 *
 * C++ Ivy wrapper for NPS
 *
 */
#include "NpsIvy.h"

extern "C" {
  #include "generated/airframe.h"
  #include "math/pprz_algebra_int.h"
  #include "math/pprz_algebra_float.h"
  #include "math/pprz_algebra_double.h"
  #include "nps_main.h"
  #include "nps_autopilot.h"
  #include "nps_atmosphere.h"
  #include "generated/settings.h"
  #include "pprzlink/dl_protocol.h"
  #include "subsystems/datalink/downlink.h"

#if USE_GPS
  #include "subsystems/gps.h"
#endif

  #include NPS_SENSORS_PARAMS
}

NpsIvy::NpsIvy(struct NpsFdm* fdm_ref, struct NpsSensors* sensors_ref){
  fdm = fdm_ref;
  sensors = sensors_ref;

  init_bus();
#ifdef __APPLE__
  const char *default_ivy_bus = "224.255.255.255";
#else
  const char *default_ivy_bus = "127.255.255.255";
#endif
  IvyStart(default_ivy_bus);
  main_loop();
}

NpsIvy::NpsIvy(struct NpsFdm* fdm_ref, struct NpsSensors* sensors_ref, char *ivy_bus){
  fdm = fdm_ref;
  sensors = sensors_ref;

  init_bus();
  IvyStart(ivy_bus);
  main_loop();
}


NpsIvy::~NpsIvy(){
  IvyStop();
}


/*
 * Parse WORLD_ENV message from gaia.
 *
 */
static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[])
{
  // wind speed in m/s
  struct FloatVect3 wind;
  wind.x = atof(argv[1]); //east
  wind.y = atof(argv[2]); //north
  wind.z = atof(argv[3]); //up

  /* set wind speed in NED */
  nps_atmosphere_set_wind_ned(wind.y, wind.x, -wind.z);

  /* not used so far */
  //float ir_contrast = atof(argv[4]);

  /* set new time factor */
  nps_set_time_factor(atof(argv[5]));

#if USE_GPS
  // directly set gps fix in subsystems/gps/gps_sim_nps.h
  gps_has_fix = atoi(argv[6]); // gps_availability
#endif
}


static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc __attribute__((unused)), char *argv[])
{
  if (atoi(argv[1]) != AC_ID) {
    return;
  }

  /* HACK:
   * we actually don't want to allow changing settings if datalink is disabled,
   * but since we currently change this variable via settings we have to allow it
   * TODO: only allow changing the datalink_enabled setting
   */

  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  printf("setting %d %f\n", index, value);
}


void NpsIvy::init_bus(void){
  const char *agent_name = AIRFRAME_NAME"_NPS";
  const char *ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);

  // bind on a general WORLD_ENV (not a reply to request)
  IvyBindMsg(on_WORLD_ENV, NULL, "^(\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  // to be able to change datalink_enabled setting back on
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
}

void NpsIvy::main_loop(void){
  std::thread th_main(&NpsIvy::run_main_loop, this);
  th_main.detach();

  std::thread th_sender(&NpsIvy::run_sender, this);
  th_sender.detach();
}

void NpsIvy::display(void){
  IvySendMsg("%d NPS_RATE_ATTITUDE %f %f %f %f %f %f",
             AC_ID,
             DegOfRad(fdm->body_ecef_rotvel.p),
             DegOfRad(fdm->body_ecef_rotvel.q),
             DegOfRad(fdm->body_ecef_rotvel.r),
             DegOfRad(fdm->ltp_to_body_eulers.phi),
             DegOfRad(fdm->ltp_to_body_eulers.theta),
             DegOfRad(fdm->ltp_to_body_eulers.psi));
  IvySendMsg("%d NPS_POS_LLH %f %f %f %f %f %f %f %f %f",
             AC_ID,
             (fdm->lla_pos_pprz.lat),
             (fdm->lla_pos_geod.lat),
             (fdm->lla_pos_geoc.lat),
             (fdm->lla_pos_pprz.lon),
             (fdm->lla_pos_geod.lon),
             (fdm->lla_pos_pprz.alt),
             (fdm->lla_pos_geod.alt),
             (fdm->agl),
             (fdm->hmsl));
  IvySendMsg("%d NPS_SPEED_POS %f %f %f %f %f %f %f %f %f",
             AC_ID,
             (fdm->ltpprz_ecef_accel.x),
             (fdm->ltpprz_ecef_accel.y),
             (fdm->ltpprz_ecef_accel.z),
             (fdm->ltpprz_ecef_vel.x),
             (fdm->ltpprz_ecef_vel.y),
             (fdm->ltpprz_ecef_vel.z),
             (fdm->ltpprz_pos.x),
             (fdm->ltpprz_pos.y),
             (fdm->ltpprz_pos.z));
  IvySendMsg("%d NPS_GYRO_BIAS %f %f %f",
             AC_ID,
             DegOfRad(RATE_FLOAT_OF_BFP(sensors->gyro.bias_random_walk_value.x) + sensors->gyro.bias_initial.x),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors->gyro.bias_random_walk_value.y) + sensors->gyro.bias_initial.y),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors->gyro.bias_random_walk_value.z) + sensors->gyro.bias_initial.z));

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  double_quat_vmult(&h_body, &fdm->ltp_to_body_quat, &fdm->ltp_h);

  IvySendMsg("%d NPS_SENSORS_SCALED %f %f %f %f %f %f",
             AC_ID,
             ((sensors->accel.value.x - sensors->accel.neutral.x) / NPS_ACCEL_SENSITIVITY_XX),
             ((sensors->accel.value.y - sensors->accel.neutral.y) / NPS_ACCEL_SENSITIVITY_YY),
             ((sensors->accel.value.z - sensors->accel.neutral.z) / NPS_ACCEL_SENSITIVITY_ZZ),
             h_body.x,
             h_body.y,
             h_body.z);

  IvySendMsg("%d NPS_WIND %f %f %f",
             AC_ID,
             fdm->wind.x,
             fdm->wind.y,
             fdm->wind.z);
}

void NpsIvy::run_main_loop(void){
  IvyMainLoop();
}

void NpsIvy::run_sender(void){
  std::chrono::high_resolution_clock::time_point start;
  std::chrono::high_resolution_clock::time_point stop;
  std::chrono::duration<int32_t, std::nano> sleep_time;
  std::chrono::milliseconds period(IVY_DISPLAY_PERIOD_MS);

  std::cout << "NpsIvy run sender Thread init" << std::endl;

  while(true)
  {
    start = std::chrono::high_resolution_clock::now();
    display();
    stop = std::chrono::high_resolution_clock::now();
    sleep_time = period - (stop - start);
    if(sleep_time > std::chrono::duration<int32_t,std::nano>(0))
    {
      std::this_thread::sleep_for(sleep_time);
    }
    else
    {
      std::cout << "NPSIvy run sender: We took too long" << std::endl;
    }
  }
}
