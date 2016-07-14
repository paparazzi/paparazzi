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



NpsIvy::NpsIvy(){
  init_bus();
#ifdef __APPLE__
  const char *default_ivy_bus = "224.255.255.255";
#else
  const char *default_ivy_bus = "127.255.255.255";
#endif
  IvyStart(default_ivy_bus);
  main_loop();
}

NpsIvy::NpsIvy(std::string bus){
  init_bus();
  IvyStart(bus.c_str());
  main_loop();
}

NpsIvy::~NpsIvy(){
  //IvyStop();

}


void NpsIvy::init_bus(void){
  //const char *agent_name = AIRFRAME_NAME"_NPS";
  const char *agent_name = "M_NPS";
  //const char *ready_msg = AIRFRAME_NAME"_NPS Ready";
  const char *ready_msg = "M_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);

  // bind on a general WORLD_ENV (not a reply to request)
  IvyBindMsg(NpsIvy::on_WORLD_ENV, NULL, "^(\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  // to be able to change datalink_enabled setting back on
  IvyBindMsg(NpsIvy::on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");

  /* binding of HelloCallback to messages starting with 'Hello' */
  IvyBindMsg(NpsIvy::HelloCallback, 0, "^Hello(.*)");

  /* binding of ByeCallback to 'Bye' */
  IvyBindMsg(NpsIvy::ByeCallback, 0, "^Bye$");
}

void NpsIvy::main_loop(void){
  std::thread th_main(&NpsIvy::run_main_loop, this);
  th_main.detach();

  std::thread th_sender(&NpsIvy::run_sender, this);
  th_sender.detach();
}

void NpsIvy::display(void){
  IvySendMsg("%d NPS_WIND %f %f %f",
             1,
             12.1,
             12.2,
             12.3);
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
    std::cout << "NpsIvy run sender Thread running" << std::endl;
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
