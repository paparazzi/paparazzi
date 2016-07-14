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
 * @file NpsIvy.h
 *
 * C++ Ivy wrapper for NPS
 *
 */

#ifndef NPS_IVY
#define NPS_IVY

#include <string.h>    // String function definitions
#include <chrono>
#include <iostream>
#include <thread>      // threads
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>




#define IVY_DISPLAY_PERIOD_MS 1000

class NpsIvy
{
  //extern void nps_ivy_init(char *ivy_bus);
  public:
    NpsIvy();
    NpsIvy(std::string bus);
    ~NpsIvy();

  private:
    void init_bus(void);
    void main_loop(void);
    void display(void); // sends messages
    void run_sender(void);
    void run_main_loop(void);

    /*
     * Parse WORLD_ENV message from gaia.
     *
     */
    static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                             void *user_data __attribute__((unused)),
                             int argc __attribute__((unused)), char *argv[])
    {
      printf("WORLD_ENV received!\n");
    }


    static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                              void *user_data __attribute__((unused)),
                              int argc __attribute__((unused)), char *argv[])
    {
      printf("DL_SETTINGS received!\n");
    }

    /* callback associated to "Hello" messages */
    static void HelloCallback (IvyClientPtr app, void *data, int argc, char **argv)
    {
      const char* arg = (argc < 1) ? "" : argv[0];
      IvySendMsg ("Bonjour%s", arg);
    }

    /* callback associated to "Bye" messages */
    static void ByeCallback (IvyClientPtr app, void *data, int argc, char **argv)
    {
      IvySendMsg ("Byebye");
      IvyStop ();
    }
};

#endif /* NPS_IVY */
