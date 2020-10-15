/*
 * Copyright (C) Murat BRONZ
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/fault/fault.c"
 * @author Murat BRONZ
 * Generates faults on the actuators
 */

#include "modules/fault/fault.h"

#if FLIGHTRECORDER_SDLOG
#include "subsystems/datalink/telemetry.h"
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

float upper_front_effectiveness;
float upper_right_effectiveness;
float upper_left_effectiveness;
float bottom_front_effectiveness;
float bottom_right_effectiveness;
float bottom_left_effectiveness;

void fault_init(void) {
upper_front_effectiveness = 1.0;
upper_right_effectiveness  = 1.0;
upper_left_effectiveness  = 1.0; 
bottom_front_effectiveness  = 1.0;
bottom_right_effectiveness  = 1.0; 
bottom_left_effectiveness  = 1.0;
}

void fault_Set_Upper_Front(float _v)
{
  upper_front_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

void fault_Set_Upper_Right(float _v)
{
  upper_right_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

void fault_Set_Upper_Left(float _v)
{
  upper_left_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

void fault_Set_Bottom_Front(float _v)
{
  bottom_front_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

void fault_Set_Bottom_Right(float _v)
{
  bottom_right_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

void fault_Set_Bottom_Left(float _v)
{
  bottom_left_effectiveness = _v;
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_SETTINGS(pprzlog_tp, flightrecorder_sdlog,
                &upper_front_effectiveness, &upper_right_effectiveness, &bottom_right_effectiveness, &bottom_front_efectiveness);
          }
#endif
}

// void fault_event() {}
// void fault_datalink_callback() {}

