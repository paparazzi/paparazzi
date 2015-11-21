/*
 * Copyright (C) Roland
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
 * @file "modules/stereocam/state2camera/state2camera.c"
 * @author Roland
 * Sends rotation using the stereoboard protocol over the UART.
 */

#include "modules/stereocam/state2camera/state2camera.h"
#include "modules/stereocam/stereoprotocol.h"
#include "subsystems/abi.h"
#include "state.h"
#include "mcu_periph/uart.h"
static int frame_number_sending = 0;
float lastKnownHeight = 0.0;
int pleaseResetOdroid = 0;

void write_serial_rot()
{
  struct Int32RMat *ltp_to_body_mat = stateGetNedToBodyRMat_i();
  static int32_t lengthArrayInformation = 11 * sizeof(int32_t);
  uint8_t ar[lengthArrayInformation];
  int32_t *pointer = (int32_t *) ar;
  for (int indexRot = 0; indexRot < 9; indexRot++) {
    pointer[indexRot] = ltp_to_body_mat->m[indexRot];
  }
  pointer[9] = (int32_t)(state.alt_agl_f * 100);  //height above ground level in CM.
  pointer[10] = frame_number_sending++;
  stereoprot_sendArray(&((UART_LINK).device), ar, lengthArrayInformation, 1);
}
