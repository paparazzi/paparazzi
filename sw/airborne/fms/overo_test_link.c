/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"


int main(int argc, char *argv[]) {
  
  if (spi_link_init())
    return -1;
  uint32_t foo = 0;
  while (1) {
    struct AutopilotMessageFoo msg_out;
    msg_out.foo  = foo;
    msg_out.bar  = foo;
    msg_out.blaa = foo;
    struct AutopilotMessageFoo msg_in;
    spi_link_send(&msg_out, sizeof(struct AutopilotMessageFoo), &msg_in);
    printf("%d %d %d\n", msg_in.foo, msg_in.bar, msg_in.blaa);
    //    usleep(1953);
    usleep(50000);
    foo++;
  }

  return 0;
}
