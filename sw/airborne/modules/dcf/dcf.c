/*
 * Copyright (C) 2017 Hector Garcia de Marina
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
 *
 */

#include <math.h>
#include <std.h>

#include "modules/dcf/dcf.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_dcf(struct transport_tx *trans, struct link_device *dev)
{
    int tablelen = 4*DCF_MAX_NEIGHBORS;
    pprz_msg_send_DCF(trans, dev, AC_ID, tablelen, &(tableNei[0][0]));
}
#endif // PERIODIC TELEMETRY

// Control
/*! Default gain k for the algorithm */
#ifndef DCF_GAIN_K
#define DCF_GAIN_K 10
#endif

dcf_con dcf_control = {DCF_GAIN_K};

/*! Default number of neighbors per aircraft */
#ifndef DCF_MAX_NEIGHBORS
#define DCF_MAX_NEIGHBORS 4
#endif
int16_t tableNei[DCF_MAX_NEIGHBORS][4];

void dcf_init(void)
{
    for(int i=0; i<DCF_MAX_NEIGHBORS; i++)
        tableNei[i][0] = -1;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DCF, send_dcf);
#endif
}

bool dcf_run(void)
{
    return true;
}
