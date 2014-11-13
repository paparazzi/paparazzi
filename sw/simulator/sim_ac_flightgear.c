/*
 * Copyright (C) 2011 Eric Parsonage eric@eparsonage.com
 * Mainly based around the equivalent file in nps
 * which was written by Antoine
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

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <math.h>
#include "std.h"

// ignore stupid warnings in JSBSim
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <FGFDMExec.h>
#pragma GCC diagnostic pop

#include "flight_gear.h"
#include "sim_ac_flightgear.h"

static struct
{
    int socket;
    struct sockaddr_in addr;
} flightgear;


void sim_ac_flightgear_init(const char* host,  unsigned int port)
{
    int so_reuseaddr = 1;
    struct protoent * pte = getprotobyname("UDP");
    flightgear.socket = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
    setsockopt(flightgear.socket, SOL_SOCKET, SO_REUSEADDR,
               &so_reuseaddr, sizeof(so_reuseaddr));
    flightgear.addr.sin_family = PF_INET;
    flightgear.addr.sin_port = htons(port);
    flightgear.addr.sin_addr.s_addr = inet_addr(host);
}

static inline double get_value(JSBSim::FGFDMExec* FDMExec, string name)
{
    return FDMExec->GetPropertyManager()->GetNode(name)->getDoubleValue();
}

void sim_ac_flightgear_send(JSBSim::FGFDMExec* FDMExec)
{

    struct FGNetGUI gui;

    gui.version = FG_NET_GUI_VERSION;

    gui.latitude  = get_value(FDMExec, "position/lat-gc-rad");
    gui.longitude = get_value(FDMExec, "position/long-gc-rad");
    gui.altitude  = get_value(FDMExec, "position/h-sl-meters");
    //  printf("%f %f %f\n", gui.latitude, gui.longitude, gui.altitude);

    gui.agl = 1.111652;

    gui.phi = get_value(FDMExec, "attitude/roll-rad");
    gui.theta = get_value(FDMExec, "attitude/pitch-rad");
    gui.psi = get_value(FDMExec, "attitude/heading-true-rad");

    gui.vcas = 0.;
    gui.climb_rate = 0.;

    gui.num_tanks = 1;
    gui.fuel_quantity[0] = get_value(FDMExec, "propulsion/total-fuel-lbs");;

    //gui.cur_time = 3198060679ul;
    //gui.cur_time = 3198060679ul + rint(fdm.time);
    gui.cur_time = 3198101679ul;
    gui.warp = 1122474394ul;

    gui.ground_elev = 0.;

    gui.tuned_freq = 125.65;
    gui.nav_radial = 90.;
    gui.in_range = 1;
    gui.dist_nm = 10.;
    gui.course_deviation_deg = 0.;
    gui.gs_deviation_deg = 0.;

    if (sendto(flightgear.socket, (char*)(&gui), sizeof(gui), 0,
               (struct sockaddr*)&flightgear.addr, sizeof(flightgear.addr)) == -1)
        printf("error sending\n");

}
