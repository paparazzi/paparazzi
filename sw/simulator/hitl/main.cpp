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
 * @file main.cpp
 *
 * HITL demo version
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <list>
#include <set>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "src/LogTime.h"
#include "src/Autopilot.h"
#include "src/VectorNav.h"
#include "src/Sender.h"

#include <boost/version.hpp>

using namespace std;

const double sim_dt = (1.0 / 400.0); // we're running at 400Hz

static Autopilot *ap;
static VectorNav *vn;
static boost::asio::serial_port *apt;
static boost::asio::serial_port *vpt;

/**
 * Opens the autopilot serial port so the correct pointer can be passed
 *
 */
void init_ap()
{
  try {
    apt->open("/dev/ttyUSB2"); // NOTE: change accordingly
    apt->set_option(boost::asio::serial_port_base::baud_rate(AP_BAUD));
  } catch (const std::exception &e) {
    std::cout << "AP: Exception: " << e.what() << "\n";
    exit(-1);
  }
  cout << "AP port opened!\n";
}

/**
 * Open Vectornav serial port
 */
void init_vn()
{
  try {
    vpt->open("/dev/ttyUSB1"); // NOTE: change accordingly
    vpt->set_option(boost::asio::serial_port_base::baud_rate(AP_BAUD));
  } catch (const std::exception &e) {
    std::cout << "VN: Exception: " << e.what() << "\n";
    exit(-1);
  }
  cout << "VN port opened!\n";
}

int main()
{
  std::cout << "Boost version: " << BOOST_LIB_VERSION << endl;

  timeval t = LogTime::getStart();

  boost::asio::io_service io1;
  boost::asio::serial_port port1(io1);
  apt = &port1;
  init_ap();
  Autopilot *ap = new Autopilot(*apt, t);

  boost::asio::io_service io2;
  boost::asio::serial_port port2(io2);
  vpt = &port2;
  init_vn();
  VectorNav *vn = new VectorNav(*vpt, t);

  cout << "Sim dt = " << sim_dt << "[s]" << endl;

  boost::asio::io_service log_io_service;
  Sender_ptr sender(new Sender(log_io_service, atoi("5505"), 30, ap, vn, sim_dt, true, t));


  boost::thread_group threads;
  cout << LogTime::getTime()  << " Binding log io service at " << endl;
  threads.create_thread(boost::bind(&boost::asio::io_service::run, &log_io_service));
  cout << "Joining threads\n";
  threads.join_all();
  return 0;
}


