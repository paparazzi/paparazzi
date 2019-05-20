#!/usr/bin/env python3
#
# Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

"""
Connect to paparazzi server to get the aircraft list and configurations

See http://wiki.paparazziuav.org/wiki/DevGuide/Server_GCS_com for more details

:Example:

    import pprz_connect
    import time

    # define a callack
    def new_ac(conf):
        print(conf)

    connect = PprzConnect(notify=new_ac)

    # do some things here or wait for event
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    # close before leaving
    connect.shutdown()

"""

from __future__ import print_function
import sys
from os import path, getenv, getpid
from time import sleep

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from ivy.std_api import *

class PprzConfig(object):
    """
    Aircraft configuration as sent by the Paparazzi server.
    Getter/setter functions should be used to access the attributes.
    """

    def __init__(self, ac_id, ac_name, airframe, flight_plan, settings, radio, color):
        self._ac_id = ac_id
        self._ac_name = ac_name
        self._airframe = airframe
        self._flight_plan = flight_plan
        self._settings = settings
        self._radio = radio
        self._color = color

    def __str__(self):
        conf_str = 'A/C {} with ID {}\n\tairframe: {}\n\tflight plan: {}\n\tsettings: {}\n\tradio: {}\n\tcolor: {}'.format(
                self._ac_name, self._ac_id, self._airframe, self._flight_plan, self._settings, self._radio, self._color)
        return conf_str

    @property
    def id(self):
        return self._ac_id

    @property
    def name(self):
        return self._ac_name

    @property
    def airframe(self):
        return self._airframe

    @property
    def flight_plan(self):
        return self._flight_plan

    @property
    def settings(self):
        return self._settings

    @property
    def radio(self):
        return self._radio

    @property
    def color(self):
        return self._color


class PprzConnect(object):
    """
    Main class to handle the initialization process with the server
    in order to retrieve the configuration of the known aircraft
    and update for the new ones
    """

    def __init__(self, notify=None, ivy=None, verbose=False):
        """
        Init function
        Create an ivy interface if not provided and request for all aircraft

        :param notify: callback function called on new aircraft, takes a PprzConfig as parameter
        :param ivy: ivy interface to contact the server, if None a new one will be created
        :param verbose: display debug information
        """
        self.verbose = verbose
        self._notify = notify
        self._req_idx = 0

        self._conf_list_by_name = {}
        self._conf_list_by_id = {}

        if ivy is None:
            self._ivy = IvyMessagesInterface("PprzConnect")
        else:
            self._ivy = ivy
        sleep(0.1)

        self.get_aircrafts()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        """
        Shutdown function

        Should be called before leaving if the ivy interface is not closed elsewhere
        """
        if self._ivy is not None:
            if self.verbose:
                print("Shutting down ivy interface...")
            self._ivy.shutdown()
            self._ivy = None

    @property
    def conf_by_name(self, ac_name=None):
        """
        Get a conf by its name

        :param ac_name: aircraft name, if None the complete dict is returned
        :type ac_name: str
        """
        if ac_name is not None:
            return self._conf_list_by_name[ac_name]
        else:
            return self._conf_list_by_name

    @property
    def conf_by_id(self, ac_id=None):
        """
        Get a conf by its ID

        :param ac_id: aircraft id, if None the complete dict is returned
        :type ac_id: str
        """
        if ac_id is not None:
            return self._conf_list_by_id[ac_id]
        else:
            return self._conf_list_by_id

    @property
    def ivy(self):
        """
        Getter function for the ivy interface
        """
        return self._ivy

    def _get_req_id(self):
        req_id = '{}_{}'.format(getpid(), self._req_idx)
        self._req_idx += 1
        return req_id

    def _message_req(self, msg_name, cb, params=None):
        bind_id = None
        def _cb(sender, msg):
            if bind_id is not None:
                self._ivy.unsubscribe(bind_id)
            cb(sender, msg)
        req_id = self._get_req_id()
        req_regex = '^{} ([^ ]* +{}( .*|$))'.format(req_id, msg_name)
        bind_id = self._ivy.subscribe(_cb, req_regex)
        req_msg = PprzMessage('ground','{}_REQ'.format(msg_name))
        if params is not None:
            req_msg.set_values(params)
        #FIXME we shouldn't use directly Ivy, but pprzlink python API is not supporting the request id for now
        IvySendMsg('pprz_connect {} {} {}'.format(req_id, req_msg.name, req_msg.payload_to_ivy_string()))
        #self._ivy.send(req_msg)

    def get_aircrafts(self):
        """
        request all aircrafts IDs from a runing server
        and new aircraft when they appear
        """
        def aircrafts_cb(sender, msg):
            ac_list = msg['ac_list']
            for ac_id in ac_list:
                self.get_config(ac_id)
            #ac_list = [int(a) for a in msg['ac_list'].split(',') if a]
            if self.verbose:
                print("aircrafts: {}".format(ac_list))
        self._message_req("AIRCRAFTS", aircrafts_cb)

        def new_ac_cb(sender, msg):
            ac_id = msg['ac_id']
            self.get_config(ac_id)
            if self.verbose:
                print("new aircraft: {}".format(ac_id))
        self._ivy.subscribe(new_ac_cb,PprzMessage('ground','NEW_AIRCRAFT'))

    def get_config(self, ac_id):
        """
        Requsest a config from the server for a given ID

        :param ac_id: aircraft ID
        :type ac_id: str
        """
        def conf_cb(sender, msg):
            conf = PprzConfig(msg['ac_id'], msg['ac_name'], msg['airframe'],
                    msg['flight_plan'], msg['settings'], msg['radio'],
                    msg['default_gui_color'])
            self._conf_list_by_name[conf.name] = conf
            self._conf_list_by_id[int(conf.id)] = conf
            if self._notify is not None:
                self._notify(conf) # user defined general callback
            if self.verbose:
                print(conf)
        self._message_req("CONFIG", conf_cb, [ac_id])


if __name__ == '__main__':
    """
    test program
    """
    try:
        connect = PprzConnect(verbose=True)
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("Stopping on request")

    connect.shutdown()

