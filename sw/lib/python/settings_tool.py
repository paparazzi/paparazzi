#!/usr/bin/env python

from __future__ import absolute_import, print_function

from ivy.std_api import *
import logging
import sys
import pprz_env

sys.path.append(pprz_env.PAPARAZZI_SRC + "/sw/lib/python")

from settings_xml_parse import PaparazziACSettings

_SHOW_IVY_MSGS_ = False


class IvySettingsInterface(PaparazziACSettings):
    def __init__(self, ac_ids):
        PaparazziACSettings.__init__(self, ac_ids[0])
        self.update_callback = None
        self.state_callback = None
        self.ac_ids = ac_ids
        # False while the aircraft is removed by the server or runs a build
        # with other settings: the setting indexes are then not valid
        self.valid = True
        self.InitIvy()

    def ProcessMessage(self, message_values, fromRemote):
        # Extract aircraft id from message and ignore if not matching
        msg_ac_id = int(message_values[0])
        if (msg_ac_id != self.ac_ids[0]):
            return

        # Extract setting value
        setting_index = int(message_values[1])
        setting_value = message_values[2]

        if _SHOW_IVY_MSGS_:
            print("Got setting with index: %s   value %s   " % (setting_index, setting_value))

        if not self.valid or setting_index >= len(self.lookup):
            return

        # Store value from message
        self.lookup[setting_index].value = setting_value

        # Callback (if present)
        if self.update_callback != None:
            self.update_callback(setting_index, setting_value, fromRemote)

    # Called for DL_VALUE (from aircraft)
    def OnValueMsg(self, agent, *larg):
        # Extract field values
        message_values = list(filter(None, larg[0].split(' ')))
        message_values = message_values[0:1] + message_values[2:]
        self.ProcessMessage(message_values, True)

    # Called for DL_SETTING (from ground)
    def OnSettingMsg(self, agent, *larg):
        # Extract field values
        message_values = list(filter(None, larg[0].split(' ')))
        self.ProcessMessage(message_values, False)

    def RegisterCallback(self, callback_function):
        self.update_callback = callback_function

    # callback_function(valid, message): called when sending settings gets
    # blocked (valid is False) or allowed again (valid is True)
    def RegisterStateCallback(self, callback_function):
        self.state_callback = callback_function

    def _SetValid(self, valid, message):
        self.valid = valid
        print(message)
        if self.state_callback is not None:
            self.state_callback(valid, message)

    def _IsOurAircraft(self, ac_id):
        return ac_id in [str(a) for a in self.ac_ids]

    # Called for AIRCRAFT_DIE (from the server, e.g. the aircraft runs another build)
    def OnAircraftDie(self, agent, ac_id):
        if self._IsOurAircraft(ac_id) and self.valid:
            self._SetValid(False, "Aircraft %s removed by the server (other build?), "
                           "settings are not sent until it is back" % ac_id)

    # Called for NEW_AIRCRAFT (from the server): check that the settings did not change
    def OnNewAircraft(self, agent, ac_id):
        if not self._IsOurAircraft(ac_id) or self.valid:
            return
        try:
            new_settings = PaparazziACSettings(self.ac_ids[0])
        except SystemExit:
            return
        old = [(s.shortname, s.var, s.min_value, s.max_value, s.step) for s in self.lookup]
        new = [(s.shortname, s.var, s.min_value, s.max_value, s.step) for s in new_settings.lookup]
        if old == new:
            self._SetValid(True, "Aircraft %s is back with the same settings, sending them again" % ac_id)
        else:
            print("Aircraft %s runs a build with other settings, restart this tool to use them" % ac_id)
            if self.state_callback is not None:
                self.state_callback(False, "Aircraft %s has other settings, restart this tool" % ac_id)

    def InitIvy(self):
        # initialising the bus
        IvyInit("settings_app", # application name for Ivy
                "",                 # ready message
                0,                  # main loop is local (ie. using IvyMainloop)
                lambda x,y: y,      # handler called on connection/deconnection
                lambda x,y: y       # handler called when a diemessage is received
                )

        # starting the bus
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyStart(pprz_env.IVY_BUS)
        IvyBindMsg(self.OnValueMsg, "(^.* DL_VALUE .*)")
        IvyBindMsg(self.OnSettingMsg, "dl DL_SETTING (.*)")
        IvyBindMsg(self.OnAircraftDie, "^ground AIRCRAFT_DIE (\\S+)")
        IvyBindMsg(self.OnNewAircraft, "^ground NEW_AIRCRAFT (\\S+)")

    def SendSetting(self, setting_index):
        if not self.valid:
            print("Setting %s not sent: the settings of the aircraft are not valid anymore" %
                  self.lookup[setting_index].shortname)
            return
        for ac_id in self.ac_ids:
            IvySendMsg("dl DL_SETTING %s %s %s" % (ac_id, setting_index, self.lookup[setting_index].value))

    def OnClose(self):
        IvyStop()


def main():
    ac_id = [ 11 ]
    ivy_interface = IvySettingsInterface(ac_id)

if __name__ == '__main__':
    main()
