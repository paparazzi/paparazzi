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
        self.InitIvy()
        self.ac_ids = ac_ids

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

    def SendSetting(self, setting_index):
        for ac_id in self.ac_ids:
            IvySendMsg("dl DL_SETTING %s %s %s" % (ac_id, setting_index, self.lookup[setting_index].value))

    def OnClose(self):
        IvyStop()


def main():
    ac_id = [ 11 ]
    ivy_interface = IvySettingsInterface(ac_id)

if __name__ == '__main__':
    main()
