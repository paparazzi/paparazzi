from __future__ import absolute_import, print_function

from ivy.std_api import *
import logging
import os
import sys
import re


class IvyMessagesInterface():
    def __init__(self, callback, init=True, verbose=True, bind_regex="(.*)"):
        self.callback = callback
        self.ivy_id = 0
        self.verbose = verbose
        self.init_ivy(init, bind_regex)

    def stop(self):
        IvyUnBindMsg(self.ivy_id)

    def shutdown(self):
        self.stop()
        IvyStop()

    def __init__del__(self):
        try:
            IvyUnBindMsg(self.ivy_id)
        except:
            pass

    def init_ivy(self, init, bind_regex):
        if init:
            IvyInit("Messages %i" % os.getpid(), "READY", 0, lambda x,y: y, lambda x,y: y)
            logging.getLogger('Ivy').setLevel(logging.WARN)
            IvyStart("")
        self.ivy_id = IvyBindMsg(self.on_ivy_msg, bind_regex)

    def on_ivy_msg(self, agent, *larg):
        """ Split ivy message up into the separate parts
        Basically parts/args in string are separated by space, but char array can also contain a space:
        |f,o,o, ,b,a,r| in old format or "foo bar" in new format
        """
        # first split on array delimiters
        l = re.split('([|\"][^|]*[|\"])', larg[0])
        # strip spaces and filter out emtpy strings
        l = [str.strip(s) for s in l if str.strip(s) is not '']
        data = []
        for s in l:
            # split non-array strings further up
            if '|' not in s and '"' not in s:
                data += s.split(' ')
            else:
                data.append(s)
        # ignore ivy message with less than 3 elements
        if len(data) < 3:
            return

        if data[0] in ["ground", "ground_dl", "dl"]:
            msg_class = data[0]
            msg_name = data[1]
            ac_id = int(data[2])
            values = list(filter(None, data[3:]))
        elif data[0] == "sim":
            return
        else:
            try:
                ac_id = int(data[0])
            except ValueError:
                if self.verbose:
                    print("ignoring message %s" % data[1])
                    sys.stdout.flush()
                return
            msg_class = "telemetry"
            msg_name = data[1]
            values = list(filter(None, data[2:]))
        self.callback(msg_class, msg_name, ac_id, values)
