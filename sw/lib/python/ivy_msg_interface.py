from __future__ import absolute_import, division, print_function

from ivy.std_api import *
from ivy.ivy import IvyIllegalStateError
import logging
import os
import sys
import re

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprz_msg.message import PprzMessage
from pprz_msg import messages_xml_map


class IvyMessagesInterface(object):
    def __init__(self, callback=None, init=True, verbose=False, bind_regex='(.*)'):
        self.callback = callback
        self.ivy_id = 0
        self.verbose = verbose
        # make sure all messages are parsed before we start creating them in callbacks
        messages_xml_map.parse_messages()
        self.init_ivy(init, bind_regex)

    def stop(self):
        IvyUnBindMsg(self.ivy_id)

    def shutdown(self):
        self.stop()
        try:
            IvyStop()
        except IvyIllegalStateError as e:
            print(e)

    def __init__del__(self):
        try:
            IvyUnBindMsg(self.ivy_id)
        except:
            pass

    def init_ivy(self, init=True, bind_regex='(.*)'):
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
        # return if no callback is set
        if self.callback is None:
            return

        # first split on array delimiters
        l = re.split('([|\"][^|\"]*[|\"])', larg[0])
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

        # check which message class it is
        msg_name = data[1]
        msg_class, msg_name = messages_xml_map.find_msg_by_name(msg_name)
        if msg_class is None:
            print("Ignoring unknown message " + larg[0])
            return
        # pass non-telemetry messages with ac_id 0
        if msg_class == "telemetry":
            try:
                ac_id = int(data[0])
            except ValueError:
                print("ignoring message " + larg[0])
                sys.stdout.flush()
        else:
            ac_id = 0
        values = list(filter(None, data[2:]))
        msg = PprzMessage(msg_class, msg_name)
        msg.set_values(values)
        self.callback(ac_id, msg)

    def send_raw_datalink(self, msg):
        if not isinstance(msg, PprzMessage):
            print("Can only send PprzMessage")
            return
        if "datalink" not in msg.msg_class:
            print("Message to embed in RAW_DATALINK needs to be of 'datalink' class")
            return
        raw = PprzMessage("ground", "RAW_DATALINK")
        raw['ac_id'] = msg['ac_id']
        raw['message'] = msg.to_csv()
        self.send(raw)

    def send(self, msg, ac_id=None):
        if isinstance(msg, PprzMessage):
            if "telemetry" in msg.msg_class:
                if ac_id is None:
                    print("ac_id needed to send telemetry message.")
                else:
                    IvySendMsg("%d %s %s" % (ac_id, msg.name, msg.payload_to_ivy_string()))
            else:
                IvySendMsg("%s %s %s" % (msg.msg_class, msg.name, msg.payload_to_ivy_string()))
        else:
            IvySendMsg(msg)
