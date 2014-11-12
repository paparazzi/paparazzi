from __future__ import absolute_import, print_function

import messages_xml_map
from ivy.std_api import *
import logging
import time
import os
import re

class Message:
    def __init__(self, class_name, name):
        messages_xml_map.ParseMessages()
        self.field_value = []
        self.field_names = messages_xml_map.message_dictionary[class_name][name]
        self.field_controls = []
        self.index = None
        self.last_seen = time.clock()
        self.name = name

class Aircraft:
    def __init__(self, id):
        self.ac_id = id
        self.messages = {}
        self.messages_book = None

class IvyMessagesInterface():
    def __init__(self, callback, initIvy = True):
        self.callback = callback
        self.ivy_id = 0
        self.InitIvy(initIvy)

    def Stop(self):
        IvyUnBindMsg(self.ivy_id)

    def Shutdown(self):
        self.Stop()
        IvyStop()

    def __init__del__(self):
        try:
            IvyUnBindMsg(self.ivy_id)
        except:
            pass

    def InitIvy(self, initIvy):
        if initIvy:
            IvyInit("Messages %i" % os.getpid(), "READY", 0, lambda x,y: y, lambda x,y: y)
            logging.getLogger('Ivy').setLevel(logging.WARN)
            IvyStart("")
        self.ivy_id = IvyBindMsg(self.OnIvyMsg, "(.*)")

    def OnIvyMsg(self, agent, *larg):
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
        try:
            ac_id = int(data[0])
            name = data[1]
            values = list(filter(None, data[2:]))
            self.callback(ac_id, name, values)
        except ValueError:
            pass
        except:
            raise

def test():
    message = Message("WHIRLY")
    print(message)
    print(message.field_names)

if __name__ == '__main__':
    test()
