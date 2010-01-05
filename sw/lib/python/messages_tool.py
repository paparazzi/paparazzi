import messages_xml_map
from ivy.std_api import *
import logging
import time
import os

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

  def __del__(self):
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
    data = larg[0].split(' ')
    try:
      ac_id = int(data[0])
      name = data[1]
      values = data[2:]
      self.callback(ac_id, name, values)
    except ValueError:
      pass
    except:
      raise

def test():
  message = Message("WHIRLY")
  print message
  print message.field_names
  
if __name__ == '__main__':
  test()
