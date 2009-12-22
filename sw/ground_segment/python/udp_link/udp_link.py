#!/usr/bin/env python

from ivy.std_api import *
import socket
import struct
import os
import logging
import sys
import threading

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/lib/python")

import messages_xml_map

class IvyUdpLink():
    def __init__(self):
      self.InitIvy()
      self.status_timer = threading.Timer(1.0, self.sendStatus)
      self.run_time = 0
      self.rx_bytes = 0
      self.rx_msgs = 0
      self.last_rx_bytes = 0
      self.last_rx_msgs = 0
      self.rx_err = 0

      messages_xml_map.ParseMessages()
      self.data_types = { 'float' : ['f', 4],
                          'uint8' : ['B', 1],
                          'uint16' : ['H', 2],
                          'uint32' : ['L', 4],
                          'int8' : ['b', 1],
                          'int16' : ['h', 2],
                          'int32' : ['l', 4]
                         }

    def Unpack(self, data_fields, type, start, length):
      return struct.unpack(type, "".join(data_fields[start:start + length]))[0]

    def ProcessMessage(self, message_values, fromRemote):
      # Extract aircraft id from message and ignore if not matching
      msg_ac_id = int(message_values[0])
      if (msg_ac_id != self.ac_ids[0]):
        return

      # Extract setting value
      setting_index = int(message_values[1])
      setting_value = message_values[2]

    # Called for DL_VALUE (from aircraft)
    def OnValueMsg(self, agent, *larg):
      # Extract field values
      message_values = larg[0].split(' ')
      message_values = message_values[0:1] + message_values[2:]
      self.ProcessMessage(message_values, True)

    def InitIvy(self):
      # initialising the bus
      IvyInit("Link", # application name for Ivy
			"READY",                 # ready message
			0,                  # main loop is local (ie. using IvyMainloop)
			lambda x,y: y,      # handler called on connection/deconnection
			lambda x,y: y       # handler called when a diemessage is received
			)

      # starting the bus
      logging.getLogger('Ivy').setLevel(logging.DEBUG)
      IvyStart("")
      #IvyBindMsg(self.OnValueMsg, "(^.* DL_VALUE .*)")

    def sendStatus(self):
      self.run_time = self.run_time + 1
      IvySendMsg("11 DOWNLINK_STATUS %i %i %i %i %i %i %i" % (
        self.run_time,
        self.rx_bytes - self.last_rx_bytes,
        self.rx_msgs - self.last_rx_msgs,
        self.rx_err,
        self.rx_bytes,
        self.rx_msgs,
        0 ))
      self.last_rx_bytes = self.rx_bytes
      self.last_rx_msgs = self.rx_msgs

      self.status_timer = threading.Timer(1.0, self.sendStatus)
      self.status_timer.start()

    def ProcessPacket(self, msg):
      if len(msg) < 4:
        self.rx_err = self.rx_err + 1
        return

      msg_offset = 0
      while msg_offset < len(msg):
	start_byte = ord(msg[msg_offset])
	msg_offset = msg_offset + 1

	if start_byte != 0x99:
	  self.rx_err = self.rx_err + 1
	  return

	msg_length = ord(msg[msg_offset])
	msg_offset = msg_offset + 1

	#timestamp = int(self.Unpack(msg, 'L', msg_offset, 4))
	#msg_offset = msg_offset + 4

	ac_id = ord(msg[msg_offset])
	msg_offset = msg_offset + 1

        msg_id = ord(msg[msg_offset])
        msg_offset = msg_offset + 1
      
        msg_name = messages_xml_map.message_dictionary_id_name[msg_id]
        msg_fields = messages_xml_map.message_dictionary_types[msg_id]

        ivy_msg = "%i %s " % (ac_id, msg_name)

        for field in msg_fields:
	  if field[-2:] == "[]":
	    baseType = field[:-2]
	    array_length = int(self.Unpack(msg, 'B', msg_offset, 1))
	    msg_offset = msg_offset + 1
	    for count in range(0, array_length):
	      array_value = str(self.Unpack(msg, self.data_types[baseType][0], msg_offset, self.data_types[baseType][1]))
	      msg_offset = msg_offset + self.data_types[baseType][1]
	      if (count == array_length - 1):
		ivy_msg += array_value + " "
	      else:
		ivy_msg += array_value + ","
	  else:
	    ivy_msg += str(self.Unpack(msg, self.data_types[field][0], msg_offset, self.data_types[field][1])) + " "
	    msg_offset = msg_offset + self.data_types[field][1]

	  if (msg_offset > len(msg)):
	    print "finished without parsing %s" % field
	    break

	msg_offset += 2 # munch munch checksum bytes
  
        self.rx_msgs = self.rx_msgs + 1
        self.rx_bytes = self.rx_bytes + len(msg)
	ivy_msg = ivy_msg[:-1]
        IvySendMsg(ivy_msg)


    def Run(self):
      server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
      msg_count = 0
      server.bind(('0.0.0.0' , 4242))
      self.status_timer.start()
      while True:
        msg = server.recv(2048)
        msg_count = msg_count + 1
        self.ProcessPacket(msg)

def main():
  udp_interface = IvyUdpLink()
  udp_interface.Run()

if __name__ == '__main__':
  main()
