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
      IvyInit("UdpLink", # application name for Ivy
			"READY",                 # ready message
			0,                  # main loop is local (ie. using IvyMainloop)
			lambda x,y: y,      # handler called on connection/deconnection
			lambda x,y: y       # handler called when a diemessage is received
			)

      # starting the bus
      logging.getLogger('Ivy').setLevel(logging.WARN)
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
      start_byte = ord(msg[msg_offset])

      if start_byte != 0x99:
        self.rx_err = self.rx_err + 1
        return

      msg_offset = msg_offset + 1
      ac_id = ord(msg[msg_offset])
      msg_offset = msg_offset + 1
      while msg_offset < len(msg):
        msg_id = ord(msg[msg_offset])
        msg_offset = msg_offset + 1
        start_byte2 = ord(msg[msg_offset])

        if start_byte2 != 0x66:
          self.rx_err = self.rx_err + 1
          return
        msg_offset = msg_offset + 1
        msg_name = messages_xml_map.message_dictionary_id_name[msg_id]
        msg_fields = messages_xml_map.message_dictionary_types[msg_id]

        ivy_msg = "%i %s" % (ac_id, msg_name)

        for field in msg_fields:
          if field == "float":
            value = struct.unpack('f', str(msg[msg_offset:msg_offset + 4]))[0]
            ivy_msg = "%s %f" % (ivy_msg, value)
            msg_offset = msg_offset + 4
          elif field == "uint8":
            value = struct.unpack('B', str(msg[msg_offset:msg_offset + 1]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 1
          elif field == "uint16":
            value = struct.unpack('H', str(msg[msg_offset:msg_offset + 2]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 2
          elif field == "uint32":
            value = struct.unpack('L', str(msg[msg_offset:msg_offset + 4]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 4
          elif field == "int8":
            value = struct.unpack('b', str(msg[msg_offset:msg_offset + 1]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 1
          elif field == "int16":
            value = struct.unpack('h', str(msg[msg_offset:msg_offset + 2]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 2
          elif field == "int32":
            value = struct.unpack('l', str(msg[msg_offset:msg_offset + 4]))[0]
            ivy_msg = "%s %i" % (ivy_msg, value)
            msg_offset = msg_offset + 4
          elif field == "uint8[]":
            value = struct.unpack('B', str(msg[msg_offset:msg_offset + 1]))[0]
            msg_offset = msg_offset + 1
            for count in range(0, value):
              array_value = struct.unpack('B', str(msg[msg_offset:msg_offset + 1]))[0]
              msg_offset = msg_offset + 1
              ivy_msg = "%s,%i" % (ivy_msg, array_value)
          else:
            print "Unknown field type %s" % field
  
        self.rx_msgs = self.rx_msgs + 1
        self.rx_bytes = self.rx_bytes + len(msg)
        IvySendMsg(ivy_msg)
        #print ivy_msg


    def Run(self):
      server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
      msg_count = 0
      server.bind(('0.0.0.0' , 51914))
      self.status_timer.start()
      while True:
        msg = server.recv(2048)
        msg_count = msg_count + 1
        self.ProcessPacket(msg)
        #if (msg_count % 120 == 0):
        # print msg_count

def main():
  udp_interface = IvyUdpLink()
  udp_interface.Run()

if __name__ == '__main__':
  main()
