#!/usr/bin/python

import StringIO
import os
import time
import zmq
import random
from datetime import datetime
import socket
import struct

dest_addr = '192.168.25.47'
dest_port = 1234
local_port = 4243

msock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
msock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msock.bind(("", int(local_port)))
# mreq = struct.pack("4sl", socket.inet_aton(telemip), socket.INADDR_ANY)
# msock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# initialize a socket, think of it as a cable
# SOCK_DGRAM specifies that this is UDP
destsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

while( 1 ):
    data = None
    try:
      data, addr = msock.recvfrom(1024)

      format = 'B' * (len(data))
      strdata = struct.unpack( format, data )

      print len( strdata ), ":", strdata

      # send the command
      destsock.sendto( data, (dest_addr, dest_port) )

    except socket.error, e:
      print 'Exception', e

