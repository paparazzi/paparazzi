#!/usr/bin/python

import os
import socket
import struct
import sys
from optparse import OptionParser

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/lib/python")

parser = OptionParser()
parser.add_option("-m", "--multicast_ip", dest="multicast_ip", help="Multicast IP where telemetry messages go to", default="224.1.1.11")
parser.add_option("-p", "--multicast_port", dest="multicast_port", default='1234', help="Multicast port where messages get sent")
parser.add_option("-d", "--dest_ip", dest="dest_addr", default='192.168.25.4', help="Multicast ip where messages get sent")
parser.add_option("-l", "--dest_port", dest="dest_port", default=4242, help="Local port to send telemetry messages to")

(options, args) = parser.parse_args()

msock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
msock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msock.bind(("", int(options.multicast_port)))
mreq = struct.pack("4sl", socket.inet_aton(options.multicast_ip), socket.INADDR_ANY)
msock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# initialize a socket, think of it as a cable
# SOCK_DGRAM specifies that this is UDP
destsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

while( 1 ):
    data = None
    try:
      data, addr = msock.recvfrom(1024)

      format = 'B' * (len(data))
      strdata = struct.unpack( format, data )

      #print len( strdata ), ":", strdata

      # send the command
      destsock.sendto( data, (options.dest_addr, options.dest_port) )

    except socket.error, e:
      print 'Exception', e

