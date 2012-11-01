#!/usr/bin/python

import StringIO
import os
import time
import zmq
import random
from datetime import datetime
import socket
import struct

telemip = '224.1.1.11'
telemport = '1234'
dest_addr = '192.168.25.4'
dest_port = 4242

if (( telemip == None ) or ( len(telemip) == 0 )):
    print "environment variable TELEM_IP needs a value!"
    exit( -1 )

if (( telemport == None ) or ( len(telemport) == 0 )):
    print "environment variable TELEM_PORT needs a value!"
    exit( -1 )

msock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
msock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msock.bind(("", int(telemport)))
mreq = struct.pack("4sl", socket.inet_aton(telemip), socket.INADDR_ANY)
msock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

def unshiftLong( data, idx ):
    result = data[ idx ] | (data[ idx+1 ] << 8) | (data[ idx+2 ] << 16) | (data[ idx+3 ] << 24)
    if(result & 0x80000000):
        result = -0x100000000 + result
    return result

def unshiftInt( data, idx ):
    result = data[ idx ] | (data[ idx+1 ] << 8)
    if(result & 0x8000):
        result = -0x10000 + result
    return result


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
      destsock.sendto( data, (dest_addr, dest_port) )

    except socket.error, e:
      print 'Exception', e



