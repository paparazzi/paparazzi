#!/usr/bin/env python
#-*- coding: utf-8 -*-

#Copyright 2014, Antoine Drouin

from __future__ import print_function

import logging, base64, socket
from gi.repository import GLib, GObject
import ivy.ivy as ivy
ivylogger = logging.getLogger('Ivy')
ivylogger.setLevel(logging.CRITICAL)

import phoenix.messages
import phoenix.pprz_transport

from os import path, getenv

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
home_dir = getenv("PAPARAZZI_HOME", path.normpath(path.join(
    path.dirname(path.abspath(__file__)), '../../../')))

default_ivybus = '127.255.255.255:2010'

class Server(ivy.IvyServer):
    def __init__(self, bus, tcp_port=4242):
        ivy.IvyServer.__init__(self, 'TCP_aircraft_server', usesDaemons=True)

        self.nb_msgs = 0
        self.nb_bytes = 0

        cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cs.bind(('', tcp_port))
        cs.listen(1)
        GObject.io_add_watch(cs, GObject.IO_IN, self.handle_conn)
        print("server listening on {:d}".format(tcp_port))

        self.transp = phoenix.pprz_transport.Transport(check_crc=False, debug=False)
        self.protocol = phoenix.messages.Protocol(path=path.join(home_dir, "var/messages.xml"), debug=True)
        self.start(bus)

        GObject.timeout_add(500, self.periodic, priority=GObject.PRIORITY_HIGH)

    def handle_conn(self, sock, cond):
        conn, addr = sock.accept()
        print("Connection from {}".format(addr))
        GObject.io_add_watch(conn, GObject.IO_IN, self.handle_data)
        return True

    def handle_data(self, sock, cond):
        buf = sock.recv(4096)
        if not len(buf):
            print("Connection closed.")
            return False
        else:
            #print phoenix.hex_of_bin(buf)
            msgs = self.transp.parse_many(buf)
            for hdr, payload in msgs:
                msg = self.protocol.get_message_by_id("telemetry", hdr.msgid)
                try:
                    ivy_str = '{} {} {}'.format(hdr.acid, msg.name, ' '.join([str(v) for v in msg.unpack_values(payload)]))
                    #print  ivy_str
                    self.send_msg(ivy_str)
                    self.nb_msgs += 1
                    self.nb_bytes += len(payload)
                except:
                    print('FAILED', msg.name)
        return True

    def periodic(self):
        print('msgs {} ({} bytes)'.format(self.nb_msgs, self.nb_bytes))
        return True

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    server = Server(default_ivybus)
    GLib.MainLoop().run()
