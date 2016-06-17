#!/usr/bin/env python

from ivy.std_api import *
import socket
import struct
import os
import logging
import sys
import threading
import time

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

import pprz_env
from pprzlink import messages_xml_map

PING_PERIOD = 5.0
STATUS_PERIOD = 1.0

STX = 0x99
STX_TS = 0x98

DATALINK_PORT = 4243
DOWNLINK_PORT = 4242


class DownLinkStatus():
    def __init__(self, ac_id, address):
        self.ac_id = ac_id
        self.address = address
        self.rx_bytes = 0
        self.rx_msgs = 0
        self.run_time = 0
        self.last_rx_bytes = 0
        self.last_rx_msgs = 0
        self.last_ping_time = 0
        self.last_pong_time = 0


class IvyUdpLink():
    def __init__(self):
        self.InitIvy()
        self.ivy_id = 0
        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
        self.ping_timer = threading.Timer(STATUS_PERIOD, self.sendPing)
        self.ac_downlink_status = {}
        self.rx_err = 0

        messages_xml_map.parse_messages()
        self.data_types = {'float': ['f', 4],
                           'uint8': ['B', 1],
                           'uint16': ['H', 2],
                           'uint32': ['L', 4],
                           'int8': ['b', 1],
                           'int16': ['h', 2],
                           'int32': ['l', 4]
                           }

    def __del__(self):
        self.stop()

    def stop(self):
        self.status_timer.cancel()
        self.ping_timer.cancel()
        IvyUnBindMsg(self.ivy_id)
        IvyStop()

    def Unpack(self, data_fields, type, start, length):
        return struct.unpack(type, "".join(data_fields[start:start + length]))[0]

    def InitIvy(self):
        # initialising the bus
        IvyInit("Link",  # application name for Ivy
                "READY",  # ready message
                0,  # main loop is local (ie. using IvyMainloop)
                lambda x, y: y,  # handler called on connection/deconnection
                lambda x, y: y  # handler called when a diemessage is received
                )

        # starting the bus
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyStart(pprz_env.IVY_BUS)
        self.ivy_id = IvyBindMsg(self.OnSettingMsg, "(^.* SETTING .*)")

    def calculate_checksum(self, msg):
        ck_a = 0
        ck_b = 0
        # start char not included in checksum for pprz protocol
        for c in msg[1:]:
            ck_a = (ck_a + ord(c)) % 256
            ck_b = (ck_b + ck_a) % 256
        return (ck_a, ck_b)

    def buildPprzMsg(self, msg_id, *args):
        stx = STX
        length = 6
        sender = 0
        msg_fields = messages_xml_map.message_dictionary_types["datalink"][msg_id]
        struct_string = "=BBBB"
        typed_args = []
        idx = 0
        for msg_type in msg_fields:
            struct_string += self.data_types[msg_type][0]
            length += self.data_types[msg_type][1]
            if (msg_type == "float"):
                typed_args.append(float(args[idx]))
            else:
                typed_args.append(int(args[idx]))
            idx += 1
        msg = struct.pack(struct_string, stx, length, sender, msg_id, *typed_args)
        (ck_a, ck_b) = self.calculate_checksum(msg)
        msg = msg + struct.pack('=BB', ck_a, ck_b)
        return msg

    def OnSettingMsg(self, agent, *larg):
        list = larg[0].split(' ')
        sender = list[0]
        msg_name = list[1]
        ac_id = list[3]
        args = list[2:]
        msg_id = messages_xml_map.message_dictionary_name_id["datalink"][msg_name]
        if self.ac_downlink_status.has_key(int(ac_id)):
            msgbuf = self.buildPprzMsg(msg_id, *args)
            address = (self.ac_downlink_status[int(ac_id)].address[0], DATALINK_PORT)
            self.server.sendto(msgbuf, address)

    def sendPing(self):
        for (ac_id, value) in self.ac_downlink_status.items():
            msg_id = messages_xml_map.message_dictionary_name_id["datalink"]["PING"]
            msgbuf = self.buildPprzMsg(msg_id)
            address = (self.ac_downlink_status[int(ac_id)].address[0], DATALINK_PORT)
            self.server.sendto(msgbuf, address)
            value.last_ping_time = time.clock()

        self.ping_timer = threading.Timer(STATUS_PERIOD, self.sendPing)
        self.ping_timer.start()

    def sendStatus(self):
        for (key, value) in self.ac_downlink_status.items():
            IvySendMsg("%i DOWNLINK_STATUS %i %i %i %i %i %i %i" % (
                value.ac_id,
                value.run_time,
                value.rx_bytes,
                value.rx_msgs,
                self.rx_err,
                value.rx_bytes - value.last_rx_bytes,
                value.rx_msgs - value.last_rx_msgs,
                1000 * value.last_pong_time))
            value.last_rx_bytes = value.rx_bytes
            value.last_rx_msgs = value.rx_msgs
            value.run_time = value.run_time + 1

        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
        self.status_timer.start()

    def updateStatus(self, ac_id, length, address, isPong):
        if not self.ac_downlink_status.has_key(ac_id):
            self.ac_downlink_status[ac_id] = DownLinkStatus(ac_id, address)

        self.ac_downlink_status[ac_id].rx_msgs += 1
        self.ac_downlink_status[ac_id].rx_bytes += length
        if isPong:
            self.ac_downlink_status[ac_id].last_pong_time = time.clock() - self.ac_downlink_status[ac_id].last_ping_time

    def ProcessPacket(self, msg, address):
        if len(msg) < 4:
            self.rx_err = self.rx_err + 1
            return

        msg_offset = 0
        while msg_offset < len(msg):
            start_byte = ord(msg[msg_offset])
            msg_start_idx = msg_offset
            msg_offset = msg_offset + 1

            if start_byte != STX and start_byte != STX_TS:
                self.rx_err = self.rx_err + 1
                return

            msg_length = ord(msg[msg_offset])
            msg_offset = msg_offset + 1

            if (start_byte == STX_TS):
                timestamp = int(self.Unpack(msg, 'L', msg_offset, 4))
                msg_offset = msg_offset + 4

            ac_id = ord(msg[msg_offset])
            msg_offset = msg_offset + 1

            msg_id = ord(msg[msg_offset])
            msg_offset = msg_offset + 1

            msg_name = messages_xml_map.message_dictionary_id_name["telemetry"][msg_id]
            msg_fields = messages_xml_map.message_dictionary_types["telemetry"][msg_id]

            ivy_msg = "%i %s " % (ac_id, msg_name)

            for field in msg_fields:
                if field[-2:] == "[]":
                    baseType = field[:-2]
                    array_length = int(self.Unpack(msg, 'B', msg_offset, 1))
                    msg_offset = msg_offset + 1
                    for count in range(0, array_length):
                        array_value = str(
                            self.Unpack(msg, self.data_types[baseType][0], msg_offset, self.data_types[baseType][1]))
                        msg_offset = msg_offset + self.data_types[baseType][1]
                        if (count == array_length - 1):
                            ivy_msg += array_value + " "
                        else:
                            ivy_msg += array_value + ","
                else:
                    ivy_msg += str(
                        self.Unpack(msg, self.data_types[field][0], msg_offset, self.data_types[field][1])) + " "
                    msg_offset = msg_offset + self.data_types[field][1]

                if (msg_offset > len(msg)):
                    print "finished without parsing %s" % field
                    break

            (ck_a, ck_b) = self.calculate_checksum(msg[msg_start_idx:msg_offset])
            msg_ck_a = int(self.Unpack(msg, 'B', msg_offset, 1))
            msg_offset += 1
            msg_ck_b = int(self.Unpack(msg, 'B', msg_offset, 1))
            msg_offset += 1

            # check for valid checksum
            if (ck_a, ck_b) == (msg_ck_a, msg_ck_b):
                self.updateStatus(ac_id, msg_length, address,
                                  msg_id == messages_xml_map.message_dictionary_name_id["telemetry"]["PONG"])

                # strip off trailing whitespace
                ivy_msg = ivy_msg[:-1]
                IvySendMsg(ivy_msg)

    def Run(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.bind(('0.0.0.0', DOWNLINK_PORT))
        self.status_timer.start()
        self.ping_timer.start()
        try:
            while True:
                (msg, address) = self.server.recvfrom(2048)
                self.ProcessPacket(msg, address)
        except KeyboardInterrupt:
            print("Stopping server on request")


def main():
    udp_interface = IvyUdpLink()
    udp_interface.Run()
    udp_interface.stop()


if __name__ == '__main__':
    main()
