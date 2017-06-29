#!/usr/bin/env python

import os
import sys

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../../../../')))
sys.path.append(PAPARAZZI_HOME + "/var/lib/python")

from ivy.std_api import *
import threading
import time

import pprzlink.udp
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message

DEFAULT_BROADCAST= "127.255.255.255"
REPEAT="Repeat"
IP_BCAST="IPBcast"

PING_PERIOD = 5.0
STATUS_PERIOD = 1.0

class DownLinkStatus():
    def __init__(self, ac_id, address):
        self.ac_id = ac_id
        self.address = address
        self.rx_bytes = 0
        self.rx_msgs = 0
        self.tx_msgs = 0
        self.run_time = 0
        self.last_rx_bytes = 0
        self.last_rx_msgs = 0
        self.last_msg_time = 0
        self.last_ping_time = 0
        self.last_pong_time = 0

class UDPLink:
    def __init__(self,opts):
        messages_xml_map.parse_messages()
        self.run_event = threading.Event()
        self.uplink_port = opts.uplink_port
        self.downlink_port = opts.downlink_port
        self.udp = pprzlink.udp.UdpMessagesInterface(self.proccess_downlink_msg, False, self.uplink_port, self.downlink_port)
        self.ivy = pprzlink.ivy.IvyMessagesInterface("UDPLink", True, False, opts.bus)
        self.ac_downlink_status = {}
        self.rx_err = 0
        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
        self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
        self.bcast_method = opts.broadcast_method
        self.bcast_addr = opts.broadcast_address

    def updateStatus(self, ac_id, length, address, isPong):
        if not self.ac_downlink_status.has_key(ac_id):
            self.ac_downlink_status[ac_id] = DownLinkStatus(ac_id, address)
        self.ac_downlink_status[ac_id].rx_msgs += 1
        self.ac_downlink_status[ac_id].rx_bytes += length
        self.ac_downlink_status[ac_id].last_msg_time = time.time()
        if isPong:
            self.ac_downlink_status[ac_id].last_pong_time = time.time() - self.ac_downlink_status[ac_id].last_ping_time

    def proccess_downlink_msg(self,sender,address,msg,length,receiver_id=None, component_id=None):
        if self.run_event.is_set():
            # print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
            self.ivy.send(msg,sender,receiver_id,component_id)
            self.updateStatus(sender, length, address,msg.name == "PONG")

    def proccess_uplink_msg(self,ac_id,msg):
        # print ('New IVY message to %s : %s' % (ac_id,msg))
        if msg.broadcasted:
            if self.bcast_method==IP_BCAST:
                self.udp.send(msg,0,(self.bcast_addr,self.uplink_port))
            else:
                for dest in self.ac_downlink_status.keys():
                    self.udp.send(msg, 0, (self.ac_downlink_status[dest].address[0], self.uplink_port))
                    self.ac_downlink_status[dest].tx_msgs += 1
        else:
            if isinstance(ac_id,str):
                ac_id = int(ac_id)
            # Only send message if the ac is known
            if self.ac_downlink_status.has_key(ac_id):
                self.udp.send(msg,0,(self.ac_downlink_status[ac_id].address[0],self.uplink_port),ac_id)
                self.ac_downlink_status[ac_id].tx_msgs+=1
            else:
                print ('Message for unknown ac %d' % ac_id)

    def initial_ivy_binds(self):
        # Subscribe to all datalink messages
        messages_datalink = messages_xml_map.get_msgs("datalink")
        for msg in messages_datalink:
            self.ivy.subscribe(self.proccess_uplink_msg, message.PprzMessage("datalink", msg))

    def run(self):
        print ('Starting UDPLink for protocol version %s' % (messages_xml_map.PROTOCOL_VERSION))
        self.udp.start()
        self.ivy.start()

        self.run_event.set()

        self.status_timer.start()
        self.ping_timer.start()

        self.initial_ivy_binds()

        try:
            while True:
                time.sleep(.5)
        except KeyboardInterrupt:
            print ("Stopping UDPLink.")
            self.status_timer.cancel()
            self.ping_timer.cancel()
            self.run_event.clear()
            # t.join()
            self.udp.stop()
            self.ivy.stop()
            self.udp.join()

    def sendPing(self):
        for (ac_id, value) in self.ac_downlink_status.items():
            if messages_xml_map.PROTOCOL_VERSION=="2.0":
                # For pprzlink V2.0 set the receiver id
                self.udp.send(message.PprzMessage('datalink','PING'), 0, self.ac_downlink_status[int(ac_id)].address, ac_id)
            else:
                self.udp.send(message.PprzMessage('datalink','PING'),0,self.ac_downlink_status[int(ac_id)].address)
            value.last_ping_time = time.time()
        self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
        self.ping_timer.start()

    def sendStatus(self):
        for (key, value) in self.ac_downlink_status.items():
            self.ivy.send("link LINK_REPORT %i %i %i %i %i %i %i %i %i %i %i" % (
                value.ac_id,
                -1,
                value.run_time,
                time.time() - value.last_msg_time,
                value.rx_bytes,
                value.rx_msgs,
                self.rx_err,
                value.rx_bytes - value.last_rx_bytes,
                value.rx_msgs - value.last_rx_msgs,
                value.tx_msgs,
                1000 * value.last_pong_time))
            value.last_rx_bytes = value.rx_bytes
            value.last_rx_msgs = value.rx_msgs
            value.run_time = value.run_time + 1

        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
        self.status_timer.start()

if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description="UDP link for paparazzi (python version)")
    parser.add_argument("--broadcast_method", choices=[IP_BCAST,REPEAT] , default=IP_BCAST, help="Broadcast method - repeating to all known aircraft or sending to IP broadcast address. [default: %(default)s]")
    parser.add_argument("--broadcast_address", default=DEFAULT_BROADCAST, help="IP address used for broadcast when broadcast method is IP_BCAST. [default: %(default)s]")
    parser.add_argument("--uplink_port", default=pprzlink.udp.UPLINK_PORT, help="Uplink UDP port. [default: %(default)s]")
    parser.add_argument("--downlink_port", default=pprzlink.udp.DOWNLINK_PORT, help="Downlink UDP port. [default: %(default)s]")
    parser.add_argument("--bus", default=pprzlink.ivy.IVY_BUS, help="Ivy bus. [default to system IVY bus]")
    args = parser.parse_args()

    link = UDPLink(args)
    link.run()
