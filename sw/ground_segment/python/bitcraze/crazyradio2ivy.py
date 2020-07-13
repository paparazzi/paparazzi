#!/usr/bin/env python3
"""
Bridge a Crazyflie connected to a Crazyradio to the Ivy software bus
with support of PPRZLINK messages

Requires 'pip3 install cflib'

As the ESB protocol works using PTX and PRX (Primary Transmitter/Reciever)
modes. Thus, data is only recieved as a response to a sent packet.
So, we need to constantly poll the receivers for bidirectional communication.

@author: Dennis Shtatnov (densht@gmail.com)
         Gautier Hattenberger for Paparazzi UAV support

"""
# import struct
from os import path, getenv
import logging
import sys
import time

from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp import RadioDriver

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.pprz_transport import PprzTransport
from pprzlink.message import PprzMessage
import pprzlink.messages_xml_map as messages_xml_map

CRTP_PORT_PPRZLINK = 9


# Only output errors from the logging framework
#logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.ERROR)


class RadioBridge:
    def __init__(self, link_uri, msg_class='telemetry', verbose=False):
        """ Initialize and run with the specified link_uri """
        self.verbose = verbose

        # Ivy interface and stream parser
        self._ivy = IvyMessagesInterface("cf2ivy")
        self._transport = PprzTransport(msg_class)

        # Create a Crazyradio
        self._driver = RadioDriver()
        self._driver.connect(link_uri, self._link_quality_cb, self._link_error_cb)

        if self.verbose:
            print('Connecting to %s' % link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        # Bind to all messages from ac_id
        def _forward_to_cf(ac_id, msg):
            try:
                data = self._transport.pack_pprz_msg(0, msg) # sender_id 0 = GCS
                for i in range(0, len(data), 30):
                    pk = CRTPPacket()
                    pk.port = CRTP_PORT_PPRZLINK
                    pk.data = data[i:(i+30)]
                    self._driver.send_packet(pk)
                if self.verbose:
                    print('Forward message', msg.name)
            except:
                if self.verbose:
                    print('Forward error for', ac_id)
        messages_datalink = messages_xml_map.get_msgs("datalink")
        for msg in messages_datalink:
            self._ivy.subscribe(_forward_to_cf, PprzMessage("datalink", msg))


    def shutdown(self):
        if self.verbose:
            print('closing cf2ivy interfaces')
        self._ivy.shutdown()
        self._driver.close()

    def run(self):
        pk = self._driver.receive_packet(0.1) # wait for next message with timeout
        if pk is not None:
            self._got_packet(pk)

    def _got_packet(self, pk):
        if pk.port == CRTP_PORT_PPRZLINK:
            for c in pk.data:
                if self._transport.parse_byte(bytes([c])):
                    (sender_id, _, _, msg) = self._transport.unpack()
                    if self.verbose:
                        print("Got message {} from {}".format(msg.name, sender_id))
                    # Forward message to Ivy bus
                    if self.is_connected:
                        try:
                            self._ivy.send(msg, sender_id=sender_id)
                        except RuntimeError as e:
                            print("Runtime error {}".format(e))
                        except ValueError as e:
                            print("Invalid message error {}".format(e))

    def _link_quality_cb(self, quality):
        pass

    def _link_error_cb(self, msg):
        if self.verbose:
            print("Link error: {}".format(msg))


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Crazyradio link for paparazzi")
    parser.add_argument('-a','--address', default=None, help="URI address of Crazyflie")
    parser.add_argument('-c','--chanel', default='80', help="URI chanel of Crazyflie (full URI will be 'radio://0/%(default)/2M'")
    parser.add_argument('-u','--uri', default=None, help="URI of Crazyflie (chanel option will not be effective)")
    parser.add_argument('-b','--bus', default=None, help="Ivy bus [default to system IVY bus]")
    parser.add_argument('-s','--scan', action='store_true', help="Scan available Crazyflie at startup")
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
    args = parser.parse_args()

    if args.scan:
        import cflib.crtp
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
        cflib.crtp.radiodriver.set_retries(3)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        # Scan for Crazyflies and use the first one found
        print('Scanning interfaces for Crazyflies...')
        if args.address is not None:
            address = int(args.address, 16)
        else:
            address = None # equivalent to default 0xE7E7E7E7E7
        available = cflib.crtp.scan_interfaces(address)
        if len(available) > 0:
            print('Crazyflies found:')
            for i in available:
                print(' ',i[0])
        else:
            print('No radio. Leaving')
            sys.exit(1)

    bridge = None
    link_uri = None
    if args.uri is not None:
        link_uri = args.uri
    else:
        link_uri = 'radio://0/' + args.chanel + '/2M'

    try:
        # Start radio to ivy bridge
        bridge = RadioBridge(link_uri, verbose=args.verbose)

        # The Crazyflie lib doesn't contain anything to keep the application alive,
        # so this is where your application should do something. In our case we
        # are just waiting until we are disconnected.
        while bridge.is_connected:
            bridge.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Failing with error:", e)

    if bridge is not None:
        bridge.is_connected = False
        bridge.shutdown()
        time.sleep(1)

    sys.exit()

