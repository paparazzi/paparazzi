from __future__ import absolute_import, division, print_function

import threading
import serial
import os
import sys

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprz_msg.message import PprzMessage
from pprz_msg.pprz_transport import PprzTransport
import pprz_msg.messages_xml_map


class SerialMessagesInterface(threading.Thread):
    def __init__(self, callback, verbose=False, device='/dev/ttyUSB0', baudrate=115200,
                 msg_class='telemetry'):
        threading.Thread.__init__(self)
        self.callback = callback
        self.verbose = verbose
        self.msg_class = msg_class
        self.running = True
        try:
            self.ser = serial.Serial(device, baudrate, timeout=1.0)
        except serial.SerialException:
            print("Error: unable to open serial port '%s'" % device)
            exit(0)
        self.trans = PprzTransport(msg_class)

    def stop(self):
        print("End thread and close serial link")
        self.running = False
        self.ser.close()

    def shutdown(self):
        self.stop()

    def __init__del__(self):
        try:
            self.ser.close()
        except:
            pass

    def send(self, msg, sender_id):
        """ Send a message over a serial link"""
        if isinstance(msg, PprzMessage):
            data = self.trans.pack_pprz_msg(sender_id, msg)
            self.ser.write(data)
            self.ser.flush()

    def run(self):
        """Thread running function"""
        try:
            while self.running:
                # Parse incoming data
                c = self.ser.read(1)
                if len(c) == 1:
                    if self.trans.parse_byte(c):
                        (sender_id, msg) = self.trans.unpack()
                        if self.verbose:
                            print("New incoming message '%s' from %i" % (msg.name, sender_id))
                        # Callback function on new message 
                        self.callback(sender_id, msg)

        except StopIteration:
            pass


def test():
    import time
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="path to messages.xml file")
    parser.add_argument("-c", "--class", help="message class", dest='msg_class', default='telemetry')
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/ttyUSB0')
    parser.add_argument("-b", "--baudrate", help="baudrate", dest='baud', default=115200, type=int)
    args = parser.parse_args()
    pprz_msg.messages_xml_map.parse_messages(args.file)
    serial_interface = SerialMessagesInterface(lambda s, m: print("new message from %i: %s" % (s, m)), device=args.dev,
                                               baudrate=args.baud, msg_class=args.msg_class, verbose=True)

    print("Starting serial interface on %s at %i baud" % (args.dev, args.baud))
    try:
        serial_interface.start()

        # give the thread some time to properly start
        time.sleep(0.1)

        # send some datalink messages to aicraft for test
        ac_id = 42
        print("sending ping")
        ping = PprzMessage('datalink', 'PING')
        serial_interface.send(ping, 0)

        get_setting = PprzMessage('datalink', 'GET_SETTING')
        get_setting['index'] = 0
        get_setting['ac_id'] = ac_id
        serial_interface.send(get_setting, 0)

        # change setting with index 0 (usually the telemetry mode)
        set_setting = PprzMessage('datalink', 'SETTING')
        set_setting['index'] = 0
        set_setting['ac_id'] = ac_id
        set_setting['value'] = 1
        serial_interface.send(set_setting, 0)

        # block = PprzMessage('datalink', 'BLOCK')
        # block['block_id'] = 3
        # block['ac_id'] = ac_id
        # serial_interface.send(block, 0)

        while serial_interface.isAlive():
            serial_interface.join(1)
    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
        serial_interface.stop()
        exit()


if __name__ == '__main__':
    test()
