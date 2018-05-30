#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv

# if PAPARAZZI_SRC and PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python") # settings_xml
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

from math import radians
from time import sleep

import threading
import serial

J_RIGHT     = 1<<0
J_LEFT      = 1<<1
J_UP        = 1<<2
J_DOWN      = 1<<3
J_A         = 1<<4
J_B         = 1<<5
J_SELECT    = 1<<6
J_START     = 1<<7

class Guidance(object):
    def __init__(self, ac_id, verbose=False):
        self.ac_id = ac_id
        self.verbose = verbose
        self._interface = None
        self.ap_mode = None
        try:
            settings = PaparazziACSettings(self.ac_id)
        except Exception as e:
            print(e)
            return
        try:
            self.ap_mode = settings.name_lookup['mode'] # try classic name
        except Exception as e:
            try:
                self.ap_mode = settings.name_lookup['ap'] # in case it is a generated autopilot
            except Exception as e:
                print(e)
                print("ap_mode setting not found, mode change not possible.")
        self._interface = IvyMessagesInterface("gb2ivy")

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

    def bind_flight_param(self, send_cb):
        def bat_cb(ac_id, msg):
            bat = float(msg['bat'])
            # should not be more that 18 characters
            send_cb('bat: '+str(bat)+' V')
        self._interface.subscribe(bat_cb, regex=('(^ground ENGINE_STATUS '+str(self.ac_id)+' .*)'))

    def set_guided_mode(self):
        """
        change mode to GUIDED.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self.ac_id
            msg['index'] = self.ap_mode.index
            try:
                msg['value'] = self.ap_mode.ValueFromName('Guided')  # AP_MODE_GUIDED
            except ValueError:
                try:
                    msg['value'] = self.ap_mode.ValueFromName('GUIDED')  # AP_MODE_GUIDED
                except ValueError:
                    msg['value'] = 19 # fallback to fixed index
            print("Setting mode to GUIDED: %s" % msg)
            self._interface.send(msg)

    def set_nav_mode(self):
        """
        change mode to NAV.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self.ac_id
            msg['index'] = self.ap_mode.index
            try:
                msg['value'] = self.ap_mode.ValueFromName('Nav')  # AP_MODE_NAV
            except ValueError:
                try:
                    msg['value'] = self.ap_mode.ValueFromName('NAV')  # AP_MODE_NAV
                except ValueError:
                    msg['value'] = 13 # fallback to fixed index
            print("Setting mode to NAV: %s" % msg)
            self._interface.send(msg)

    def move_at_body_vel(self, forward=0.0, right=0.0, down=0.0, yaw=0.0):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0xE2
        msg['x'] = forward
        msg['y'] = right
        msg['z'] = down
        msg['yaw'] = yaw
        print("move at vel body: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def command_callback(self, command):
        """
        convert incoming command into velocity
        """
        right = 0.0
        forward = 0.0
        down = 0.0
        yaw = 0.0
        command = int(command)
        if command & J_RIGHT:
            right += 2.0
        if command & J_LEFT:
            right -= 2.0
        if command & J_UP:
            forward += 2.0
        if command & J_DOWN:
            forward -= 2.0
        if command & J_A:
            down -= 1.0
        if command & J_B:
            down += 1.0
        if command & J_START:
            yaw += radians(20)
        if command & J_SELECT:
            yaw -= radians(20)
        self.move_at_body_vel(forward, right, down, yaw)


class SerialInterface(threading.Thread):
    def __init__(self, callback, verbose=False, device='/dev/ttyUSB0', baudrate=9600):
        threading.Thread.__init__(self)
        self.callback = callback
        self.verbose = verbose
        self.running = True
        try:
            self.ser = serial.Serial(device, baudrate, timeout=1.0)
        except serial.SerialException:
            print("Error: unable to open serial port '%s'" % device)
            exit(0)
        #self.trans = PprzTransport(msg_class)

    def stop(self):
        print("End thread and close serial link")
        self.running = False
        self.ser.close()

    def shutdown(self):
        self.stop()

    def __del__(self):
        try:
            self.ser.close()
        except:
            pass

    def send(self, msg):
        """
        Send a text message over a serial link
        Max number of char for a GameBoy screen is 18
        Truncate or pad with space when needed
        """
        text = msg[:17]
        text = text + (' ' * (17-len(text)))
        for c in text:
            self.ser.write(c)
            self.ser.flush()
            sleep(0.01)
        # send 0 for sync
        self.ser.write('\0')

    def run(self):
        """
        Thread running function
        """
        try:
            while self.running:
                # Parse incoming data
                c = self.ser.readline()
                if len(c) > 0:
                    # Callback function on new message
                    self.callback(c)

        except StopIteration:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--acid", help="aircraft ID", dest='acid', default=1, type=int)
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/ttyUSB0')
    parser.add_argument("-b", "--baudrate", help="baudrate", dest='baud', default=9600, type=int)
    args = parser.parse_args()

    print("Starting serial interface on %s at %i baud" % (args.dev, args.baud))
    try:
        guidance = Guidance(args.acid)
        serial_interface = SerialInterface(guidance.command_callback, device=args.dev, baudrate=args.baud)
        serial_interface.start()
        guidance.bind_flight_param(serial_interface.send)

        # give the thread some time to properly start
        sleep(0.1)
        guidance.set_guided_mode()

        while serial_interface.isAlive():
            serial_interface.join(1)
    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
        guidance.set_nav_mode()
        serial_interface.stop()
        guidance.shutdown()
        exit()
    except (OSError):
        print('Serial port not found')
        guidance.shutdown()
        exit()

if __name__ == '__main__':
    main()

