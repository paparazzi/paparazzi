#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

from math import radians
from time import sleep
import numpy as np

class Rotorcraft:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.X = np.zeros(3) # Position
        self.V = np.zeros(3) # Velocity
        self.W = np.zeros(3) # Angles
        self.timeout = 0

class Guidance(object):
    def __init__(self, verbose=False, interface=None, quad_ids = None):
        self.verbose = verbose
        self._interface = interface
        self.auto2_index = None                
        self.ac_id = quad_ids[0]
        self.ids = quad_ids
        self.ap_mode = None
        self.rotorcrafts = [Rotorcraft(i) for i in quad_ids]

        try:
            settings = PaparazziACSettings(self.ac_id) # target and follower should be checked, FIX ME !
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
        self._interface = IvyMessagesInterface("Deep Guidance is on the way...")

        # bind to INS message
        def ins_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "INS":
                rc = self.rotorcrafts[self.ids.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                rc.X[0] = float(msg['ins_x']) * i2p
                rc.X[1] = float(msg['ins_y']) * i2p
                rc.X[2] = float(msg['ins_z']) * i2p
                rc.V[0] = float(msg['ins_xd']) * i2v
                rc.V[1] = float(msg['ins_yd']) * i2v
                rc.V[2] = float(msg['ins_zd']) * i2v
                rc.timeout = 0
                rc.initialized = True
        # self._interface.subscribe(ins_cb, PprzMessage("telemetry", "INS"))

        #################################################################
        def rotorcraft_fp_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "ROTORCRAFT_FP":
                rc = self.rotorcrafts[self.ids.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                i2w = 1. / 2**12     # integer to angle
                rc.X[0] = float(msg['north']) * i2p
                rc.X[1] = float(msg['east']) * i2p
                rc.X[2] = float(msg['up']) * i2p
                rc.V[0] = float(msg['vnorth']) * i2v
                rc.V[1] = float(msg['veast']) * i2v
                rc.V[2] = float(msg['vup']) * i2v
                rc.W[2] = float(msg['psi']) * i2w
                rc.timeout = 0
                rc.initialized = True
        
        # Un-comment this if the quadrotors are providing state information to use_deep_guidance.py
        self._interface.subscribe(rotorcraft_fp_cb, PprzMessage("telemetry", "ROTORCRAFT_FP"))
    
        # bind to GROUND_REF message : ENAC Voliere is sending LTP_ENU
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id in self.ids:
                rc = self.rotorcrafts[self.ids.index(ac_id)]
                # X and V in NED
                rc.X[0] = float(msg['pos'][1])
                rc.X[1] = float(msg['pos'][0])
                rc.X[2] = float(msg['pos'][2])
                rc.V[0] = float(msg['speed'][1])
                rc.V[1] = float(msg['speed'][0])
                rc.V[2] = float(msg['speed'][2])
                rc.timeout = 0
                rc.initialized = True
        
        # Un-comment this if optitrack is being used for state information for use_deep_guidance.py **For use only in the Voliere**
        #self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))
        ################################################################

    # <message name="ROTORCRAFT_FP" id="147">
    #   <field name="east"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="north"    type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="up"       type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="veast"    type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="vnorth"   type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="vup"      type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="phi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="theta"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="psi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="carrot_east"   type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_north"  type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_up"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_psi"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="thrust"        type="int32"/>
    #   <field name="flight_time"   type="uint16" unit="s"/>
    # </message>

    # <message name="INS" id="198">
    #   <field name="ins_x"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_y"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_z"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_xd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_yd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_zd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_xdd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    #   <field name="ins_ydd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    #   <field name="ins_zdd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    # </message>

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

    def set_guided_mode(self, quad_id = None):
        """
        change mode to GUIDED.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = quad_id
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

    def set_nav_mode(self, quad_id = None):
        """
        change mode to NAV.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = quad_id
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

    def goto_ned(self, north, east, down, heading=0.0):
        """
        goto a local NorthEastDown position in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x00
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = heading
        print("goto NED: %s" % msg)
        # embed the message in RAW_DATALINK so that the server can log it
        self._interface.send_raw_datalink(msg)

    def goto_ned_relative(self, north, east, down, yaw=0.0):
        """
        goto a local NorthEastDown position relative to current position in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x0D
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = yaw
        print("goto NED relative: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def goto_body_relative(self, forward, right, down, yaw=0.0):
        """
        goto to a position relative to current position and heading in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x0E
        msg['x'] = forward
        msg['y'] = right
        msg['z'] = down
        msg['yaw'] = yaw
        print("goto body relative: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def move_at_ned_vel(self, north=0.0, east=0.0, down=0.0, yaw=0.0, quad_id = None):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = quad_id
        msg['flags'] = 0xE0
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = yaw
        #print("move at vel NED: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def move_at_body_vel(self, forward=0.0, right=0.0, down=0.0, yaw=0.0):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x62
        msg['x'] = forward
        msg['y'] = right
        msg['z'] = down
        msg['yaw'] = yaw
        print("move at vel body: %s" % msg)
        self._interface.send_raw_datalink(msg)
        
    def jump_to_block(self, block_id):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("ground", "JUMP_TO_BLOCK")
        msg['ac_id'] = self.ac_id
        msg['block_id'] = block_id
        print("jumping to block: %s" % block_id)
        print("ac id: %s" % self.ac_id)
        self._interface.send(msg)

    def accelerate(self, north=0.0, east=0.0, down=0.0, quad_id = 2):
        msg = PprzMessage("datalink", "DESIRED_SETPOINT")
        msg['ac_id'] = quad_id
        msg['flag'] = 0 # full 3D
        msg['ux'] = north
        msg['uy'] = east
        msg['uz'] = down
        self._interface.send(msg)

    # <message name="GUIDED_SETPOINT_NED" id="40" link="forwarded">
    #   <description>
    #     Set vehicle position or velocity in NED.
    #     Frame can be specified with the bits 0-3
    #     Velocity of position setpoint can be specified with the bits 5-7
    #     Flags field definition:
    #     - bit 0: x,y as offset coordinates
    #     - bit 1: x,y in body coordinates
    #     - bit 2: z as offset coordinates
    #     - bit 3: yaw as offset coordinates
    #     - bit 4: free
    #     - bit 5: x,y as vel
    #     - bit 6: z as vel
    #     - bit 7: yaw as rate
    #   </description>
    #   <field name="ac_id" type="uint8"/>
    #   <field name="flags" type="uint8">bits 0-3: frame, bits 5-7: use as velocity</field>
    #   <field name="x" type="float" unit="m">X position/velocity in NED</field>
    #   <field name="y" type="float" unit="m">Y position/velocity in NED</field>
    #   <field name="z" type="float" unit="m">Z position/velocity in NED (negative altitude)</field>
    #   <field name="yaw" type="float" unit="rad" alt_unit="deg">yaw/rate setpoint</field>
    # </message>

# class IvyRequester(object):
#     def __init__(self, interface=None):
#         self._interface = interface
#         if interface is None:
#             self._interface = IvyMessagesInterface("ivy requester")
#         self.ac_list = []

#     def __del__(self):
#         self.shutdown()

#     def shutdown(self):
#         if self._interface is not None:
#             print("Shutting down ivy interface...")
#             self._interface.shutdown()
#             self._interface = None

#     def get_aircrafts(self):

#         def aircrafts_cb(ac_id, msg):
#             self.ac_list = [int(a) for a in msg['ac_list'].split(',') if a]
#             print("aircrafts: {}".format(self.ac_list))

#         self._interface.subscribe(aircrafts_cb, "(.*AIRCRAFTS .*)")
#         sender = 'get_aircrafts'
#         request_id = '42_1' # fake request id, should be PID_index
#         self._interface.send("{} {} AIRCRAFTS_REQ".format(sender, request_id))
#         # hack: sleep briefly to wait for answer
#         sleep(0.1)
#         return self.ac_list
