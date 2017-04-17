import serial
import threading
import sys
from os import path, getenv
import struct

"""Import Paparazzi Message library"""
# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)
from pprz_msg.message import PprzMessage

"""Global variables"""
PPRZ_STX = 0x99

class SerialMessage():
    def __init__(self):
        self.valid = False
        self.length = 0
        self.counter = 0
        self.sender_id = 0
        self.msg_id = 0
        self.payload = bytearray()
        self.payload_items = None
        self.crc_a = 0
        self.crc_b = 0

class SerialMessageLink():
    """ Send Paparazzi messages over a serial data link.
    
    This library can send and receive paparazzi messages over a serial data
    link, without using Ivy.

    Subscribe to messages with the subscribe function. The callback gets a
    SerialMessage as argument, which will have the requested data in
    payload_items. Example:
    subscribe("DL_VALUE", myCallbackFunction)

    Send messages using the sendMessage function. Example:
    sendMessage("datalink", "SETTING", (setting_id, aircraft_id, setting_value))
    """

    def __init__(self, port, baud):
        self.data_types = { 'float' : ['f', 4],
                            'uint8' : ['B', 1],
                            'uint16' : ['H', 2],
                            'uint32' : ['L', 4],
                            'int8' : ['b', 1],
                            'int16' : ['h', 2],
                            'int32' : ['l', 4]
                           }

        """Initial value for the state machine"""
        self.resetSearchState()

        """Initial subscribed messages dictionary"""
        self.subscriptions = {}

        """Start serial connection"""
        self.serial = serial.Serial(port=port, baudrate=baud)
        self.serial.timeout = 0.5

        """Start thread for reading data on serial port"""
        self.thread = None
        self.alive = threading.Event()
        self.thread = threading.Thread(target=self.serialCallback)
        """Configure to exit thread function when done"""
        self.thread.setDaemon(1)
        self.alive.set()
        self.thread.start()

    def resetSearchState(self):
        self.msg = SerialMessage()
        self.state = 'search_begin'

    def subscribe(self, msg_name, callback):
        pprz_msg = PprzMessage("telemetry", msg_name)
        """ Would be nicer if message.py had a getter for this:"""
        msg_id = pprz_msg._id

        """Define binary format to use for unpacking"""
        fmt = "="
        for a in pprz_msg.fieldtypes:
            fmt += (self.data_types[a][0])

        """Add subscription"""
        self.subscriptions[msg_id] = [callback, pprz_msg, fmt]
        
    def close(self):
        # Wait for thread to finish
        if self.thread is not None:
            self.alive.clear()
            self.thread.join()
            self.thread = None

        # Close serial connection
        self.serial.close()

    def serialCallback(self):
        while self.alive.isSet():
            """Does not block because of serial timeout set in init:"""
            indata = bytearray(self.serial.read(self.serial.inWaiting() or 1))
            for b in indata:
                self.parseByte(int(b))

    """
    PPRZ-message: ABCxxxxxxxDE
        A PPRZ_STX (0x99)
        B LENGTH (A->E)
        C PPRZ_DATA
          0 SENDER_ID
          1 MSG_ID
          2 MSG_PAYLOAD
          . DATA (messages.xml)
        D PPRZ_CHECKSUM_A (sum[B->C])
        E PPRZ_CHECKSUM_B (sum[ck_a])
    """
    def parseByte(self, b):
        if self.state == 'search_begin':
            if b is PPRZ_STX:
                self.msg.counter += 1
                self.state = 'B'

        elif self.state is 'B':
            self.msg.length = b
            self.msg.crc_a += b
            self.msg.crc_b += self.msg.crc_a
            self.msg.counter += 1
            self.state = 'C0'

        elif self.state is 'C0':
            self.msg.sender_id = b
            self.msg.crc_a += b
            self.msg.crc_b += self.msg.crc_a
            self.msg.counter += 1
            self.state = 'C1'

        elif self.state is 'C1':
            self.msg.msg_id = b
            self.msg.crc_a += b
            self.msg.crc_b += self.msg.crc_a
            self.msg.counter += 1
            self.state = 'C2'

        elif self.state is 'C2':
            self.msg.payload.append(b)
            self.msg.crc_a += b
            self.msg.crc_b += self.msg.crc_a
            self.msg.counter += 1
            if self.msg.counter is self.msg.length - 2:
                self.state = 'D'

        elif self.state is 'D':
            """Check if crc_a is correct, otherwise reset"""
            if self.msg.crc_a % 256 is b:
                self.state = 'E'
            else:
                print "CRC_A incorect"
                self.resetSearchState()

        elif self.state is 'E':
            """Check if crc_b is correct, otherwise reject msg"""
            if self.msg.crc_b % 256 is b:
                self.msg.valid = True
                self.messageReceived()
            else:
                print "CRC_B incorrect"
            self.resetSearchState()
        else:
            print "Should never reach this: An unexisting state is called"

    def messageReceived(self):
        """Check if the message has a subscription"""
        if self.msg.msg_id in self.subscriptions and self.msg.valid is True:
            a = self.subscriptions[self.msg.msg_id]
            callback = a[0]
            pprz_msg = a[1]
            fmt = a[2]
            
            """Unpack the data into convenient string"""
            self.msg.payload_items = struct.unpack(fmt, self.msg.payload)

            """Call the callback function of subscription, pass message"""
            callback(self.msg)

    def sendMessage(self, msg_class, msg_name, items):
        """Find the message format"""
        pprz_msg = PprzMessage(msg_class, msg_name)
        msg_id = pprz_msg._id

        """Define binary format to use for packing"""
        fmt = "="
        length = 6
        for a in pprz_msg.fieldtypes:
            fmt += (self.data_types[a][0])
            length += (self.data_types[a][1])
        packed = bytearray(struct.pack(fmt, *items))
        
        """Construct serial package"""
        crc_a = 0
        crc_b = 0
        buf = bytearray()

        """Preamble and length"""
        buf.append(PPRZ_STX)
        buf.append(length)
        crc_a += length
        crc_b += crc_a
        """Sender id"""
        buf.append(0)
        crc_a += 0
        crc_b += crc_a
        """Message id"""
        buf.append(msg_id)
        crc_a += msg_id
        crc_b += crc_a
        """Payload"""
        for b in packed:
            buf.append(b)
            crc_a += int(b)
            crc_b += crc_a
        """CRC_A"""
        buf.append(crc_a % 256)
        """CRC_B"""
        buf.append(crc_b % 256)

        self.serial.write(buf)
