"""
Paparazzi transport encoding utilities

"""

from __future__ import print_function
import struct
from pprz_msg.message import PprzMessage
import messages_xml_map
from enum import Enum

STX = 0x99
STX_TS = 0x98

class PprzParserState(Enum):
    WaitSTX = 1
    GotSTX = 2
    GotLength = 3
    GotPayload = 4
    GotCRC1 = 5

class PprzTransport(object):
    """parser for binary Paparazzi messages"""
    def __init__(self, msg_class='telemetry'):
        self.msg_class = msg_class
        self.reset_parser()

    def reset_parser(self):
        self.state = PprzParserState.WaitSTX
        self.length = 0
        self.buf = []
        self.ck_a = 0
        self.ck_b = 0
        self.idx = 0

    def parse_byte(self, c):
        """parse new byte, return True when a new full message is available"""
        b = ord(c)
        if self.state == PprzParserState.WaitSTX:
            if b == STX:
                self.state = PprzParserState.GotSTX
        elif self.state == PprzParserState.GotSTX:
            self.length = b - 4
            self.buf = []
            self.ck_a = b % 256
            self.ck_b = b % 256
            self.idx = 0
            self.state = PprzParserState.GotLength
        elif self.state == PprzParserState.GotLength:
            self.buf.append(c);
            self.ck_a = (self.ck_a + b) % 256
            self.ck_b = (self.ck_b + self.ck_a) % 256
            self.idx += 1
            if self.idx == self.length:
                self.state = PprzParserState.GotPayload
        elif self.state == PprzParserState.GotPayload:
            if self.ck_a == b:
                self.state = PprzParserState.GotCRC1
            else:
                self.state = PprzParserState.WaitSTX
        elif self.state == PprzParserState.GotCRC1:
            self.state = PprzParserState.WaitSTX
            if self.ck_b == b:
                """New message available"""
                return True
        else:
            self.state = PprzParserState.WaitSTX
        return False

    def get_buffer(self):
        return self.buf

    def unpack_pprz_msg(self, msg_class, data):
        """Unpack a raw PPRZ message"""
        sender_id = ord(data[0])
        msg_id = ord(data[1])
        msg_name = messages_xml_map.get_msg_name(msg_class, msg_id)
        msg = PprzMessage(msg_class, msg_name)
        msg.binary_to_payload(data[2:])
        return (sender_id, msg)

    def unpack(self):
        """Unpack the last received message"""
        return self.unpack_pprz_msg(self.msg_class, self.buf)

    def calculate_checksum(self, msg):
        ck_a = 0
        ck_b = 0
        # start char not included in checksum for pprz protocol
        for c in msg[1:]:
            ck_a = (ck_a + ord(c)) % 256
            ck_b = (ck_b + ck_a) % 256
        return (ck_a, ck_b)

    def pack_pprz_msg(self, sender, msg):
        data = msg.payload_to_binary()
        # STX + length + sender_id + msg_id + data + ck_a + ck_b
        length = 6 + len(data)
        msg = struct.pack("=BBBB", STX, length, sender, msg.msg_id) + data
        (ck_a, ck_b) = self.calculate_checksum(msg)
        msg = msg + struct.pack('=BB', ck_a, ck_b)
        return msg

