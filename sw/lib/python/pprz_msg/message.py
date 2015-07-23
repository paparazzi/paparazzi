"""
Paparazzi message representation

"""

from __future__ import print_function
import sys
import json
import struct
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

class PprzParser(object):
    """parser for binary Paparazzi messages"""
    def __init__(self):
        self.reset_parser()

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

    def reset_parser(self):
        self.state = PprzParserState.WaitSTX
        self.length = 0
        self.buf = []
        self.ck_a = 0
        self.ck_b = 0
        self.idx = 0

            

class PprzMessageError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()
    def __str__(self):
        return self.message


class PprzMessage(object):
    """base Paparazzi message class"""
    def __init__(self, class_name, name):
        self._class_name = class_name
        self._name = name
        self._id = messages_xml_map.get_msg_id(class_name, name)
        self._fieldnames = messages_xml_map.get_msg_fields(class_name, name)
        self._fieldtypes = messages_xml_map.get_msg_fieldtypes(class_name, self._id)
        self._fieldvalues = []
        # set empty values according to type
        for t in self._fieldtypes:
            if t == "char[]":
                self._fieldvalues.append('')
            elif '[' in t:
                self._fieldvalues.append([0])
            else:
                self._fieldvalues.append(0)

    @property
    def name(self):
        """Get the message name."""
        return self._name

    @property
    def msg_class(self):
        """Get the message class."""
        return self._class_name

    @property
    def fieldnames(self):
        """Get list of field names."""
        return self._fieldnames

    @property
    def fieldvalues(self):
        """Get list of field values."""
        return self._fieldvalues

    @property
    def fieldtypes(self):
        """Get list of field types."""
        return self._fieldtypes

    def fieldbintypes(self, t):
        """Get type and lenfth for binary format"""
        data_types = {
                'float': ['f', 4],
                'uint8': ['B', 1],
                'uint16': ['H', 2],
                'uint32': ['L', 4],
                'int8': ['b', 1],
                'int16': ['h', 2],
                'int32': ['l', 4],
                'char': ['c', 1]
                }
        baseType = t
        if t[-2:] == "[]":
            baseType = t[:-2]
        return data_types[baseType]

    def get_field(self, idx):
        """Get field value by index."""
        return self._fieldvalues[idx]

    def __getattr__(self, attr):
        # Try to dynamically return the field value for the given name
        for idx, f in enumerate(self.fieldnames):
            if f == attr:
                return self.fieldvalues[idx]
        raise AttributeError("No such attribute %s" % attr)

    def set_values(self, values):
        if len(values) == len(self.fieldnames):
            self._fieldvalues = values
        else:
            print("set values %i %i" %(len(values), len(self.fieldnames)))
            raise PprzMessageError("Error: fields not matching")

    def set_value_by_name(self, name, value):
        # Try to set a value from its name
        for idx, f in enumerate(self.fieldnames):
            if f == name:
                self._fieldvalues[idx] = value
                return
        raise AttributeError("No such attribute %s" % name)

    def __str__(self):
        ret = '%s.%s {' % (self.msg_class, self.name)
        for idx, f in enumerate(self.fieldnames):
            ret += '%s : %s, ' % (f, self.fieldvalues[idx])
        ret = ret[0:-2] + '}'
        return ret

    def to_dict(self, payload_only=False):
        d = {}
        if not payload_only:
            d['msgname'] = self.name
            d['msgclass'] = self.msg_class
        for idx, f in enumerate(self.fieldnames):
            d[f] = self.fieldvalues[idx]
        return d

    def to_json(self, payload_only=False):
        return json.dumps(self.to_dict(payload_only))

    def payload_to_ivy_string(self):
        ivy_str = ''
        for idx, t in enumerate(self.fieldtypes):
            if "char[" in t:
                ivy_str += '"' + self.fieldvalues[idx] + '"'
            elif '[' in t:
                ivy_str += ','.join([str(x) for x in self.fieldvalues[idx]])
            else:
                ivy_str += str(self.fieldvalues[idx])
            ivy_str += ' '
        return ivy_str

    def payload_to_binary(self):
        struct_string = "="
        data = []
        length = 0
        for idx, t in enumerate(self.fieldtypes):
            binType = self.fieldbintypes(t)
            struct_string += binType[0]
            array_length = 1
            if "char[" in t:
                array_length = len(self.fieldvalues[idx])
                for c in self.fieldvalues[idx]:
                    data.append(int(c))
            elif '[' in t:
                array_length = len(self.fieldvalues[idx])
                for x in self.fieldvalues[idx]:
                    data.append(x)
            else:
                data.append(self.fieldvalues[idx])
            length += binType[1] * array_length
        msg = struct.pack(struct_string, *data)
        return msg

    def binary_to_payload(self, data):
        msg_offset = 0
        values = []
        for idx, t in enumerate(self.fieldtypes):
            binType = self.fieldbintypes(t)
            if t[-2:] == "[]":
                array_length = int(struct.unpack('B', data[msg_offset])[0])
                msg_offset = msg_offset + 1
                array_value = []
                for count in range(0, array_length):
                    array_value.append(struct.unpack(binType[0], "".join(data[msg_offset:msg_offset+binType[1]])))
                    msg_offset = msg_offset + binType[1]
                values.append(array_value)
            else:
                value = struct.unpack(binType[0], "".join(data[msg_offset:msg_offset+binType[1]]))
                msg_offset = msg_offset + binType[1]
                values.append(value)
        self.set_values(values)

    def calculate_checksum(self, msg):
        ck_a = 0
        ck_b = 0
        # start char not included in checksum for pprz protocol
        for c in msg[1:]:
            ck_a = (ck_a + ord(c)) % 256
            ck_b = (ck_b + ck_a) % 256
        return (ck_a, ck_b)

    def payload_to_pprz_msg(self, sender):
        stx = STX
        data = self.payload_to_binary()
        length = 6 + len(data)
        msg = struct.pack("=BBBB", STX, length, sender, self._id) + data
        (ck_a, ck_b) = self.calculate_checksum(msg)
        msg = msg + struct.pack('=BB', ck_a, ck_b)
        return msg

def test():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="path to messages.xml file")
    parser.add_argument("-c", "--class", help="message class", dest="msg_class", default="telemetry")
    args = parser.parse_args()
    messages_xml_map.parse_messages(args.file)
    messages = [PprzMessage(args.msg_class, n) for n in messages_xml_map.get_msgs(args.msg_class)]
    print("Listing %i messages in '%s' msg_class" % (len(messages), args.msg_class))
    for msg in messages:
        print(msg)

if __name__ == '__main__':
    test()
