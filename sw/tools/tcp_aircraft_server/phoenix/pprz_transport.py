#Copyright 2014, Antoine Drouin
from __future__ import print_function
import array

STX = 0x99
#6 non payload bytes; STX, LEN, MSGID, ACID, CK_A, CK_B
NUM_NON_PAYLOAD_BYTES = 6

class TransportHeaderFooter:
    """
    The header/footer of a message

    **Attributes:**
        - stx: start byte
        - length: length in bytes (total length, i.e. including header/footer
        - acid: aircraft ID of sender/destination
        - msgid: message ID
        - ck_a: checksum high byte
        - ck_b: checksum low byte
    """
    def __init__(self, stx=STX, length=NUM_NON_PAYLOAD_BYTES, acid=0, msgid=0, ck_a=0, ck_b=0):
        self.stx = stx
        self.length = length
        self.acid = acid
        self.msgid = msgid
        self.ck_a = ck_a
        self.ck_b = ck_b

class Transport:
    """
    Class that extracts a wasp payload from a string or sequence of
    characters (data is sent in little endian byte order)

    Data is expected in the following form ::

        |STX|length|AC_ID|MESSAGE_ID|... payload=(length-6) bytes ...|Checksum A|Checksum B|

    Payload ::

        |... MESSAGE DATA ...|

    There are 6 non payload bytes in a packet (described in :mod:`TransportHeaderFooter`
        - STX
        - length
        - AC_ID
        - MESSAGE_ID
        - Checksum A
        - Checksum B
    """

    STATE_UNINIT,       \
    STATE_GOT_STX,      \
    STATE_GOT_LENGTH,   \
    STATE_GOT_ACID,     \
    STATE_GOT_MSGID,    \
    STATE_GOT_PAYLOAD,  \
    STATE_GOT_CRC1 =    list(range(0,7))

    def __init__(self, check_crc=True, debug=False):
        self._check_crc = check_crc
        self._debug = debug
        self._buf = array.array('c','\0'*256)
        self._state = self.STATE_UNINIT
        self._total_len = 0
        self._payload_len = 0
        self._payload_idx = 0
        self._ck_a = 0
        self._ck_b = 0
        self._error = 0
        self._acid = 0
        self._msgid = 0

    def _debug_msg(self, msg):
        if self._debug:
            print(msg)

    def pack_message_with_values(self, header, message, *values):
        return self.pack_one(
                        header,
                        message,
                        message.pack_values(*values))

    def pack_one(self, header, message, payload):
        payload_len = len(payload)
        total_len = payload_len + NUM_NON_PAYLOAD_BYTES

        #create an array big enough to hold data before the payload,
        #i.e. exclude the checksum
        buf = array.array('c','\0'*(NUM_NON_PAYLOAD_BYTES - 2))

        buf[0] = chr(header.stx)
        buf[1] = chr(total_len)
        buf[2] = chr(header.acid)
        buf[3] = chr(message.id)

        buf.fromstring(payload)

        ck_a = total_len
        ck_b = total_len
        for i in range(2,len(buf)):
            ck_a = (ck_a + ord(buf[i])) % 256
            ck_b = (ck_b + ck_a) % 256

        buf.append(chr(ck_a))
        buf.append(chr(ck_b))

        return buf

    def parse_many(self, string):
        """
        Similar to parse_one, but operates on a string, returning
        multiple payloads if successful

        :returns: A list of payloads strings
        """
        payloads = []
        for c in string:
            ok,h,p = self.parse_one(c)
            if ok:
                payloads.append((h,p))
        return payloads

    def parse_one(self, c):
        """
        Attempts to parse one character. Returns just the payload, and
        not the data in the transport layer, i.e. it does not return
        STX, the length, or the checksums

        :returns: The payload string, or an empty string if insuficcient data is available
        """

        def update_checksum(d):
            #wrap to 8bit (simulate 8 bit addition)
            self._ck_a = (self._ck_a + d) % 256
            self._ck_b = (self._ck_b + self._ck_a) % 256

        def add_to_buf(char, uint8):
            self._buf[self._payload_idx] = char
            self._payload_idx += 1
            update_checksum(uint8)

        payload = ""
        error = False
        received = False
        #convert to 8bit int
        d = ord(c)

        if self._state == self.STATE_UNINIT:
            if d == STX:
                self._state += 1
                self._ck_a = STX
                self._ck_b = STX
                self._debug_msg("-- STX")
        elif self._state == self.STATE_GOT_STX:
            self._total_len = d
            self._payload_len = d - NUM_NON_PAYLOAD_BYTES
            self._payload_idx = 0
            update_checksum(d)
            self._state += 1
            self._debug_msg("-- SIZE: PL (%s) TOT (%s)" % (self._payload_len, self._total_len))
        elif self._state == self.STATE_GOT_LENGTH:
            self._debug_msg("-- ACID: %x" % d)
            self._acid = d
            update_checksum(d)
            self._state += 1
        elif self._state == self.STATE_GOT_ACID:
            self._debug_msg("-- MSGID: %x" % d)
            self._msgid = d
            update_checksum(d)
            if self._payload_len == 0:
                self._state = self.STATE_GOT_PAYLOAD
            else:
                self._state += 1
        elif self._state == self.STATE_GOT_MSGID:
            add_to_buf(c, d)
            if self._payload_idx == self._payload_len:
                self._state += 1
                self._debug_msg("-- PL")
        elif self._state == self.STATE_GOT_PAYLOAD:
            if d != self._ck_a and self._check_crc:
                error = True
                self._debug_msg("-- CRC_A ERROR %x v %x" % (d, self._ck_a))
            else:
                self._state += 1
                self._debug_msg("-- CRC_A OK")
        elif self._state == self.STATE_GOT_CRC1:
            if d != self._ck_b and self._check_crc:
                error = True
                self._debug_msg("-- CRC_B ERROR")
            else:
                payload = self._buf[:self._payload_len].tostring()
                received = True
                self._state = self.STATE_UNINIT
                self._debug_msg("-- CRC_B OK")

        if error:
            self._error += 1
            self._state = self.STATE_UNINIT
        elif received:
            header = TransportHeaderFooter(
                length=self._total_len,
                acid=self._acid,
                msgid=self._msgid,
                ck_a=self._ck_a,
                ck_b=self._ck_b)
            return True, header, payload

        return False, None, None


