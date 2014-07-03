#Copyright 2014, Antoine Drouin
import os, serial, logging
from gi.repository import GLib, Gio, GObject

import phoenix.pprz_transport as transp

LOG = logging.getLogger('serial_link')
LOG.setLevel(logging.ERROR)
#LOG.setLevel(logging.DEBUG)
import pdb

class PhoenixCommunication(GObject.GObject):

    DEFAULT_PORT = "/dev/ttyO4"
    DEFAULT_SPEED = 57600

    __gsignals__ = {
        "message-received" : (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, [
            GObject.TYPE_PYOBJECT,      #header
            GObject.TYPE_PYOBJECT]),    #payload
        "status-changed" : (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, [])
        }

    def __init__(self, pprz_protocol):
        GObject.GObject.__init__(self)
        self._port = PhoenixCommunication.DEFAULT_PORT
        self._speed = PhoenixCommunication.DEFAULT_SPEED
        self._watch = None
        self._is_opened = False;
        self._transp = transp.Transport(False, False)
        self._pprz_protocol = pprz_protocol
        self._serial = None
        self._available_ports = []
        self.update_available_ports()
        self._nb_rx_msgs = 0
        self._nb_rx_bytes = 0
        self._nb_tx_msgs = 0
        self._nb_tx_bytes = 0


    def open(self):
        """
        Open serial port
        """
        LOG.debug("Opening Port: %s @ %d" % (self._port, self._speed))
        try:
            self._serial = serial.Serial(self._port, self._speed, timeout=0)
            self._is_opened = True;
        except serial.SerialException:
            self._is_opened = False;
        self.on_connection_changed()

    def send_msg(self, msg_class, msg_name, *fields_values):
        """
        send a message
        """
        m  = self._pprz_protocol.get_message_by_name(msg_class, msg_name)
        h = transp.TransportHeaderFooter(acid=0)
        bin_msg = self._transp.pack_message_with_values(h, m, *fields_values)
        self._serial.write(bin_msg)
        self._nb_tx_msgs += 1
        self._nb_tx_bytes += len(bin_msg)

    def on_connection_changed(self):
        if self._watch:
            GObject.source_remove(self._watch)
        if self._is_opened:
            self.watch = GObject.io_add_watch(
                self._serial.fileno(),
                GObject.IO_IN | GObject.IO_PRI,
                self.on_serial_data_available,
                priority=GObject.PRIORITY_HIGH)
        self.update_available_ports()
        self.emit("status-changed")

    def get_status(self):
        return self._is_opened, self._port

    def get_stats(self):
        return self._nb_rx_msgs, self._nb_rx_bytes, self._nb_tx_msgs, self._nb_tx_bytes


    def on_serial_data_available(self, fd, condition):
        try:
            data = self._serial.read(4096)
            LOG.debug("read serial data : %d" % (len(data)))
            self._nb_rx_bytes += len(data)
            #        print data.encode("hex")
            msgs = self._transp.parse_many(data)
            LOG.debug("parsed msg : %d" % (len(msgs)))
            #        print msgs
            for header, payload in msgs:
                self.emit("message-received", header, payload)
                self._nb_rx_msgs += 1
            return True
        except serial.SerialException:
            self._is_opened = False;
            self.on_connection_changed()
            return False

    def update_available_ports(self):
#        self.available_ports = filter(lambda x: x.startswith("ttyUSB") or x.startswith("ttyS") , os.listdir("/dev"))
        self.available_ports = [x for x in os.listdir("/dev") if x.startswith("ttyUSB")]
        self.available_ports.sort()

    def get_available_ports(self):
        return self.available_ports
