"""
Paparazzi message representation

"""

from __future__ import print_function
import sys
import json
import messages_xml_map


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
        self._fieldvalues = []

    def get_msgname(self):
        return self._name

    def get_classname(self):
        return self._class_name

    def get_fieldnames(self):
        return self._fieldnames

    def get_fieldvalues(self):
        return self._fieldvalues

    def get_field(self, idx):
        return self._fieldvalues[idx]

    def set_values(self, values):
        if len(values) == len(self._fieldnames):
            self._fieldvalues = values
        else:
            raise PprzMessageError("Error: fields not matching")

    def __str__(self):
        ret = '%s.%s {' % (self._class_name, self._name)
        for idx, f in enumerate(self._fieldnames):
            ret += '%s : %s, ' % (f, self._fieldvalues[idx])
        ret = ret[0:-2] + '}'
        return ret

    def to_dict(self, payload_only=False):
        d = {}
        if not payload_only:
            d['msgname'] = self._name
            d['msgclass'] = self._class_name
        for idx, f in enumerate(self._fieldnames):
            d[f] = self._fieldvalues[idx]
        return d

    def to_json(self, payload_only=False):
        return json.dumps(self.to_dict(payload_only))
