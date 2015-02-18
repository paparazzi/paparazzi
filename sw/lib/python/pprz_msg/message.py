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
            raise PprzMessageError("Error: fields not matching")

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
