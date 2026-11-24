#
# Copyright (C) 2026 Freek van Tienen
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

"""
Module specific messages (MODULE_SPECIFIC / MODULE_SPECIFIC_UP).

A module declares its own messages in its XML file:

    <messages id="1">
      <message name="STATUS" id="0">
        <field name="position" type="uint16"/>
        <field name="cg" type="float[3]" unit="m"/>
      </message>
      <message name="SET_ZERO" id="0" class="datalink" fun="foo_set_zero"/>
    </messages>

The payload of MODULE_SPECIFIC(_UP) is then:
  - module_id: the static unique id of the module (`messages@id`)
  - reserved:  low byte = message id inside the module, high byte = instance (0 for now)
  - data:      the fields of the message, packed little endian without padding

A variable length array (`type[]`) is only allowed as the last field, its length is
deduced from the data length.

This file is used to generate the airborne header (sw/tools/generators/gen_module_msgs.py),
to check that module ids are unique, and to decode the messages on the ground.
"""

import glob
import os
import re
import struct
from collections import OrderedDict
from xml.etree import ElementTree

# name: (struct format, size, C type)
TYPES = {
    'int8': ('b', 1, 'int8_t'),
    'uint8': ('B', 1, 'uint8_t'),
    'int16': ('h', 2, 'int16_t'),
    'uint16': ('H', 2, 'uint16_t'),
    'int32': ('i', 4, 'int32_t'),
    'uint32': ('I', 4, 'uint32_t'),
    'int64': ('q', 8, 'int64_t'),
    'uint64': ('Q', 8, 'uint64_t'),
    'float': ('f', 4, 'float'),
    'double': ('d', 8, 'double'),
    'char': ('c', 1, 'char'),
}

CLASSES = ('telemetry', 'datalink')

# Max size of the data array: a pprzlink v2 frame is at most 255 bytes,
# the MODULE_SPECIFIC_UP header (ac_id, module_id, reserved, data length) takes 6 of them
MAX_DATA = 240

MODULE_ID_MAX = 0xFFFF
MSG_ID_MAX = 0xFF


def reserved_field(msg_id, instance=0):
    """ Value of the `reserved` field of MODULE_SPECIFIC(_UP) """
    return (msg_id & 0xFF) | ((instance & 0xFF) << 8)


def split_reserved(reserved):
    """ (msg_id, instance) from the `reserved` field """
    return reserved & 0xFF, (reserved >> 8) & 0xFF


class ModuleMsgsError(Exception):
    pass


class Field(object):
    def __init__(self, name, type_str, unit=None, alt_unit=None, alt_unit_coef=None, description=None):
        self.name = name
        self.type_str = type_str
        self.unit = unit
        self.alt_unit = alt_unit
        self.alt_unit_coef = alt_unit_coef
        self.description = description
        m = re.match(r'^([a-z0-9]+)(\[(\d*)\])?$', type_str)
        if m is None or m.group(1) not in TYPES:
            raise ModuleMsgsError("unknown type '%s' for field '%s'" % (type_str, name))
        self.base = m.group(1)
        self.is_array = m.group(2) is not None
        self.array_len = int(m.group(3)) if self.is_array and m.group(3) else None
        self.is_var = self.is_array and self.array_len is None
        if self.is_array and self.array_len == 0:
            raise ModuleMsgsError("field '%s': fixed arrays need a length > 0" % name)
        if not re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', name):
            raise ModuleMsgsError("invalid field name '%s'" % name)

    @property
    def fmt(self):
        return TYPES[self.base][0]

    @property
    def elem_size(self):
        return TYPES[self.base][1]

    @property
    def c_type(self):
        return TYPES[self.base][2]

    @property
    def size(self):
        """ size in bytes (element size for a variable array) """
        if self.is_var:
            return 0
        return self.elem_size * (self.array_len or 1)


class Message(object):
    def __init__(self, module, name, msg_id, msg_class, fields, fun=None, description=None):
        self.module = module
        self.name = name
        self.id = msg_id
        self.msg_class = msg_class
        self.fields = fields
        self.fun = fun
        self.description = description

    @property
    def fixed_size(self):
        return sum(f.size for f in self.fields)

    @property
    def var_field(self):
        if self.fields and self.fields[-1].is_var:
            return self.fields[-1]
        return None

    def decode(self, data):
        """ Decode the data bytes of a message into an OrderedDict of field values """
        data = bytes(data)
        if len(data) < self.fixed_size:
            raise ModuleMsgsError("%s.%s: %d bytes received, %d expected"
                                  % (self.module.name, self.name, len(data), self.fixed_size))
        values = OrderedDict()
        offset = 0
        for f in self.fields:
            if f.is_var:
                n = (len(data) - offset) // f.elem_size
            elif f.is_array:
                n = f.array_len
            else:
                n = 1
            raw = data[offset:offset + n * f.elem_size]
            offset += n * f.elem_size
            if f.base == 'char':
                values[f.name] = raw.split(b'\0', 1)[0].decode('utf-8', 'replace') if f.is_array else raw.decode('utf-8', 'replace')
                continue
            v = struct.unpack('<%d%s' % (n, f.fmt), raw)
            values[f.name] = list(v) if f.is_array else v[0]
        return values

    def encode(self, values, instance=0):
        """ Encode a dict of field values, returns (module_id, reserved, data bytes) """
        data = b''
        for f in self.fields:
            v = values[f.name]
            if f.base == 'char':
                b = v.encode('utf-8') if isinstance(v, str) else bytes(v)
                if f.is_array and not f.is_var:
                    b = b[:f.array_len].ljust(f.array_len, b'\0')
                data += b
                continue
            if f.is_array:
                if not f.is_var and len(v) != f.array_len:
                    raise ModuleMsgsError("field '%s': %d values expected" % (f.name, f.array_len))
                data += struct.pack('<%d%s' % (len(v), f.fmt), *v)
            else:
                data += struct.pack('<' + f.fmt, v)
        if len(data) > MAX_DATA:
            raise ModuleMsgsError("%s.%s: data too long (%d bytes)" % (self.module.name, self.name, len(data)))
        return self.module.id, reserved_field(self.id, instance), data


class Module(object):
    def __init__(self, name, module_id, filename):
        self.name = name
        self.id = module_id
        self.filename = filename
        self.messages = []

    def get_message(self, msg_class, msg_id):
        for m in self.messages:
            if m.msg_class == msg_class and m.id == msg_id:
                return m
        return None

    def get_message_by_name(self, name):
        for m in self.messages:
            if m.name == name:
                return m
        return None


def _int(s, what):
    try:
        return int(s, 0)
    except (TypeError, ValueError):
        raise ModuleMsgsError("invalid %s '%s'" % (what, s))


def parse_module(filename):
    """ Parse the <messages> node of a module XML file, returns a Module or None if there is none """
    try:
        root = ElementTree.parse(filename).getroot()
    except ElementTree.ParseError as e:
        raise ModuleMsgsError("%s: %s" % (filename, e))
    if root.tag != 'module':
        return None
    node = root.find('messages')
    if node is None:
        return None
    name = root.get('name')
    try:
        module_id = _int(node.get('id'), 'module id')
        if not 0 < module_id <= MODULE_ID_MAX:
            raise ModuleMsgsError("module id %d out of range [1, %d]" % (module_id, MODULE_ID_MAX))
        module = Module(name, module_id, filename)
        for m in node.findall('message'):
            msg_name = m.get('name')
            if msg_name is None or not re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', msg_name):
                raise ModuleMsgsError("invalid message name '%s'" % msg_name)
            msg_id = _int(m.get('id'), 'message id')
            if not 0 <= msg_id <= MSG_ID_MAX:
                raise ModuleMsgsError("message %s: id %d out of range [0, %d]" % (msg_name, msg_id, MSG_ID_MAX))
            msg_class = m.get('class', 'telemetry')
            if msg_class not in CLASSES:
                raise ModuleMsgsError("message %s: unknown class '%s'" % (msg_name, msg_class))
            fun = m.get('fun')
            if msg_class == 'datalink' and fun is None:
                raise ModuleMsgsError("message %s: datalink messages need a 'fun' callback" % msg_name)
            if fun is not None and not re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', fun):
                raise ModuleMsgsError("message %s: 'fun' is a function name, without arguments" % msg_name)
            fields = []
            for f in m.findall('field'):
                fields.append(Field(f.get('name', ''), f.get('type', ''), f.get('unit'),
                                    f.get('alt_unit'), f.get('alt_unit_coef'),
                                    (f.text or '').strip() or None))
            names = [f.name for f in fields]
            if len(set(names)) != len(names):
                raise ModuleMsgsError("message %s: duplicated field names" % msg_name)
            if any(f.is_var for f in fields[:-1]):
                raise ModuleMsgsError("message %s: only the last field can be a variable length array" % msg_name)
            msg = Message(module, msg_name, msg_id, msg_class, fields, fun,
                          (m.findtext('description') or '').strip() or None)
            if msg.fixed_size > MAX_DATA:
                raise ModuleMsgsError("message %s: %d bytes, max is %d" % (msg_name, msg.fixed_size, MAX_DATA))
            if module.get_message(msg_class, msg_id) is not None:
                raise ModuleMsgsError("message %s: id %d already used in class %s" % (msg_name, msg_id, msg_class))
            if module.get_message_by_name(msg_name) is not None:
                raise ModuleMsgsError("message %s: name already used" % msg_name)
            module.messages.append(msg)
    except ModuleMsgsError as e:
        raise ModuleMsgsError("%s: %s" % (filename, e))
    return module


def check_unique_ids(modules):
    """ Raise ModuleMsgsError if two modules use the same id """
    ids = {}
    errors = []
    for m in modules:
        if m.id in ids and ids[m.id].name != m.name:
            errors.append("module id %d used by '%s' (%s) and '%s' (%s)"
                          % (m.id, ids[m.id].name, ids[m.id].filename, m.name, m.filename))
        ids.setdefault(m.id, m)
    if errors:
        raise ModuleMsgsError("\n".join(errors))


def module_xml_files(paths=None):
    """ All the module XML files from the module paths (conf/modules and PAPARAZZI_MODULES_PATH) """
    if paths is None:
        home = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..')))
        paths = [os.path.join(home, 'conf', 'modules')]
        paths += [os.path.join(p, 'modules') for p in os.getenv('PAPARAZZI_MODULES_PATH', '').split(':') if p]
    files = []
    for p in paths:
        files += sorted(glob.glob(os.path.join(p, '*.xml')))
    return files


def load_modules(files=None):
    """ Parse the module XML files, returns the list of modules declaring messages """
    if files is None:
        files = module_xml_files()
    modules = []
    for f in files:
        m = parse_module(f)
        if m is not None:
            modules.append(m)
    return modules


class ModuleMsgs(object):
    """ Decoder for MODULE_SPECIFIC (class telemetry) and MODULE_SPECIFIC_UP (class datalink) """

    def __init__(self, files=None):
        self.modules = load_modules(files)
        check_unique_ids(self.modules)
        self.by_id = dict((m.id, m) for m in self.modules)
        self.by_name = dict((m.name, m) for m in self.modules)

    def get_message(self, module_id, reserved, msg_class='telemetry'):
        module = self.by_id.get(module_id)
        if module is None:
            return None
        msg_id, _ = split_reserved(reserved)
        return module.get_message(msg_class, msg_id)

    def decode(self, module_id, reserved, data, msg_class='telemetry'):
        """ Returns (module name, message name, instance, OrderedDict of values), or None if unknown """
        msg = self.get_message(module_id, reserved, msg_class)
        if msg is None:
            return None
        _, instance = split_reserved(reserved)
        return msg.module.name, msg.name, instance, msg.decode(data)

    def decode_pprz_message(self, pprz_msg):
        """ Decode a pprzlink PprzMessage MODULE_SPECIFIC or MODULE_SPECIFIC_UP """
        msg_class = 'datalink' if pprz_msg.name == 'MODULE_SPECIFIC_UP' else 'telemetry'
        data = pprz_msg['data']
        if isinstance(data, str):
            data = data.split(',')
        data = [int(x) for x in data if str(x).strip() != '']
        return self.decode(int(pprz_msg['module_id']), int(pprz_msg['reserved']), bytes(data), msg_class)

    def encode(self, module_name, msg_name, values, instance=0):
        """ Returns (module_id, reserved, data bytes) """
        module = self.by_name[module_name]
        msg = module.get_message_by_name(msg_name)
        if msg is None:
            raise ModuleMsgsError("unknown message %s.%s" % (module_name, msg_name))
        return msg.encode(values, instance)
