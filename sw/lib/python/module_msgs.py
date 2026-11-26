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

A module can declare one message to the ground and one from the ground in its XML file,
with the same fields as a pprzlink message:

    <messages>
      <downlink>
        <field name="position" type="uint16"/>
        <field name="cg" type="float[3]" unit="m"/>
      </downlink>
      <uplink fun="foo_parse_uplink">
        <field name="mass" type="float" unit="kg"/>
      </uplink>
    </messages>

At the top level make, the messages of all the modules are added to var/messages.xml, as the
classes `module_telemetry` and `module_datalink`: the message name is the module name (upper case),
and the message id is the module id, assigned automatically (modules sorted by name). It is sent
in the `module_id` field of MODULE_SPECIFIC / MODULE_SPECIFIC_UP. As var/messages.xml is part of
the aircraft configuration, the aircraft MD5 changes when an id changes, and logs keep the
messages.xml they were recorded with.

The `data` field holds the fields encoded as a pprzlink payload (little endian, a uint8 length
before a variable array), so they are decoded with the pprzlink parser.

This file is used to generate messages.xml and the airborne header (sw/tools/generators/gen_module_msgs.py),
and to decode the messages on the ground.
"""

import glob
import os
import re
import sys
from xml.etree import ElementTree

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..')))

# types supported by pprzlink (python lib): name -> (size, C type)
TYPES = {
    'int8': (1, 'int8_t'),
    'uint8': (1, 'uint8_t'),
    'int16': (2, 'int16_t'),
    'uint16': (2, 'uint16_t'),
    'int32': (4, 'int32_t'),
    'uint32': (4, 'uint32_t'),
    'float': (4, 'float'),
    'double': (8, 'double'),
    'char': (1, 'char'),
}

DIRECTIONS = ('downlink', 'uplink')
# direction -> (pprzlink class of the decoded message, pprzlink message carrying it)
PPRZ_CLASSES = {'downlink': ('module_telemetry', 'MODULE_SPECIFIC'),
                'uplink': ('module_datalink', 'MODULE_SPECIFIC_UP')}

# Max size of the data array: a pprzlink v2 frame is at most 255 bytes,
# the MODULE_SPECIFIC_UP header (ac_id, module_id, reserved, data length) takes 6 of them
MAX_DATA = 240
# message ids of a pprzlink class are a uint8 in messages.xml (pprz_schema.xsd)
MODULE_ID_MAX = 255
FIELD_ATTRIBS = ('name', 'type', 'unit', 'alt_unit', 'alt_unit_coef', 'values', 'format')


class ModuleMsgsError(Exception):
    pass


class Field(object):
    def __init__(self, xml):
        self.xml = xml
        self.name = xml.get('name', '')
        self.type_str = xml.get('type', '')
        m = re.match(r'^([a-z0-9]+)(\[(\d*)\])?$', self.type_str)
        if m is None or m.group(1) not in TYPES:
            raise ModuleMsgsError("unknown type '%s' for field '%s' (types: %s)"
                                  % (self.type_str, self.name, ", ".join(TYPES)))
        if not re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', self.name):
            raise ModuleMsgsError("invalid field name '%s'" % self.name)
        self.base = m.group(1)
        self.is_array = m.group(2) is not None
        self.array_len = int(m.group(3)) if self.is_array and m.group(3) else None
        self.is_var = self.is_array and self.array_len is None
        if self.is_array and self.array_len == 0:
            raise ModuleMsgsError("field '%s': fixed arrays need a length > 0" % self.name)

    @property
    def elem_size(self):
        return TYPES[self.base][0]

    @property
    def c_type(self):
        return TYPES[self.base][1]

    @property
    def size(self):
        """ size in bytes, only the length byte for a variable array """
        if self.is_var:
            return 1
        return self.elem_size * (self.array_len or 1)


class ModuleMsg(object):
    """ The downlink or uplink message of a module """

    def __init__(self, module, direction, fields, fun=None):
        self.module = module
        self.direction = direction
        self.fields = fields
        self.fun = fun

    @property
    def fixed_size(self):
        """ size without the elements of the variable array """
        return sum(f.size for f in self.fields)

    @property
    def var_field(self):
        if self.fields and self.fields[-1].is_var:
            return self.fields[-1]
        return None


class Module(object):
    def __init__(self, name, filename):
        self.name = name
        self.id = None  # assigned by assign_ids()
        self.filename = filename
        self.downlink = None
        self.uplink = None

    @property
    def messages(self):
        return [m for m in (self.downlink, self.uplink) if m is not None]


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
    try:
        if node.get('id') is not None:
            raise ModuleMsgsError("the module id is assigned automatically, remove messages@id")
        name = root.get('name')
        if not re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', name or ''):
            raise ModuleMsgsError("module name '%s' can't be used in C names" % name)
        module = Module(name, filename)
        for direction in DIRECTIONS:
            nodes = node.findall(direction)
            if not nodes:
                continue
            if len(nodes) > 1:
                raise ModuleMsgsError("only one %s message per module" % direction)
            d = nodes[0]
            fields = [Field(f) for f in d.findall('field')]
            names = [f.name for f in fields]
            if len(set(names)) != len(names):
                raise ModuleMsgsError("%s: duplicated field names" % direction)
            if any(f.is_var for f in fields[:-1]):
                raise ModuleMsgsError("%s: only the last field can be a variable length array" % direction)
            fun = d.get('fun')
            if direction == 'uplink' and fun is None:
                raise ModuleMsgsError("uplink: a 'fun' callback is needed")
            if fun is not None and not re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', fun):
                raise ModuleMsgsError("uplink: 'fun' is a function name, without arguments")
            msg = ModuleMsg(module, direction, fields, fun)
            if msg.fixed_size > MAX_DATA:
                raise ModuleMsgsError("%s: %d bytes, max is %d" % (direction, msg.fixed_size, MAX_DATA))
            setattr(module, direction, msg)
    except ModuleMsgsError as e:
        raise ModuleMsgsError("%s: %s" % (filename, e))
    return module


def assign_ids(modules):
    """
    Assign the module ids: modules sorted by name, from 1.
    Raise ModuleMsgsError if two modules have the same name (case insensitive) or if there are too many
    """
    by_name = {}
    errors = []
    for m in modules:
        key = m.name.upper()
        if key in by_name:
            errors.append("module '%s' (%s) and '%s' (%s) declare messages with the same name"
                          % (by_name[key].name, by_name[key].filename, m.name, m.filename))
        by_name[key] = m
    if errors:
        raise ModuleMsgsError("\n".join(errors))
    if len(by_name) > MODULE_ID_MAX:
        raise ModuleMsgsError("%d modules with messages, max is %d" % (len(by_name), MODULE_ID_MAX))
    for i, key in enumerate(sorted(by_name)):
        by_name[key].id = i + 1
    return sorted(modules, key=lambda m: m.id)


def read_ids(messages_file):
    """ The module ids from a messages.xml file: dict module name (upper case) -> id """
    try:
        root = ElementTree.parse(messages_file).getroot()
    except (IOError, ElementTree.ParseError) as e:
        raise ModuleMsgsError("can't read %s: %s" % (messages_file, e))
    ids = {}
    for direction in DIRECTIONS:
        cls = root.find("msg_class[@name='%s']" % PPRZ_CLASSES[direction][0])
        if cls is None:
            continue
        for msg in cls.findall('message'):
            ids[msg.get('name')] = int(msg.get('id'))
    return ids


def set_ids(modules, messages_file):
    """ Set the module ids from a messages.xml file (generated by the top level make) """
    ids = read_ids(messages_file)
    missing = [m.name for m in modules if m.name.upper() not in ids]
    if missing:
        raise ModuleMsgsError("module(s) %s not in %s, run 'make' at the top level to update it"
                              % (", ".join(missing), messages_file))
    for m in modules:
        m.id = ids[m.name.upper()]
    return modules


def module_xml_files(paths=None):
    """ All the module XML files from the module paths (conf/modules and PAPARAZZI_MODULES_PATH) """
    if paths is None:
        paths = [os.path.join(PPRZ_HOME, 'conf', 'modules')]
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


def pprzlink_classes(modules, class_ids):
    """ The module messages as pprzlink message classes, XML elements """
    classes = []
    for direction in DIRECTIONS:
        cls_name = PPRZ_CLASSES[direction][0]
        cls = ElementTree.Element('msg_class', name=cls_name, id=str(class_ids[cls_name]))
        for module in modules:
            msg = getattr(module, direction)
            if msg is None:
                continue
            m = ElementTree.SubElement(cls, 'message', name=module.name.upper(), id=str(module.id))
            for f in msg.fields:
                attribs = dict((a, f.xml.get(a)) for a in FIELD_ATTRIBS if f.xml.get(a) is not None)
                field = ElementTree.SubElement(m, 'field', attribs)
                if f.xml.text and f.xml.text.strip():
                    field.text = f.xml.text.strip()
        classes.append(cls)
    return classes


def merge_messages(base_file, modules):
    """
    The text of base_file (a pprzlink messages.xml) with the module messages classes added at the end,
    the module ids must be assigned. The base file is kept as is, with its comments.
    """
    with open(base_file) as f:
        text = f.read()
    try:
        root = ElementTree.fromstring(text.encode())
    except ElementTree.ParseError as e:
        raise ModuleMsgsError("%s: %s" % (base_file, e))
    names = [c.get('name') for c in root.findall('msg_class')]
    for direction in DIRECTIONS:
        if PPRZ_CLASSES[direction][0] in names:
            raise ModuleMsgsError("%s already has a %s class" % (base_file, PPRZ_CLASSES[direction][0]))
    next_id = max([int(c.get('id')) for c in root.findall('msg_class')] + [0]) + 1
    class_ids = {}
    for direction in DIRECTIONS:
        class_ids[PPRZ_CLASSES[direction][0]] = next_id
        next_id += 1
    end = text.rfind('</protocol>')
    if end < 0:
        raise ModuleMsgsError("%s: no </protocol>" % base_file)
    out = " <!-- module specific messages, added from the module XML files by gen_module_msgs.py,\n"
    out += "      carried by MODULE_SPECIFIC (module_telemetry) and MODULE_SPECIFIC_UP (module_datalink) -->\n"
    for cls in pprzlink_classes(modules, class_ids):
        if len(cls) == 0:
            continue  # a class needs at least one message (messages.dtd)
        ElementTree.indent(cls, space="  ", level=1)
        out += " " + ElementTree.tostring(cls, encoding='unicode').rstrip() + "\n\n"
    return text[:end] + out + text[end:]


class ModuleMsgs(object):
    """
    Decode MODULE_SPECIFIC (telemetry) and MODULE_SPECIFIC_UP (datalink) pprzlink messages
    into messages of the classes module_telemetry / module_datalink, with the pprzlink python lib
    (from var/messages.xml, or the messages_file given, e.g. the one of a log)
    """

    def __init__(self, messages_file=None):
        if PPRZ_HOME + "/var/lib/python" not in sys.path:
            sys.path.append(PPRZ_HOME + "/var/lib/python")
        from pprzlink import messages_xml_map
        if messages_file is not None:
            messages_xml_map.parse_messages(messages_file)
        else:
            messages_xml_map._ensure_message_dictionary()
        self.id_name = {}
        self.name_id = {}
        for direction in DIRECTIONS:
            cls = PPRZ_CLASSES[direction][0]
            self.id_name[direction] = dict(messages_xml_map.message_dictionary_id_name.get(cls, {}))
            self.name_id[direction] = dict(messages_xml_map.message_dictionary_name_id.get(cls, {}))
        if not any(self.id_name.values()):
            raise ModuleMsgsError("no module messages in messages.xml")

    def decode(self, pprz_msg):
        """
        Decode a MODULE_SPECIFIC or MODULE_SPECIFIC_UP PprzMessage, returns a PprzMessage
        of class module_telemetry / module_datalink, or None if it is not a known module message
        """
        from pprzlink.message import PprzMessage
        if pprz_msg.name not in ('MODULE_SPECIFIC', 'MODULE_SPECIFIC_UP'):
            return None
        direction = 'uplink' if pprz_msg.name == 'MODULE_SPECIFIC_UP' else 'downlink'
        name = self.id_name[direction].get(int(pprz_msg['module_id']))
        if name is None:
            return None
        data = pprz_msg['data']
        if isinstance(data, str):
            data = data.split(',')
        data = bytes(int(x) for x in data if str(x).strip() != '')
        msg = PprzMessage(PPRZ_CLASSES[direction][0], name)
        try:
            msg.binary_to_payload(data)
        except Exception as e:
            raise ModuleMsgsError("%s %s: can't decode %s (%s)" % (name, direction, data.hex(), e))
        return msg

    def encode(self, module_name, values, ac_id=None):
        """
        Encode the uplink message of a module (the downlink one if ac_id is None) from a dict of values,
        returns a MODULE_SPECIFIC_UP (or MODULE_SPECIFIC) PprzMessage
        """
        from pprzlink.message import PprzMessage
        direction = 'downlink' if ac_id is None else 'uplink'
        name = module_name.upper()
        if name not in self.name_id[direction]:
            raise ModuleMsgsError("module %s has no %s message" % (module_name, direction))
        cls, carrier = PPRZ_CLASSES[direction]
        inner = PprzMessage(cls, name)
        inner.set_values([values[f] for f in inner.fieldnames])
        data = inner.payload_to_binary()
        if len(data) > MAX_DATA:
            raise ModuleMsgsError("%s: data too long (%d bytes)" % (module_name, len(data)))
        msg = PprzMessage('telemetry' if ac_id is None else 'datalink', carrier)
        if ac_id is not None:
            msg['ac_id'] = ac_id
        msg['module_id'] = self.name_id[direction][name]
        msg['reserved'] = 0
        msg['data'] = list(data)
        return msg
