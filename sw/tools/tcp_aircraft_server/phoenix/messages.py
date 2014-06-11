#Copyright 2014, Antoine Drouin
# vim: ai ts=4 sts=4 et sw=4

import re
import os.path
import struct
import logging
import  pdb

import phoenix
from . import xmlobject

LOG = logging.getLogger('PhoenixMessages')
LOG.setLevel(logging.DEBUG)
#ch = logging.StreamHandler()
#ch.setLevel(logging.DEBUG)
#LOG.addHandler(ch)

#################################################################################
#
#
#
#
#################################################################################
class Field:
    """
    **Attributes:**
        - name   : field name

        - ctype  : string representing the C type of the field,
                   i.e. uint8[ARRAY_LEN] or uint8[]
        - type   : string representing the C type of a single
                   instance of the field, i.e. uint8

        - is_array :

        - num_elements :

        - length : the number of bytes to store the field,
                   i.e. sizeof(uint8) * ARRAY_LEN
        - pytype : the python type used to store the field value
    """

    ARRAY_RE = re.compile("^([a-z0-9]+)\[(\d{,})\]$")

    def __init__(self, f, **kwargs):
        self.name = f.name
        self.ctype = f.type
        m = self.ARRAY_RE.match(f.type)
        #check if it is an array
        if m:
            self.is_array = True
            elmt_type, elmt_nb = m.groups()
            self.type = elmt_type
            self.element_length = phoenix.TYPE_TO_LENGTH_MAP[elmt_type]
            if elmt_nb != '':
                self.num_elements = int(elmt_nb)
                self.length = phoenix.TYPE_TO_LENGTH_MAP[elmt_type] * self.num_elements
            else:
                self.num_elements = -1
                self.length = -1
        else:
            self.is_array = False
            self.type = f.type
            self.num_elements = 1
            self.length = phoenix.TYPE_TO_LENGTH_MAP[f.type]
            self.element_length = None

        #cache the python type for speed
        if self.type == "float" or self.type == "double":
            self.pytype = float
        elif self.type == "char":
            if self.is_array:
                self.pytype = str
            else:
                self.pytype = chr
        else:
            self.pytype = int


    def __str__(self):
        return "<Field: %s (%s)>" % (self.name, self.ctype)

#################################################################################
#
#
#
#
#################################################################################
class Message:
    """
    **Attributes:**
        - id: message integer id (from messages.xml)
        - name: message name (upper case) (from messages.xml)
        - fields: a list of :class:`Field` object (type according to field_class)
        - size: the length of the message (in bytes)
        - num_values: the total number of values in the message (as a
          field can have multiple elements, such as if it is an array)
        - num_fields: the number of fields in the message
        - doc: documentation string
        - is_command: is the message a command (i.e. needs to be ACK'd)
    """
    def __init__(self, m, field_klass, **field_kwargs):
        self._field_kwargs = field_kwargs
        self.name = m.name.upper()
        if int(m.id) <= 255 and int(m.id) > 0:
            self.id = int(m.id)
        else:
            raise Exception("Message IDs must be 0 > ID <= 255")

        try:
            self.fields = [field_klass(f, **self._field_kwargs) for f in xmlobject.ensure_list(m.field)]
        except AttributeError:
            self.fields = []

        try:
            self.is_command = m.command == "1"
        except AttributeError:
            self.is_command = False

        try:
            self.doc = m.doc
        except AttributeError:
            self.doc = ""

        self.size = 0
        self.num_values = 0
        for f in self.fields:
            self.size += f.length
            self.num_values += f.num_elements

        self.num_fields = len(self.fields)

    @property
    def pretty_name(self):
        return self.name.replace("_"," ").title()

    def __str__(self):
        return "<Message: %s (%s)>" % (self.name, self.id)

#################################################################################
#
#
#
#
#################################################################################
class PyField(Field):
    """
    A pythonic object representing a field in a message

    **Attributes:**
        - is_enum: True if the Field is an enum
        - struct_format: the :mod:`struct` format string for this type
        - coef: the unit coefficient for this type (from messages.xml)
    """

    #: maps user defined names to python struct compatible ids
    TYPE_TO_STRUCT_MAP = {
            "char"  :   "B",    #treat char as uint8 internally, only modify how they are displayed
            "uint8" :   "B",
            "int8"  :   "b",
            "uint16":   "H",
            "int16" :   "h",
            "uint32":   "I",
            "int32" :   "i",
            "float" :   "f",
            "double":   "f"
    }
    #: maps type to range of acceptible values
    TYPE_TO_RANGE_MAP = {
            "char"  :   (0,2**8-1),
            "uint8" :   (0,2**8-1),
            "int8"  :   (-2**(8-1),2**(8-1)-1),
            "uint16":   (0,2**16-1),
            "int16" :   (-2**(16-1),2**(16-1)-1),
            "uint32":   (0,2**32-1),
            "int32" :   (-2**(32-1),2**(32-1)-1),
            "float" :   (-3.4e38,3.4e38),    #not exactly correct
            "double":   (-3.4e38,3.4e38)     #not exactly correct
    }

    def __init__(self, node, **kwargs):
        Field.__init__(self, node, **kwargs)

        shared_values = kwargs["shared_values"]
        self.display_real_unit = False #kwargs["display_real_unit"]

        self.is_enum = False
        if self.type == "uint8":
            try:
                values = node.values
                if values[0] == "@":
                    self._enum_values = shared_values[values[1:]]
                else:
                    self._enum_values = values.split("|")
                self.is_enum = True
            except AttributeError:
                self._enum_values = []
            except KeyError as e:
                raise Exception("Referenced value does not exist: %s", e)

        if self.is_array:
            if self.num_elements > 0:
                self.struct_format = "%d%s" % (self.num_elements, self.TYPE_TO_STRUCT_MAP[self.type])
                self._size = struct.calcsize(self.struct_format)
            else:
            # variable size array, FIXME :(
                self.struct_format = ""
                self._size = -1
        else:
            self.struct_format = self.TYPE_TO_STRUCT_MAP[self.type]
            self._size = struct.calcsize(self.struct_format)

        try:
            self._fstr = node.format
        except AttributeError:
            self._fstr = "%s"
            #if self.is_array:
            #    self._fstr = "%s"
            #else:
            #    self._fstr = phoenix.TYPE_TO_PRINT_MAP[self.pytype]

        try:
            self._fstr += " %s" % node.unit
        except AttributeError:
            pass

        # if an alternate unit is provided,
        # update display accordingly
        try:
            alt_unit = node.alt_unit
        except AttributeError:
            alt_unit = ""

        try:
            alt_unit_format = node.alt_unit_format
        except AttributeError:
            alt_unit_format = "%s"

        try:
            self.alt_unit_coef = float(node.alt_unit_coef)
            if self.display_real_unit:
                self._fstr = alt_unit_format + " " + alt_unit + " ("+ self._fstr +")"
            else:
                self._fstr = alt_unit_format + " " + alt_unit
        except:
            self.alt_unit_coef = None

    def get_default_value(self):
        """ Returns a sensible default value for the type """
        if self.is_array and self.type != "char":
            return list( [self.pytype() for i in range(self.num_elements)] )
        else:
            return self.pytype()

    def interpret_value_from_user_string(self, string, default=None, sep=","):
        """
        Tries to interpret the supplied string as best according to the type
        of the field. For example, the following are legal
            - "foo bar": if field is char[]
            - 5: if field is an integer type
            - 3.4: if field is a float
            - 1,2,3: if field is an array of integers
            - ENUM_VALUE: if field is an enum

        In case of failure, the default value (or the value passed in the
        default argument) is returned
        """
        try:
            if self.is_array and self.type != "char":
                vals = string.split(sep)
                if len(vals) != self.num_elements:
                    raise ValueError
                return list( [self.pytype(v) for v in vals] )
            elif self.is_enum:
                try:
                    #first look if the user suppled a string is an enum value
                    return self._enum_values.index(string)
                except ValueError:
                    #if not, assume it is a number, the constructor of the field
                    #will take care of it
                    return self.pytype(string)
            else:
                return self.pytype(string)
        except ValueError:
            #invalid user input for type
            if default:
                return default
            else:
                return self.get_default_value()

    def get_printable_value(self, value):
        """ Returns a printable string in a human readable format """
        if self.is_array:
            if self.type == "char":
                return "".join([chr(c) for c in value])
            else:
                #Returns a printable array, e.g '[1, 2, 3]'
                return str(value)
        else:
            #Return a single formatted number string
            if self.is_enum:
                #If this is an uint8 enum type then return the
                #enum value
                try:
                    return "%s" % self._enum_values[value]
                except IndexError:
                    return "?%s?" % value
            else:
                if self.alt_unit_coef:
                    if self.display_real_unit:
                        return self._fstr % (self.alt_unit_coef * value, value)
                    else:
                        return self._fstr % (self.alt_unit_coef * value)
                else:
                    return self._fstr % (value)

    def get_scaled_value(self, value):
        """
        Returns the scaled (according to alt_unit_coef). Note, that unlike
        the other get_ functions, this does not respect the original type of
        the field. A float is always returned
        """
        if self.is_array:
            if self.type == "char": return value
            else: return [float(val * self.alt_unit_coef) for val in value]
        else:
            if self.is_enum: return value
            else: return float(value * self.alt_unit_coef) if self.alt_unit_coef else value

    def get_value_range(self):
        """
        Returns the legal values this type is allowed to hold. If the type is
        and enum this returns a n-tuple of all allowed enum values. Otherwise
        this returns a 2-tuple of the minimum and maximum values this
        field can hold
        """
        if self.is_enum:
            return self._enum_values
        else:
            return self.TYPE_TO_RANGE_MAP[self.ctype]

#################################################################################
#
#
#
#
#################################################################################
class PyMessage(Message):
    """
    Represents a message to/from the UAV
    """

    #Messages are packed in the payload in little endian format
    MESSAGE_ENDIANESS = "<"

    def __init__(self, name, id, node, **kwargs):
        Message.__init__(self, node, PyField, **kwargs)
        self._fields_by_name = {}

        format = self.MESSAGE_ENDIANESS
        for f in self.fields:
            format += f.struct_format
            self._fields_by_name[f.name] = f

        #cache the struct for performace reasons
        self._struct = struct.Struct(format)

    def __getitem__(self, key):
        f = self.get_field_by_name(key)
        if not f:
            raise KeyError("Could not find field %s" % key)
        return f

    def get_fields(self):
        """ Returns a list of :class:`PyField` objects """
        return self.fields

    def get_field_by_name(self, name):
        try:
            return self._fields_by_name[name]
        except KeyError:
            return None

    def get_field_index(self, name):
        i = -1
        for f in self.fields:
            i = i + 1
            if f.name == name:
                break
        return i

    def get_field_values(self, vals):
        """
        Returns a list of values for each field in the message. The return
        type is dependent on the type of the field. If the field is an array
        type then the returned list will contain a tuple of the field array
        elements. For example ::

            <message name="TEST_MESSAGE" id="26">
                <field name="a_uint8" type="uint8" values="OK|LOST|REALLY_LOST"/>
                <field name="a_int8" type="int8"/>
                <field name="a_uint16" type="uint16" unit="adc"/>
                <field name="a_int16" type="int16"/>
                <field name="a_uint32" type="uint32" alt_unit="deg/s" alt_unit_coef="0.0139882"/>
                <field name="a_int32" type="int32"/>
                <field name="a_float" type="float"/>
                <field name="a_array" type="uint8[3]"/>
            </message>

        will return the following ::

            [1, -1, 1000, -1000, 100000, -100000, 1.5, (1, 2, 3)]

        """
        i = 0
        v = []
        for f in self.fields:
            ne = f.num_elements
            if f.is_array:
                v.append( vals[i:i+ne] )
            else:
                v.append( vals[i] )
            i += ne
        return v

    def pack_values(self, *values):
        """
        Assemble the list of field values into a message payload string

        :param values: a list of values of the type expected by the appropriate
         field. For example, if the 3rd field in the message is a *uint8* then
         the third value should be an Int
        """
        assert len(values) == self.num_values, "%s != %s" % (len(values), self.num_values)

        if self.fields:
            return self._struct.pack(*values)
        return ""

    def unpack_values(self, string):
        """
        Unlike :func:`get_field_values` this function flattens array
        fields into the returned list. For example ::

            <message name="TEST_MESSAGE" id="26">
                <field name="a_uint8" type="uint8" values="OK|LOST|REALLY_LOST"/>
                <field name="a_int8" type="int8"/>
                <field name="a_uint16" type="uint16" unit="adc"/>
                <field name="a_int16" type="int16"/>
                <field name="a_uint32" type="uint32" alt_unit="deg/s" alt_unit_coef="0.0139882"/>
                <field name="a_int32" type="int32"/>
                <field name="a_float" type="float"/>
                <field name="a_array" type="uint8[3]"/>
            </message>

        will return the following ::

            (1,-1, 1000, -1000, 100000, -100000, 1.5, 1, 2, 3)

        :param string: the message paylod string to unpack
        """
        if self.fields:
            assert type(string) == str
            assert len(string) == self._struct.size, "%s != %s" % (len(string), self._struct.size)
            return self._struct.unpack(string)
        return ()

    def unpack_printable_values(self, string, joiner=" "):
        """
        Returns a string, suitable for printing or displaying to the user,
        of the given message paylod. The string contains values for each
        field in the message

        :param string: the message payload to unpack
        :param joiner: the string used between different message fields
        """
        if self.fields:
            vals = self.unpack_values(string)
            assert len(vals) == self.num_values
            fvals = self.get_field_values(vals)
            assert len(fvals) == self.num_fields

            pvals = [self.fields[i].get_printable_value(fvals[i]) for i in range(self.num_fields)]

            if joiner:
                return joiner.join(pvals)
            else:
                return pvals
        else:
            return ""

    def unpack_scaled_values(self, string):
        """
        As :func:`unpack_values` but returns the values scaled as per the
        unit coefficient
        """
        if self.fields:
            vals = self.unpack_values(string)
            assert len(vals) == self.num_values
            fvals = self.get_field_values(vals)
            assert len(fvals) == self.num_fields

            return [self.fields[i].get_scaled_value(fvals[i]) for i in range(self.num_fields)]
        return ()

    def get_default_values(self):
        """ Returns a list of sensible default values for each field """
        return [ f.get_default_value() for f in self.fields ]


#################################################################################
#
#
#
#
#################################################################################
class MessageClass:
    """
    Represents a class of messages in *messages.xml*
    """
    def __init__(self, x, shared_values):
        self.name = x.name
        self._messages = xmlobject.ensure_list( x.message )
        self._msgs_by_id = {}
        self._msgs_by_name = {}
        for m in self._messages:
            msg = PyMessage(m.name, m.id, m, shared_values=shared_values)
            #print m.name
            self._msgs_by_id[int(m.id)] = msg
            self._msgs_by_name[m.name] = msg

    def get_message_by_name(self, name):
        try:
            return self._msgs_by_name[name]
        except KeyError:
            LOG.error("No message with name {:s} in class {:s}".format(name, self.name))
            return None

    def get_message_by_id(self, id):
        try:
            return self._msgs_by_id[id]
        except KeyError:
            LOG.error("No message with id {:s} in class {:s}".format(id, self.name))
            return None




#################################################################################
#
#
#
#
#################################################################################
class Protocol:
    """
    A pythonic wrapper for parsing *messages.xml*
    """
    def __init__(self, **kwargs):
        """
        **Keywords:**
            - debug - should extra information be printed while parsing
              *messages.xml*
            - path - a pathname from which the file can be read
            - file - an open file object from which the raw xml
              can be read
            - raw - the raw xml itself
            - root - name of root tag, if not reading content
        """
        self._debug = kwargs.get("debug", False)

        path = kwargs.get("path")
        if path and not os.path.exists(path):
            raise Exception("Could not find message file")

        try:
            root = xmlobject.XMLFile(**kwargs).root
        except AttributeError:
            raise Exception("Invalid XML")

        try:
            classes = root.msg_class
        except AttributeError:
            raise Exception("Missing message class")

        # FIXME: remove that
        self._values = {}
        self._msg_classes = {}
        for msg_class in root.msg_class:
            if msg_class.name == "telemetry" or msg_class.name == "datalink":
                self._msg_classes[msg_class.name] = MessageClass(msg_class, self._values)


    def __getitem__(self, key):
        m = self.get_message_by_name(key)
        if not m:
            raise KeyError("Could not find message: %s" % key)
        return m



    def get_messages(self):
        """ Returns a list of :class:`PyMessage` objects """
        return list(self._msgs_by_id.values())

    def get_message_by_name(self, class_name, msg_name):
        try:
            return self._msg_classes[class_name].get_message_by_name(msg_name)
        except KeyError:
            LOG.error("No message class with name {:s}".format(class_name))
            return None

    def get_message_by_id(self, class_name, msg_id):
        try:
            return self._msg_classes[class_name].get_message_by_id(msg_id)
        except KeyError:
            LOG.error("No message class with name {:s}".format(class_name))
            return None


