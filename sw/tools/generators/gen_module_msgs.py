#!/usr/bin/env python3
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
Generate the airborne header for the module specific messages (MODULE_SPECIFIC / MODULE_SPECIFIC_UP)
of the modules loaded in an aircraft, see sw/lib/python/module_msgs.py.

  gen_module_msgs.py -o modules_msgs.h module1.xml module2.xml ...   generate the header
  gen_module_msgs.py --check [module.xml ...]                        check that module ids are unique
                                                                      (all modules if no file given)
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../lib/python'))
import module_msgs as mm  # noqa: E402


def c_name(s):
    return s.replace('.', '_').replace('-', '_')


def prefix(msg):
    return "MODULE_MSG_%s_%s" % (c_name(msg.module.name).upper(), msg.name.upper())


def lower_name(msg):
    return "%s_%s" % (c_name(msg.module.name).lower(), msg.name.lower())


def module_id_name(module):
    return "MODULE_MSGS_ID_%s" % c_name(module.name).upper()


def gen_struct(msg):
    out = "struct __attribute__((packed)) module_msg_%s {\n" % lower_name(msg)
    for f in msg.fields:
        if f.is_var:
            out += "  %s %s[0];\n" % (f.c_type, f.name)
        elif f.is_array:
            out += "  %s %s[%d];\n" % (f.c_type, f.name, f.array_len)
        else:
            out += "  %s %s;\n" % (f.c_type, f.name)
    out += "};\n"
    return out


def gen_send(msg):
    p = prefix(msg)
    args = []
    for f in msg.fields:
        if f.is_var:
            args.append(("uint8_t ", "nb_" + f.name))
            args.append(("const %s *" % f.c_type, f.name))
        elif f.is_array:
            args.append(("const %s *" % f.c_type, f.name))
        else:
            args.append(("%s " % f.c_type, f.name))
    params = "".join(", %s%s" % (t, n) for t, n in args)
    unused = "".join(", %s%s __attribute__((unused))" % (t, n) for t, n in args)
    fname = "module_msg_send_%s" % lower_name(msg)
    buf_size = "MODULE_MSGS_MAX_DATA" if msg.var_field else "%s_SIZE" % p
    out = "#if DOWNLINK\n"
    out += "static inline void %s(struct transport_tx *trans, struct link_device *dev%s)\n{\n" % (fname, params)
    out += "  uint8_t _data[%s > 0 ? %s : 1];\n" % (buf_size, buf_size)
    out += "  uint8_t _len = 0;\n"
    for f in msg.fields:
        if f.is_var:
            out += "  if (nb_%s > (MODULE_MSGS_MAX_DATA - _len) / %d) { nb_%s = (MODULE_MSGS_MAX_DATA - _len) / %d; }\n" \
                % (f.name, f.elem_size, f.name, f.elem_size)
            out += "  memcpy(&_data[_len], %s, nb_%s * %d);\n" % (f.name, f.name, f.elem_size)
            out += "  _len += nb_%s * %d;\n" % (f.name, f.elem_size)
        elif f.is_array:
            out += "  memcpy(&_data[_len], %s, %d);\n" % (f.name, f.size)
            out += "  _len += %d;\n" % f.size
        else:
            out += "  memcpy(&_data[_len], &%s, %d);\n" % (f.name, f.size)
            out += "  _len += %d;\n" % f.size
    out += "  uint16_t _module_id = %s;\n" % module_id_name(msg.module)
    out += "  uint16_t _reserved = MODULE_MSGS_RESERVED(%s_ID, 0);\n" % p
    out += "  pprz_msg_send_MODULE_SPECIFIC(trans, dev, AC_ID, &_module_id, &_reserved, _len, _data);\n"
    out += "}\n"
    if args:
        out += "#define DOWNLINK_SEND_%s(_trans, _dev, ...) %s(&((_trans).trans_tx), &((_dev).device), __VA_ARGS__)\n" \
            % (p, fname)
    else:
        out += "#define DOWNLINK_SEND_%s(_trans, _dev) %s(&((_trans).trans_tx), &((_dev).device))\n" % (p, fname)
    out += "#else\n"
    out += "static inline void %s(struct transport_tx *trans __attribute__((unused)), " \
           "struct link_device *dev __attribute__((unused))%s) {}\n" % (fname, unused)
    out += "#define DOWNLINK_SEND_%s(_trans, _dev, ...) {}\n" % p
    out += "#endif\n"
    return out


def gen_message(msg):
    p = prefix(msg)
    out = "\n/* %s.%s (%s)%s */\n" % (msg.module.name, msg.name, msg.msg_class,
                                     (": " + msg.description) if msg.description else "")
    out += "#define %s_ID %d\n" % (p, msg.id)
    out += "#define %s_SIZE %d\n" % (p, msg.fixed_size)
    if msg.var_field:
        f = msg.var_field
        out += "#define %s_%s_LENGTH(_len) (((_len) - %s_SIZE) / %d)\n" % (p, f.name, p, f.elem_size)
    if msg.fields:
        out += gen_struct(msg)
    if msg.msg_class == 'telemetry':
        out += gen_send(msg)
    else:
        if msg.fields:
            out += "extern void %s(uint8_t instance, const struct module_msg_%s *msg, uint8_t len);\n" \
                % (msg.fun, lower_name(msg))
        else:
            out += "extern void %s(uint8_t instance);\n" % msg.fun
    return out


def gen_uplink_parser(modules):
    out = "\n/** Parse a MODULE_SPECIFIC_UP message and call the callback of the module */\n"
    out += "static inline void module_msgs_parse_uplink(uint8_t *buf)\n{\n"
    out += "  if (DL_MODULE_SPECIFIC_UP_ac_id(buf) != AC_ID) { return; }\n"
    out += "  uint16_t module_id = DL_MODULE_SPECIFIC_UP_module_id(buf);\n"
    out += "  uint16_t reserved = DL_MODULE_SPECIFIC_UP_reserved(buf);\n"
    out += "  uint8_t msg_id = MODULE_MSGS_MSG_ID(reserved);\n"
    out += "  uint8_t instance __attribute__((unused)) = MODULE_MSGS_INSTANCE(reserved);\n"
    out += "  uint8_t len __attribute__((unused)) = DL_MODULE_SPECIFIC_UP_data_length(buf);\n"
    out += "  uint8_t *data __attribute__((unused)) = DL_MODULE_SPECIFIC_UP_data(buf);\n"
    out += "  switch (module_id) {\n"
    for module in modules:
        msgs = [m for m in module.messages if m.msg_class == 'datalink']
        if not msgs:
            continue
        out += "    case %s:\n" % module_id_name(module)
        out += "      switch (msg_id) {\n"
        for msg in msgs:
            p = prefix(msg)
            out += "        case %s_ID:\n" % p
            if msg.fields:
                out += "          if (len >= %s_SIZE) { %s(instance, (const struct module_msg_%s *)data, len); }\n" \
                    % (p, msg.fun, lower_name(msg))
            else:
                out += "          %s(instance);\n" % msg.fun
            out += "          break;\n"
        out += "        default: break;\n"
        out += "      }\n"
        out += "      break;\n"
    out += "    default: break;\n"
    out += "  }\n"
    out += "}\n"
    return out


def gen_header(modules):
    out = "/* This file has been generated by gen_module_msgs.py from the module XML files */\n"
    out += "/* Please DO NOT EDIT */\n\n"
    out += "#ifndef MODULES_MSGS_H\n#define MODULES_MSGS_H\n\n"
    out += "#include \"std.h\"\n#include <string.h>\n"
    out += "#include \"generated/airframe.h\"\n"
    out += "#include \"pprzlink/messages.h\"\n"
    out += "#include \"pprzlink/dl_protocol.h\"\n\n"
    out += "/** Max size of the data of a module message */\n"
    out += "#define MODULE_MSGS_MAX_DATA %d\n" % mm.MAX_DATA
    out += "/** The reserved field holds the message id (low byte) and the module instance (high byte) */\n"
    out += "#define MODULE_MSGS_RESERVED(_msg_id, _instance) ((uint16_t)(((_msg_id) & 0xFF) | (((_instance) & 0xFF) << 8)))\n"
    out += "#define MODULE_MSGS_MSG_ID(_reserved) ((uint8_t)((_reserved) & 0xFF))\n"
    out += "#define MODULE_MSGS_INSTANCE(_reserved) ((uint8_t)(((_reserved) >> 8) & 0xFF))\n\n"
    for module in modules:
        out += "#define %s %d\n" % (module_id_name(module), module.id)
    for module in modules:
        for msg in module.messages:
            out += gen_message(msg)
    if any(m.msg_class == 'datalink' for module in modules for m in module.messages):
        out += "\n#define MODULE_MSGS_UPLINK 1\n"
        out += gen_uplink_parser(modules)
    out += "\n#endif /* MODULES_MSGS_H */\n"
    return out


def main():
    parser = argparse.ArgumentParser(description="Generate the module specific messages header")
    parser.add_argument('-o', '--output', help="output header file")
    parser.add_argument('--check', action='store_true',
                        help="only check the module XML files (all modules if no file given)")
    parser.add_argument('files', nargs='*', help="module XML files")
    args = parser.parse_args()

    try:
        files = args.files
        if args.check and not files:
            files = mm.module_xml_files()
        modules = mm.load_modules(files)
        mm.check_unique_ids(modules)
    except mm.ModuleMsgsError as e:
        print("Module messages error: %s" % e, file=sys.stderr)
        sys.exit(1)

    if args.check:
        print("%d modules with messages, ids are unique" % len(modules))
        return
    if args.output is None:
        parser.error("an output file is needed")
    header = gen_header(modules)
    # only write if changed, to avoid rebuilds
    try:
        with open(args.output) as f:
            if f.read() == header:
                return
    except IOError:
        pass
    with open(args.output, 'w') as f:
        f.write(header)


if __name__ == '__main__':
    main()
