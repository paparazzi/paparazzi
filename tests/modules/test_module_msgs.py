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
Check the module specific messages (MODULE_SPECIFIC / MODULE_SPECIFIC_UP) of all modules:
the <messages> nodes are valid, the module ids are unique, and every message can be encoded and decoded.
"""

import sys
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../')))
sys.path.insert(0, path.join(PPRZ_HOME, "sw", "lib", "python"))

import TAP.Simple as T  # noqa: E402
import module_msgs as mm  # noqa: E402


def sample_values(msg):
    values = {}
    for i, f in enumerate(msg.fields):
        n = f.array_len if (f.is_array and not f.is_var) else 3
        if f.base == 'char':
            v = ['abcdefgh'[j % 8] for j in range(n)]
        elif f.base in ('float', 'double'):
            v = [0.5 * (i + j + 1) for j in range(n)]
        else:
            v = [(i + j + 1) for j in range(n)]
        values[f.name] = v if f.is_array else v[0]
    return values


def normalize(v):
    if isinstance(v, (list, tuple)):
        return [normalize(x) for x in v]
    return v.decode() if isinstance(v, bytes) else v


if __name__ == "__main__":
    files = mm.module_xml_files([path.join(PPRZ_HOME, "conf", "modules")])

    modules = []
    errors = []
    for f in files:
        try:
            m = mm.parse_module(f)
            if m is not None:
                modules.append(m)
        except mm.ModuleMsgsError as e:
            errors.append(str(e))

    messages = [msg for m in modules for msg in m.messages]
    T.plan(2 + len(messages))

    T.ok(not errors, "module messages are valid")
    for e in errors:
        T.diag(e)

    try:
        mm.check_unique_ids(modules)
        T.ok(True, "module ids are unique (%d modules with messages)" % len(modules))
        codec = mm.ModuleMsgs([m.filename for m in modules])
    except mm.ModuleMsgsError as e:
        T.ok(False, "module ids are unique")
        T.diag(str(e))
        codec = None

    # encode and decode a message with the pprzlink python lib
    for msg in messages:
        name = "%s %s encode/decode" % (msg.module.name, msg.direction)
        if codec is None:
            T.ok(False, name)
            continue
        values = sample_values(msg)
        try:
            carrier = codec.encode(msg.module.name, values, ac_id=(1 if msg.direction == 'uplink' else None))
            decoded = codec.decode(carrier)
            T.eq_ok(dict((f, normalize(decoded[f])) for f in decoded.fieldnames), values, name)
        except Exception as e:
            T.ok(False, name)
            T.diag(str(e))
