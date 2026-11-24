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
Print the MODULE_SPECIFIC and MODULE_SPECIFIC_UP messages on the Ivy bus,
decoded with the <messages> description of the module XML files.
"""

import argparse
import sys
import time
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface  # noqa: E402
import module_msgs as mm  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-b', '--ivy_bus', default=None, help="Ivy bus (default from IVY_BUS)")
    parser.add_argument('-m', '--module', default=None, help="only show the messages of this module")
    args = parser.parse_args()

    decoder = mm.ModuleMsgs()

    def callback(ac_id, msg):
        if msg.name not in ('MODULE_SPECIFIC', 'MODULE_SPECIFIC_UP'):
            return
        if msg.name == 'MODULE_SPECIFIC_UP':
            ac_id = msg['ac_id']
        try:
            res = decoder.decode_pprz_message(msg)
        except mm.ModuleMsgsError as e:
            print("%s %s: %s" % (ac_id, msg.name, e))
            return
        if res is None:
            if args.module is None:
                print("%s %s unknown module_id=%s reserved=%s" % (ac_id, msg.name, msg['module_id'], msg['reserved']))
            return
        module, name, instance, values = res
        if args.module is not None and module != args.module:
            return
        inst = "[%d]" % instance if instance else ""
        print("%s %s%s.%s %s" % (ac_id, module, inst, name, " ".join("%s=%s" % kv for kv in values.items())))

    kwargs = {} if args.ivy_bus is None else {'ivy_bus': args.ivy_bus}
    ivy = IvyMessagesInterface("ModuleMsgsViewer", **kwargs)
    ivy.subscribe(callback)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    ivy.shutdown()


if __name__ == '__main__':
    main()
