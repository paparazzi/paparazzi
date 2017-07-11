#!/usr/bin/env python
#
# Copyright (C) 2012-2017 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from __future__ import print_function
from parrot_utils import ParrotUtils

class Swing(ParrotUtils):
    uav_name = 'Swing'
    address = '192.168.4.1'
    version_file = None
    upload_path = '/data/edu/'
    prompt = '$ '

    def uav_status(self):
        print('Parrot version:\t\t' + self.check_version())

    def init_extra_parser(self):
        # nothing here
        pass

    def parse_extra_args(self, args):
        # nothing here
        pass


if __name__ == "__main__":
    swing = Swing()
    swing.parse_args()
    exit(0)

