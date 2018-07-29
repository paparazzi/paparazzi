#!/usr/bin/env python
#
# Copyright (C) 2012-2014 The Paparazzi Team
#               2015 Freek van Tienen <freek.v.tienen@gmail.com>
#               2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

class Disco(ParrotUtils):
    uav_name = 'Disco'
    address = '192.168.42.1'
    version_file = '/version.txt'
    upload_path = '/data/ftp/'
    check_version_before_run = True
    update_time_before_run = True

    def uav_status(self):
        print('Parrot version:\t\t' + str(self.check_version()))

    def init_extra_parser(self):

        # Parse the extra arguments
        self.parser.add_argument('--min_version', metavar='MIN', default='1.4.1',
                help='force minimum version allowed')
        self.parser.add_argument('--max_version', metavar='MAX', default='1.7.1',
                help='force maximum version allowed')

    def parse_extra_args(self, args):
        # nothing here
        pass


if __name__ == "__main__":
    disco = Disco()
    disco.parse_args()
    exit(0)

