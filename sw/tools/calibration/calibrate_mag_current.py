#! /usr/bin/env python

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
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#


import sys
import os
from optparse import OptionParser

import calibration_utils


def main():
    usage = "usage: %prog [options] log_filename.data" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-i", "--id", dest="ac_id",
                      action="store",
                      help="aircraft id to use")
    parser.add_option("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()
    options.sensor = "MAG"

    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if os.path.isfile(args[0]):
            filename = args[0]
        else:
            print(args[0] + " not found")
            sys.exit(1)

    ac_ids = calibration_utils.get_ids_in_log(filename)
    if options.ac_id is None:
        if len(ac_ids) == 1:
            options.ac_id = ac_ids[0]
        else:
            parser.error("More than one aircraft id found in log file. Specify the id to use.")
    if options.verbose:
        print("Using aircraft id "+options.ac_id)

    if not filename.endswith(".data"):
        parser.error("Please specify a *.data log file")
    if options.verbose:
        print("reading file "+filename+" for aircraft "+options.ac_id+" and sensor "+options.sensor)

    # read raw measurements from log file
    measurements = calibration_utils.read_log_mag_current(options.ac_id, filename)
    if len(measurements) == 0:
        print("Error: found zero IMU_MAG_CURRENT_CALIBRATION measurements for aircraft with id "+options.ac_id+" in log file!")
        sys.exit(1)
    if options.verbose:
        print("found "+str(len(measurements))+" records")

    coefficient = calibration_utils.estimate_mag_current_relation(measurements)

    print("")
    print("<define name= \"MAG_X_CURRENT_COEF\" value=\""+str(coefficient[0])+"\"/>")
    print("<define name= \"MAG_Y_CURRENT_COEF\" value=\""+str(coefficient[1])+"\"/>")
    print("<define name= \"MAG_Z_CURRENT_COEF\" value=\""+str(coefficient[2])+"\"/>")


if __name__ == "__main__":
    main()
