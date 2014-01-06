#! /usr/bin/env python

#  Copyright (C) 2011 Antoine Drouin
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
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

import os
import sys
import ahrs_utils


def main():

#   traj_nb = 0  # static
#   traj_nb = 1  # sine orientation
#   traj_nb = 2  # sine X quad
#   traj_nb = 3  # step_phi
#   traj_nb = 4  # step_phi_2nd_order
#   traj_nb = 5  # step_bias
#   traj_nb = 6  # coordinated circle
#   traj_nb = 7  # stop stop X quad
    traj_nb = 8  # bungee takeoff

    build_opt1 = []
#   build_opt1 = ["Q="]
    build_opt1 += ["FREQUENCY=120"]
#   build_opt1 += ["PROPAGATE_LOW_PASS_RATES=1"]
    build_opt1 += ["GRAVITY_UPDATE_COORDINATED_TURN=1"]
    build_opt1 += ["GRAVITY_UPDATE_NORM_HEURISTIC=1"]
#   build_opt1 += ["DISABLE_MAG_UPDATE=1"]
#   build_opt1 += ["USE_GPS_HEADING=1"]
#   build_opt1 += ["MAG_UPDATE_YAW_ONLY=1"]

#   ahrs_type1 = "FCQ"
#   ahrs_type1 = "FCR2"
#   ahrs_type1 = "FLQ"
    ahrs_type1 = "ICQ"
#   ahrs_type1 = "FCR"

    sim_res1 = ahrs_utils.run_simulation(ahrs_type1, build_opt1, traj_nb)
#   import code; code.interact(local=locals())

    build_opt2 = []
    build_opt2 += ["FREQUENCY=120"]
    build_opt2 += ["GRAVITY_UPDATE_COORDINATED_TURN=1"]
    build_opt2 += ["GRAVITY_UPDATE_NORM_HEURISTIC=1"]
    build_opt2 += ["USE_AHRS_GPS_ACCELERATIONS=1"]
#   build_opt2 += ["DISABLE_MAG_UPDATE=1"]
#   build_opt2 += ["USE_GPS_HEADING=1"]
#   build_opt2 += ["MAG_UPDATE_YAW_ONLY=0"]
#   build_opt2 += ["PROPAGATE_LOW_PASS_RATES=1"]
#   build_opt2 = build_opt1

#   ahrs_type2 = "FLQ"
#   ahrs_type2 = "FCR2"
#   ahrs_type2 = "ICQ"
    ahrs_type2 = "FCR"
#   ahrs_type2 = ahrs_type1

    sim_res2 = ahrs_utils.run_simulation(ahrs_type2, build_opt2, traj_nb)

    ahrs_utils.plot_simulation_results(False, 'b', ahrs_type1, sim_res1)
    ahrs_utils.plot_simulation_results(True,  'g', ahrs_type2, sim_res2)
    ahrs_utils.show_plot()

if __name__ == "__main__":
    script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
    if script_path != os.path.abspath(os.getcwd()):
        sys.exit("Please run this script from " + script_path)
    main()
