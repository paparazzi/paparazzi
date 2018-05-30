#!/usr/bin/env python3

# Paparazzi center utilities
#
# Copyright (C) 2016 ENAC, Florian BITARD (intern student)
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
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

###############################################################################
# [Imports]

import lib.environment as env
import hmi

import PyQt5.QtWidgets as Widgets
import sys
import os


###############################################################################
# [Main function]
def main():
    """Main program : creates the main window and starts the main loop."""

    # Set this file's location as working directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Set the environment variables (useful for some processes) :
    os.putenv(env.HOME_NAME, env.PAPARAZZI_HOME)
    os.putenv(env.SRC_NAME, env.PAPARAZZI_SRC)

    app = Widgets.QApplication(sys.argv)

    # TODO : SET APPLICATION ICON IN THE LAUNCHER (UBUNTU : UNITY)

    main_window = hmi.Hmi()
    main_window.init_all_hmi()
    main_window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
