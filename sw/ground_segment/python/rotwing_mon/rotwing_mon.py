#!/usr/bin/env python3
#
# Copyright (C) 2023 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

import wx
import rotwing_viewer

class RotWingApp(wx.App):

    def __init__(self, args):
        self.args = args
        wx.App.__init__(self)

    def OnInit(self):
        self.main = rotwing_viewer.RotWingFrame(self.args)
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main(args):
    application = RotWingApp(args)
    application.MainLoop()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Rotwing Status Monitor")
    parser.add_argument('-s', '--speech', dest='speech', default=False, type=bool, help="Rotwing status viewer speech")
    args = parser.parse_args()

    main(args)