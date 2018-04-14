#!/usr/bin/env python
#
# Copyright (C) 2016 TUDelft
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
import sys
import argparse
import distance_counter

parser = argparse.ArgumentParser(description='Capture PAYLOAD messages over the IVY bus and forward to a remote application.', epilog='payload.py is part of the paparazzi-uav project.')
settings = parser.parse_args()

print(settings)

class DistanceFrame(wx.App):
    def OnInit(self):
        self.main = distance_counter.DistanceCounterFrame(settings)
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = DistanceFrame(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
