#!/usr/bin/env python3
#
# Copyright (C) 2018 TUDelft
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
import rot_wing_viewer

class RotWingApp(wx.App):
    def OnInit(self):
        self.main = rot_wing_viewer.RotWingFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = RotWingApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()