#
# Copyright (C) 2024 TUDelft
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

# This is not the main script. Run dist.py to have a distance counter.

import sys
import os


PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))

sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

class MissionVisualizer(object):

    def __init__(self):
        """
        Initialize the Mission visualizer
        """

        # Create a new ivy interface
        self.ivy_interface = IvyMessagesInterface("mission visualizer")
        self.ivy_interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        if msg.name == "MISSION_ITEM":
            self.mission_item_receiver(msg)
    
    def mission_item_receiver(self, msg):
        seq = msg['seq']
        lat = msg['lat'] # e7 deg
        lon = msg['lon'] # e7 deg
        self.delete_point(seq)
        self.draw_point(seq, lat, lon)
        return

    def draw_point(self, seq, lat, lon):
        """
        Draw a point
        """

        # Draw the shape
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = seq
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "orange"
        msg['opacity'] = 3
        msg['shape'] = 0 # Circle
        msg['status'] = 0 # Create
        msg['latarr'] = [lat] # e-7 deg
        msg['lonarr'] = [lon] # e-7 deg
        msg['radius'] = 5
        self.ivy_interface.send(msg)

    def delete_point(self, seq):
        """
        Delete a point
        """

        # Draw the shape
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = seq
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "orange"
        msg['opacity'] = 3
        msg['shape'] = 0 # Circle
        msg['status'] = 1 # Delete
        msg['latarr'] = [0] # e-7 deg
        msg['lonarr'] = [0] # e-7 deg
        msg['radius'] = 5
        self.ivy_interface.send(msg)

if __name__ == '__main__':
    visualizer = MissionVisualizer()