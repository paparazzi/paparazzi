#!/usr/bin/python3

import sys
from os import path, getenv
import time
from enum import Enum

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.dirname(path.abspath(__file__))))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect


class Shape(Enum):
    CIRCLE  = 0
    POLYGON = 1
    LINE    = 2
    TEXT    = 3


class Opacity(Enum):
    TRANSPARENT = 0
    LIGHT_FILL  = 1
    MEDIUM_FILL = 2
    OPAQUE      = 3


class Status(Enum):
    CREATE = 0
    DELETE = 1


POLYGON_COORDS = [
    (43.462, 1.272),
    (43.4622, 1.2684),
    (43.4628, 1.2686),
    (43.464, 1.2710),
    (43.4657, 1.2714),
    (43.4656, 1.2738)
]


def polygon(shape_id):
    msg_shape = PprzMessage("ground", "SHAPE")
    msg_shape['id'] = shape_id
    msg_shape['linecolor'] = 'red'
    msg_shape['fillcolor'] = 'red'
    msg_shape['opacity'] = Opacity.LIGHT_FILL.value
    msg_shape['shape'] = Shape.POLYGON.value
    msg_shape['status'] = Status.CREATE.value
    lonarr = []
    latarr = []
    for coord in POLYGON_COORDS:
        latarr.append(int(coord[0] * 1e7))
        lonarr.append(int(coord[1] * 1e7))
    msg_shape['latarr'] = latarr
    msg_shape['lonarr'] = lonarr
    msg_shape['text'] = "Crop"
    return msg_shape

def circle(shape_id):
    msg_shape = PprzMessage("ground", "SHAPE")
    msg_shape['id'] = shape_id
    msg_shape['linecolor'] = 'green'
    msg_shape['fillcolor'] = 'yellow'
    msg_shape['opacity'] = Opacity.MEDIUM_FILL.value
    msg_shape['shape'] = Shape.CIRCLE.value
    msg_shape['status'] = Status.CREATE.value
    msg_shape['latarr'] = [int(43.4640*1e7)]
    msg_shape['lonarr'] = [int(1.275*1e7)]
    msg_shape['radius'] = int(100)
    msg_shape['text'] = "hello"
    return msg_shape

def line(shape_id):
    msg_shape = PprzMessage("ground", "SHAPE")
    msg_shape['id'] = shape_id
    msg_shape['linecolor'] = 'purple'
    msg_shape['shape'] = Shape.LINE.value
    msg_shape['status'] = Status.CREATE.value
    msg_shape['latarr'] = [int(43.46296*1e7), int(43.46216*1e7)]
    msg_shape['lonarr'] = [int(1.272*1e7), int(1.2744*1e7)]
    msg_shape['text'] = "runway"
    return msg_shape

if __name__ == '__main__':
    ivy = IvyMessagesInterface("Shape drawer")
    # wait for ivy to be correctly started
    time.sleep(0.1)
    
    msg = polygon(1)
    ivy.send(msg)

    msg = circle(2)
    ivy.send(msg)

    msg = line(3)
    ivy.send(msg)

    ivy.shutdown()

