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

import PyQt5.QtGui as Gui


###############################################################################
# [Constants]

NORMAL_CURSOR_SHAPE = "normal"
WAIT_CURSOR_SHAPE = "wait"


###############################################################################
# [Functions]

def generate_qcolor(color):
    qcolor = Gui.QColor()
    qcolor.setNamedColor(color)
    return qcolor


def generate_widget_palette(widget, qcolor):
    qpalette = Gui.QPalette(widget.palette())
    qpalette.setColor(Gui.QPalette.Background, qcolor)
    return qpalette


def generate_qcursor(shape):
    qcursor = Gui.QCursor()
    if shape == WAIT_CURSOR_SHAPE:
        qcursor.setShape(16)
    elif shape == NORMAL_CURSOR_SHAPE:
        qcursor.setShape(0)
    else:
        qcursor.setShape(0)
    return qcursor


def generate_qpixmap(picture_path=""):
    return Gui.QPixmap(picture_path)


def generate_qicon(icon_path=None):
    if icon_path is not None:
        return Gui.QIcon(icon_path)
    else:
        return Gui.QIcon()
