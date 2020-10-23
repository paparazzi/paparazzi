#!/usr/bin/env python3
#
# Copyright (C) 2020 Fabien Bonneval <fabien.bonneval@enac.fr>
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
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from lxml import etree


def get_attrib(node: etree.Element, tag: str, typ=None):
    """XML Element attribute getter which is insensitive to case, handle default value and type conversion"""
    try:
        value = node.attrib[tag.lower()]
    except KeyError:
        value = node.attrib[tag.upper()]

    if typ is None:
        return value
    else:
        return typ(value)


def get_attrib_default(node: etree.Element, tag: str, default, typ=None):
    try:
        value = get_attrib(node, tag, typ=typ)
    except KeyError:
        value = default

    if typ is not None and default is not None:
        return typ(value)
    else:
        return value
