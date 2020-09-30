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
from typing import List, Union
import sys
from xml_utils import get_attrib, get_attrib_default


class FlightPlan:

    def __init__(self):
        self.header = ""        # type: str
        self.waypoints = []     # type: List[Waypoint]
        self.exceptions = []    # type: List[Exc]
        self.blocks = []        # type: List[Block]
        self.name = None
        self.lat0 = None
        self.lon0 = None
        self.max_dist_from_home = None
        self.ground_alt = None
        self.security_height = None
        self.alt = None

    @staticmethod
    def parse(fp_xml):
        fp = FlightPlan()
        fp_tree = etree.parse(fp_xml)
        fp_element = fp_tree.find("flight_plan")

        fp.name = get_attrib(fp_element, "name")
        fp.lat0 = get_attrib(fp_element, "lat0")
        fp.lon0 = get_attrib(fp_element, "lon0")
        fp.max_dist_from_home = get_attrib(fp_element, "max_dist_from_home")
        fp.ground_alt = get_attrib(fp_element, "ground_alt")
        fp.security_height = get_attrib(fp_element, "security_height")
        fp.alt = get_attrib(fp_element, "alt")

        if fp_element.find("header") is not None:
            fp.header = fp_element.find("header").text

        ways_elt = fp_element.find("waypoints")
        fp.waypoints = FlightPlan.parse_waypoints(ways_elt)

        blocks_elt = fp_element.find("blocks")
        fp.blocks = FlightPlan.parse_blocks(blocks_elt)

        excs_elt = fp_element.find("exceptions")
        fp.exceptions = FlightPlan.parse_exceptions(excs_elt)
        return fp

    @staticmethod
    def parse_waypoints(ways_elt: etree.Element):
        waypoints = []
        w_no = 1    # first waypoint number is 1.
        for way_e in ways_elt.findall("waypoint"):
            name = get_attrib(way_e, "name")
            x = get_attrib_default(way_e, "x", None, float)
            y = get_attrib_default(way_e, "y", None, float)
            lat = get_attrib_default(way_e, "lat", None)
            lon = get_attrib_default(way_e, "lon", None)
            alt = get_attrib_default(way_e, "alt", None, float)
            height = get_attrib_default(way_e, "height", None, float)
            waypoint = Waypoint(name, x, y, lat, lon, alt, height, w_no)
            waypoints.append(waypoint)
            w_no += 1
        return waypoints

    @staticmethod
    def parse_blocks(blocks_elt):
        blocks = []
        for b_e in blocks_elt.findall("block"):
            name = get_attrib(b_e, "name")
            no = get_attrib(b_e, "no")
            block = Block(name, no, b_e)
            blocks.append(block)
        return blocks

    @staticmethod
    def parse_exceptions(exs_elt):
        if exs_elt is None:
            return []
        excs = []
        for ex_e in exs_elt.findall("exception"):
            cond = get_attrib(ex_e, "cond")
            deroute = get_attrib(ex_e, "deroute")
            exc = Exc(cond, deroute)
            excs.append(exc)
        return excs

    def get_waypoint(self, key: Union[str, int]):
        """
        :param key: Waypoint name or number
        :type key: str or int
        """
        if type(key) == str:
            for wp in self.waypoints:
                if wp.name == key:
                    return wp
        elif type(key) == int:
            for wp in self.waypoints:
                if wp.no == key:
                    return wp

    def get_block(self, key: Union[str, int]):
        """
        :param key: Block name or number
        :type key: str or int
        """
        if type(key) == str:
            for block in self.blocks:
                if block.name == key:
                    return block
        elif type(key) == int:
            for block in self.blocks:
                if block.no == key:
                    return block

    def get_block_groups(self):
        return list(set(filter(lambda x: x is not None,
                               [get_attrib_default(block.xml, "group", None) for block in self.blocks])))

    def get_blocks_from_group(self, groupname):
        return list(filter(lambda block: get_attrib_default(block.xml, "group", None) == groupname, self.blocks))


class Waypoint:
    def __init__(self, name, x, y, lat, lon, alt, height, no):
        self.name = name
        self.x = x
        self.y = y
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.height = height
        self.no = no


class Block:
    def __init__(self, name, no, xml):
        self.name = name
        self.no = no
        self.xml = xml


class Exc:
    def __init__(self, cond, deroute):
        self.cond = cond
        self.deroute = deroute


if __name__ == "__main__":
    flight_plan = FlightPlan.parse(sys.argv[1])
    for b in flight_plan.blocks:
        print(b.name)
