#!/usr/bin/env python3

from lxml import etree
from typing import List
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
        for way_e in ways_elt.findall("waypoint"):
            name = get_attrib(way_e, "name")
            x = get_attrib_default(way_e, "x", None, float)
            y = get_attrib_default(way_e, "y", None, float)
            lat = get_attrib_default(way_e, "lat", None)
            lon = get_attrib_default(way_e, "lon", None)
            alt = get_attrib_default(way_e, "alt", None, float)
            height = get_attrib_default(way_e, "height", None, float)
            waypoint = Waypoint(name, x, y, lat, lon, alt, height)
            waypoints.append(waypoint)
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

    def get_waypoint_no(self, name: str):
        for wp in self.waypoints:
            if wp.name == name:
                return self.waypoints.index(wp) + 1

    def get_block_names(self):
        return [block.name for block in self.blocks]

    def get_block_groups(self):
        return list(set(filter(lambda x: x is not None,
                               [get_attrib_default(block.xml, "group", None) for block in self.blocks])))

    def get_blocks_from_group(self, groupname):
        return list(filter(lambda block: get_attrib_default(block.xml, "group", None) == groupname, self.blocks))

    def get_block_number(self, block_name):
        for block in self.blocks:
            if block.name == block_name:
                return block.no
        raise Exception("No block named {}".format(block_name))


class Waypoint:
    def __init__(self, name, x, y, lat, lon, alt, height):
        self.name = name
        self.x = x
        self.y = y
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.height = height


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
