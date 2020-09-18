#!/usr/bin/env python3

from lxml import etree
from typing import List


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

    def parse(self, fp_xml):
        fp_tree = etree.parse(fp_xml)
        fp_element = fp_tree.find("flight_plan")

        self.name = fp_element.attrib["name"]
        self.lat0 = fp_element.attrib["lat0"]
        self.lon0 = fp_element.attrib["lon0"]
        self.max_dist_from_home = fp_element.attrib["max_dist_from_home"]
        self.ground_alt = fp_element.attrib["ground_alt"]
        self.security_height = fp_element.attrib["security_height"]
        self.alt = fp_element.attrib["alt"]

        if fp_element.find("header") is not None:
            self.header = fp_element.find("header").text

        ways_elt = fp_element.find("waypoints")
        self.waypoints = FlightPlan.parse_waypoints(ways_elt)

        blocks_elt = fp_element.find("blocks")
        self.blocks = FlightPlan.parse_blocks(blocks_elt)

        excs_elt = fp_element.find("exceptions")
        self.exceptions = FlightPlan.parse_exceptions(excs_elt)

    @staticmethod
    def parse_waypoints(ways_elt: etree.Element):
        waypoints = []
        for way_e in ways_elt.findall("waypoint"):
            name = way_e.attrib["name"]
            x = way_e.attrib.get("x")
            y = way_e.attrib.get("y")
            lat = way_e.attrib.get("lat")
            lon = way_e.attrib.get("lon")
            alt = way_e.attrib.get("alt")
            height = way_e.attrib.get("height")
            waypoint = Waypoint(name, x, y, lat, lon, alt, height)
            waypoints.append(waypoint)
        return waypoints

    @staticmethod
    def parse_blocks(blocks_elt):
        blocks = []
        for b_e in blocks_elt.findall("block"):
            name = b_e.attrib["name"]
            no = b_e.attrib["no"]
            block = Block(name, no, b_e)
            blocks.append(block)
        return blocks

    @staticmethod
    def parse_exceptions(exs_elt):
        if exs_elt is None:
            return []
        excs = []
        for ex_e in exs_elt.findall("exception"):
            cond = ex_e.attrib["cond"]
            deroute = ex_e.attrib["deroute"]
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
        return list(set(filter(lambda x: x is not None, [block.xml.attrib.get("group") for block in self.blocks])))

    def get_blocks_from_group(self, groupname):
        return list(filter(lambda b: b.xml.attrib.get("group") == groupname, self.blocks))

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

