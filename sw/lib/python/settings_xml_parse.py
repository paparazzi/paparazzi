#!/usr/bin/env python

from __future__ import absolute_import, print_function

import os
import sys
from lxml import etree

def printerr(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../..')))
# Class for all settings
class PaparazziACSettings:
    "Paparazzi Settings Class"
    ac_id = 0
    groups = []
    lookup = []
    name_lookup = {}
    # Takes a string file path for settings XML file and
    # returns a settings AC object

    def __init__(self, ac_id):
        self.ac_id = ac_id
        paparazzi_home = PPRZ_HOME
        paparazzi_conf = os.path.join(paparazzi_home, 'conf')
        conf_xml_path = os.path.join(paparazzi_conf, 'conf.xml')
        conf_tree = etree.parse(conf_xml_path)
        # extract aircraft node from conf.xml file
        ac_node = conf_tree.xpath('/conf/aircraft[@ac_id=%i]' % ac_id)
        if (len(ac_node) != 1):
            printerr("Aircraft ID %i not found." % ac_id)
            sys.exit(1)

        # save AC name for reference
        self.name = ac_node[0].attrib['name']

        # get settings.xml file path from var/aircrafts/<ac_name> directory
        settings_xml_path = os.path.join(paparazzi_home, 'var/aircrafts/' + self.name + '/settings.xml')

        if not os.path.isfile(settings_xml_path):
            printerr("Could not find %s, did you build it?" % settings_xml_path)
            sys.exit(1)

        index = 0 # keep track of index/id of setting starting at 0
        tree = etree.parse(settings_xml_path)

        for the_tab in tree.xpath("//dl_settings"):
            try:
                if 'NAME' in the_tab.attrib:
                    setting_group_name = the_tab.attrib['NAME']
                else:
                    setting_group_name = the_tab.attrib['name']
            except:
                #printerr("Could not read name of settings group")
                continue

            #print("parsing setting group:", setting_group_name)
            setting_group = PaparazziSettingsGroup(setting_group_name)

            for the_setting in the_tab.xpath('dl_setting'):
                try:
                    if 'shortname' in the_setting.attrib:
                        shortname = the_setting.attrib['shortname']
                    if 'VAR' in the_setting.attrib:
                        varname = the_setting.attrib['VAR']
                    else:
                        varname = the_setting.attrib['var']
                    if 'shortname' in the_setting.attrib:
                        shortname = the_setting.attrib['shortname']
                    else:
                        shortname = varname
                except:
                    printerr("Could not get name for setting in group", setting_group)
                    continue

                settings = PaparazziSetting(shortname, varname)
                settings.index = index

                try:
                    if 'MIN' in the_setting.attrib:
                        settings.min_value = float(the_setting.attrib['MIN'])
                    else:
                        settings.min_value = float(the_setting.attrib['min'])

                    if 'MAX' in the_setting.attrib:
                        settings.max_value = float(the_setting.attrib['MAX'])
                    else:
                        settings.max_value = float(the_setting.attrib['max'])

                    if 'STEP' in the_setting.attrib:
                        settings.step = float(the_setting.attrib['STEP'])
                    else:
                        settings.step = float(the_setting.attrib['step'])
                except:
                    printerr("Could not get min/max/step for setting", name)
                    continue

                if 'values' in the_setting.attrib:
                    settings.values = the_setting.attrib['values'].split('|')
                    count = int((settings.max_value - settings.min_value + settings.step) / settings.step)
                    if len(settings.values) != count:
                        printerr("Warning: possibly wrong number of values (%i) for %s (expected %i)" % (len(settings.values), shortname, count))

                setting_group.member_list.append(settings)
                self.lookup.append(settings)
                self.name_lookup[shortname] = settings
                index = index + 1

            self.groups.append(setting_group)

    def GetACName(self):
        return self.name

# Class for named group of settings
class PaparazziSettingsGroup:
    "Paparazzi Setting Group Class"
    name = 0
    member_list = []

    def __init__(self, name):
        self.name = name
        self.member_list = []

# Class for a single paparazzi setting
class PaparazziSetting:
    "Paparazzi Setting Class"
    shortname = ""
    var = ""
    min_value = 0
    max_value = 1
    step = 1
    index = 0
    value = None
    values = None

    def __init__(self, shortname, var):
        self.shortname = shortname
        self.var = var

    def __str__(self):
        return "var: %s, shortname: %s, index: %i" % (self.var, self.shortname, self.index)

    # return the index in 'values' table matching a given name or None if no values defined
    # may raise ValueError if name is not in values list
    def ValueFromName(self, name):
        if self.values is None:
            return None
        return self.values.index(name) + self.min_value



def test():
    ac_id = 164
    ac_settings = PaparazziACSettings(ac_id)
    for setting_group in ac_settings.groups:
        print(setting_group.name)
    for setting in setting_group.member_list:
        print(" " + setting.shortname)

if __name__ == '__main__':
    test()
