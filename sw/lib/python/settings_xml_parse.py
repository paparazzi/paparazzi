#!/usr/bin/env python

import os
import sys
from lxml import etree

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
    paparazzi_home = os.getenv("PAPARAZZI_HOME")
    conf_xml_path = "%s/conf/conf.xml" % paparazzi_home
    conf_tree = etree.parse(conf_xml_path)
    # extract aircraft node from conf.xml file
    ac_node = conf_tree.xpath('/conf/aircraft[@ac_id=%i]' % ac_id)
    if (len(ac_node) != 1):
      print "Aircraft ID %i not found." % ac_id

    # get settings file path from aircraft xml node
    settings_xml_path = "%s/conf/%s" % (paparazzi_home, ac_node[0].attrib['settings'])

    # save AC name for reference
    self.name = ac_node[0].attrib['name']

    tree = etree.parse(settings_xml_path)
  
    index = 0 # keep track of index/id of setting starting at 0
    for the_tab in tree.xpath("//dl_settings"):
      if the_tab.attrib.has_key('NAME'):
        setting_group = PaparazziSettingsGroup(the_tab.attrib['NAME'])
      elif the_tab.attrib.has_key('NAME'):
        setting_group = PaparazziSettingsGroup(the_tab.attrib['name'])
      else:
        continue
 
      for the_setting in the_tab.xpath('dl_setting'):
        if the_setting.attrib.has_key('shortname'):
          name = the_setting.attrib['shortname']
        elif the_setting.attrib.has_key('VAR'):
          name = the_setting.attrib['VAR']
        else:
          name = the_setting.attrib['var']
        settings = PaparazziSetting(name)
        settings.index = index
        if the_setting.attrib.has_key('MIN'):
          settings.min_value = float(the_setting.attrib['MIN'])
        else:
          settings.min_value = float(the_setting.attrib['min'])
        if the_setting.attrib.has_key('MAX'):
          settings.max_value = float(the_setting.attrib['MAX'])
        else:
          settings.max_value = float(the_setting.attrib['max'])
        if the_setting.attrib.has_key('STEP'):
          settings.step = float(the_setting.attrib['STEP'])
        else:
          settings.step = float(the_setting.attrib['step'])
	if (the_setting.attrib.has_key('values')):
	  settings.values = the_setting.attrib['values'].split('|')
	  count = int((settings.max_value - settings.min_value + settings.step) / settings.step)
	  if (len(settings.values) != count):
	    print "Warning: wrong number of values (%i) for %s (expected %i)" % (len(settings.values), name, count)
  
        setting_group.member_list.append(settings)
        self.lookup.append(settings)
        self.name_lookup[name] = settings
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
  min_value = 0
  max_value = 1
  step = 1
  index = 0
  value = None
  values = None
  def __init__(self, shortname):
    self.shortname = shortname


def test():
  ac_id = 164
  ac_settings = PaparazziACSettings(ac_id)
  for setting_group in ac_settings.groups:
    print setting_group.name
    for setting in setting_group.member_list:
      print " " + setting.shortname

if __name__ == '__main__':
  test()


