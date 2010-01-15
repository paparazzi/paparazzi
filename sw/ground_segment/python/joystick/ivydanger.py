#! /usr/bin/env python

from arduino_dangerboard import arduino_dangerboard
from ivy.std_api import *
import logging
import os
import sys
import getopt

PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from settings_tool import IvySettingsInterface

DEFAULT_AC_IDS = [ 11 ]

# Map dangerboard sliders to these settings from aircraft settings
# file, in that order (dimension of this list needs to match number
# of sliders!)
DEFAULT_SLIDERS = ["throttle_sp", "cyclic_sp", "collective_sp"]

POT_MIN = 0.0
POT_MAX = 1023.0

class IvyStick(arduino_dangerboard):
  def __init__(self, ac_ids, settings_names):
    arduino_dangerboard.__init__(self)
    self.last_values = [0] * self.SLIDER_COUNT
    self.ac_settings = IvySettingsInterface(ac_ids)
    self.settings = []
    for name in settings_names:
      self.settings.append(self.ac_settings.name_lookup[name])

  def ScalePot(self, pot_value, min_value, max_value, step_size):
    scale_factor = (max_value - min_value) / (POT_MAX - POT_MIN)
    offset = (POT_MIN - min_value) / scale_factor
    return int((pot_value - offset) * scale_factor / step_size) * step_size

  def HandleEvent(self):
    for setting_index in range(0, self.SLIDER_COUNT):
      value = self.ScalePot(self.sliders[setting_index],
        self.settings[setting_index].min_value,
        self.settings[setting_index].max_value,
        self.settings[setting_index].step)

      # keep track of last value and only send a value
      # if the value has changed since last update
      if (value != self.last_values[setting_index]):
        self.settings[setting_index].value = value
        self.ac_settings.SendSetting(self.settings[setting_index].index)
        self.last_values[setting_index] = value

def Usage(scmd):
  lpathitem = scmd.split('/')
  fmt = '''Usage: %s [-h | --help] [-a AC_ID | --ac_id=AC_ID]
where
\t-h | --help print this message
\t-a AC_ID | --ac_id=AC_ID where AC_ID is an aircraft ID to use for settings (multiple IDs may be passed)
\t-s S1:S2:S3 | --sliders=S1:S2:S3 where S1, S2, S3 are the names of the slider settings to send
'''
  print fmt   %  lpathitem[-1]

def GetOptions():
  # Map dangerboard sliders to these settings from aircraft settings
  # file, in that order (dimension of this list needs to match number
  # of sliders!)
  options = {'ac_id':DEFAULT_AC_IDS, 'sliders':DEFAULT_SLIDERS}
  try:
    optlist, left_args = getopt.getopt(sys.argv[1:],'h:a:s:', ['help', 'ac_id=', 'sliders='])
  except getopt.GetoptError:
    # print help information and exit:
    Usage(sys.argv[0])
    sys.exit(2)
  for o, a in optlist:
    if o in ("-h", "--help"):
      Usage(sys.argv[0])
      sys.exit()
    elif o in ("-a", "--ac_id"):
      options['ac_id'] = [ int(a) ]
    elif o in ("-s", "--sliders"):
      options['sliders'] = a.split(':')

  return options

def main():
  options = GetOptions()
  ivyStick = IvyStick(options['ac_id'], options['sliders'])
  ivyStick.poll()

if __name__ == '__main__':
  main()
