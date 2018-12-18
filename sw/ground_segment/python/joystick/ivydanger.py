#! /usr/bin/env python

from __future__ import print_function
from arduino_dangerboard import arduino_dangerboard
from ivy.std_api import *
import logging
import os
import sys
import getopt
import signal

PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from settings_tool import IvySettingsInterface

DEFAULT_AC_IDS = [ ]

# Map dangerboard sliders to these settings from aircraft settings
# file, in that order (dimension of this list needs to match number
# of sliders!)
DEFAULT_SLIDERS = [ ]

DEFAULT_PORT = '/dev/ttyUSB0'

class IvyStick(arduino_dangerboard):
  def __init__(self, ac_ids, settings_names, port):
    arduino_dangerboard.__init__(self, port)
    if (len(settings_names) > self.SLIDER_COUNT):
      raise Exception("Number of settings greater than number of sliders")
    if (len(ac_ids) < 1):
      raise Exception("Need at least one ac_id")
    self.last_values = [0] * len(settings_names)
    self.ac_settings = IvySettingsInterface(ac_ids)
    self.settings = []
    for name in settings_names:
      self.settings.append(self.ac_settings.name_lookup[name])

  def ScalePot(self, pot_value, min_value, max_value, step_size):
    scale_factor = (max_value - min_value) / (self.POT_MAX - self.POT_MIN)
    offset = (self.POT_MIN - min_value) / scale_factor
    return int((pot_value - offset) * scale_factor / step_size) * step_size

  def HandleEvent(self):
    for setting_index in range(0, len(self.settings)):
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
\t-p PORT | --port=PORT where PORT is the name of the serial port for the slider box
'''
  print(fmt   %  lpathitem[-1])

def GetOptions():
  # Map dangerboard sliders to these settings from aircraft settings
  # file, in that order (dimension of this list needs to match number
  # of sliders!)
  options = {'ac_id':DEFAULT_AC_IDS, 'sliders':DEFAULT_SLIDERS, 'port':DEFAULT_PORT}
  try:
    optlist, left_args = getopt.getopt(sys.argv[1:],'h:a:s:p:', ['help', 'ac_id=', 'sliders=', 'port='])
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
    elif o in ("-p", "--port"):
      options['port'] = a

  return options

def signal_handler(signal, frame):
  sys.exit(0)

def main():
  signal.signal(signal.SIGINT, signal_handler)

  options = GetOptions()
  ivyStick = IvyStick(options['ac_id'], options['sliders'], options['port'])
  ivyStick.poll()

if __name__ == '__main__':
  main()
