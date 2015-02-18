#!/usr/bin/env python
#Boa:App:BoaApp

import wx
import os
import settingsframe
import getopt
import sys

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
import settings_xml_parse

def Usage(scmd):
    lpathitem = scmd.split('/')
    fmt = '''Usage: %s [-h | --help] [-a AC_ID | --ac_id=AC_ID]
where
\t-h | --help print this message
\t-a AC_ID | --ac_id=AC_ID where AC_ID is an aircraft ID to use for settings (multiple IDs may passed)
'''
    print(fmt % lpathitem[-1])

def GetOptions():
    options = {'ac_id':[]}
    try:
        optlist, left_args = getopt.getopt(sys.argv[1:],'h:a:', ['help', 'ac_id='])
    except getopt.GetoptError:
        # print help information and exit:
        Usage(sys.argv[0])
        sys.exit(2)
    for o, a in optlist:
        if o in ("-h", "--help"):
            Usage(sys.argv[0])
            sys.exit()
        elif o in ("-a", "--ac_id"):
            options['ac_id'].append(int(a))

    return options

class SettingsApp(wx.App):
    def OnInit(self):
        options = GetOptions()
        if not options['ac_id']:
            Usage(sys.argv[0])
            sys.exit("Error: Please specify at least one aircraft ID.")
        self.main = settingsframe.create(None, options['ac_id'])
        self.main.Show()
        self.SetTopWindow(self.main)

        return True

def main():
    application = SettingsApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
