import wx

import sys
import time
import threading

import logging
from ivy.std_api import *
_SHOW_IVY_MSGS_ = False


from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from settings_xml_parse import PaparazziACSettings

PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)


from ivy_msg_interface import IvyMessagesInterface
from pprz_msg.message import PprzMessage

WIDTH = 480
HEIGHT = 320


class Message(PprzMessage):
    def __init__(self, class_name, name):
        super(Message, self).__init__(class_name, name)
        self.field_controls = []
        self.index = None
        self.last_seen = time.clock()


class Aircraft(object):
    def __init__(self, ac_id):
        self.ac_id = ac_id
        self.messages = {}
        self.messages_book = None

class LoggerCmd():
    start = 0
    stop = 1
    download = 2


class SDLogDownloadFrame(wx.Frame):
    def __init__(self, ac_id, msg_class="telemetry"):
        self.settings = PaparazziACSettings(ac_id[0])
        self.ac_id = ac_id[0]

        # Init ivy
        self.msg_class = msg_class
        self.InitIvy()

        # GUI
        wx.Frame.__init__(self, id=-1, parent=None, name=u'MessagesFrame', size=wx.Size(WIDTH, HEIGHT), style=wx.DEFAULT_FRAME_STYLE, title=u'SDCard Log Downloader')
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # Menubar
        menuBar = wx.MenuBar()
        fileMenu = wx.Menu()
        fitem = fileMenu.Append(wx.ID_EXIT, 'Quit', 'Close the SD logger download application')
        self.Bind(wx.EVT_MENU, self.OnClose, fitem)
        advancedMenu = wx.Menu()
        fitem = advancedMenu.Append(1, 'Log information', 'Get status of recently stored log.')
        self.Bind(wx.EVT_MENU, self.OnStatusRequest, fitem)
        menuBar.Append(fileMenu, '&File')
        menuBar.Append(advancedMenu, '&Advanced')
        self.SetMenuBar(menuBar)

        # Other widgets
        self.inDataLabel = wx.StaticText(self, id=12, label="", pos=wx.Point(20, 200))
        self.statusDataLabel = wx.StaticText(self, id=13, label="", pos=wx.Point(20, 220))
        self.progressBar = wx.Gauge(self, range=100, pos=wx.Point(100, 50), size=wx.Size(280, 30))
        self.startButton = wx.Button(self, id=1, label="Start", pos=wx.Point(100, 100), size=wx.Size(100, 25))
        self.stopButton = wx.Button(self, id=2, label="Stop", pos=wx.Point(280, 100), size=wx.Size(100, 25))
        self.stopButton.Disable()
        self.downloadButton = wx.Button(self, id=3, label="Download!", pos=wx.Point(190, 150), size=wx.Size(100, 25))
        self.Bind(wx.EVT_BUTTON, self.onButton)

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        sizer.Layout()

        # Some variables
        self.download_counter = 0
        self.download_available = 0
        self.last_command = 0
        self.unique_id = 0
        self.download_timer = threading.Timer
        self.request_timer = threading.Timer
        self.timeout_time = 0.1

    # Called on button push
    def onButton(self, event):
        button = event.GetEventObject()
        index = int(button.GetId())
        self.last_command = index
        setting_index = self.settings.name_lookup["sdlogger.cmd"].index
        unique_id_idx = self.settings.name_lookup["sdlogger.unique_id"].index
        if index == 1:
            # First set unique id
            IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, unique_id_idx, int(time.time()) - 1428500000))
            self.last_command = 0
            # Continuing trough callback from setting unique id
        elif index == 2:
            IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, setting_index, 2))
            self.stopButton.Disable()
            self.request_timer = threading.Timer(0.5, self.stopButton.Enable)
            self.request_timer.start()
        elif index == 3:
            IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, setting_index, 3))
            self.downloadButton.Disable()

    def OnStatusRequest(self, event):
        setting_index = self.settings.name_lookup["sdlogger.cmd"].index
        IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, setting_index, 3))

    def OnLogPacketReceive(self, agent, *larg):
        wx.CallAfter(self.process_log_packet, larg[0])

        # Called to update GUI with new values
    def process_log_packet(self, message):
        self.inDataLabel.SetLabel(message)
        # Extract field values
        message_values = list(filter(None, message.split(' ')))
        message_values = message_values[0:1] + message_values[2:]
        if self.last_command == 3:
            self.download_counter = 0
            self.download_available = int(message_values[1])
            self.unique_id = message_values[3]
            self.statusDataLabel.SetLabel(message)
            # Set the unique ID before reading, otherwise zeros will be returned
            setting_index = self.settings.name_lookup["sdlogger.unique_id"].index
            IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, setting_index, self.unique_id))
            #self.RequestNextPacket() do this in callback instead
        elif self.last_command == 57:
            self.download_timer.cancel()
            self.inDataLabel.SetLabel(message)
            fh = open("logfile.txt", "a")
            fh.write(message + "\n")
            fh.close()
            percentage = (self.download_counter * 100)/self.download_available
            self.progressBar.SetValue(percentage)
            self.download_counter += 1
            self.timeout_time = 0.1
            self.RequestNextPacket()

    def OnSettingConfirmation(self, agent, *larg):
        wx.CallAfter(self.process_setting_confirmation, larg[0])

    def process_setting_confirmation(self, message):
        # Extract field values
        message_values = list(filter(None, message.split(' ')))
        message_values = message_values[0:1] + message_values[2:]
        cmd_idx = self.settings.name_lookup["sdlogger.cmd"].index
        unique_id_idx = self.settings.name_lookup["sdlogger.unique_id"].index
        given_idx = int(message_values[1])
        if given_idx == cmd_idx:
            self.inDataLabel.SetLabel(message)
            given_cmd = int(float(message_values[2]))
            # start command
            if given_cmd == 1:
                self.request_timer.cancel()
                self.startButton.Disable()
                self.downloadButton.Disable()
                self.stopButton.Enable()
            if given_cmd == 2:
                self.request_timer.cancel()
                self.stopButton.Disable()
                self.startButton.Enable()
                self.downloadButton.Enable()
        elif given_idx == unique_id_idx:
            # If set for start logging
            if self.last_command == 0:
                # continue with logging command
                IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, cmd_idx, 1))
                self.startButton.Disable()
                self.request_timer = threading.Timer(0.5, self.startButton.Enable)
                self.request_timer.start()
            # If set for download request
            if self.last_command == 3:
                self.download_counter += 1
                self.RequestNextPacket()

    def RequestNextPacket(self):
        if (self.download_counter <= self.download_available):
            self.timeout_time = self.timeout_time * 2
            self.last_command = 57
            setting_index = self.settings.name_lookup["sdlogger.request_id"].index
            IvySendMsg("dl DL_SETTING %s %s %s" % (self.ac_id, setting_index, self.download_counter))
            self.download_timer = threading.Timer(0.3, self.RequestNextPacket)
            self.download_timer.start()
        else:
            self.inDataLabel.SetLabel("Download complete!")
            self.downloadButton.Enable()

    def InitIvy(self):
        # initialising the bus
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyInit("sdlog_download_app", # application name for Ivy
                "",                 # ready message
                0,                  # main loop is local (ie. using IvyMainloop)
                lambda x,y: y,      # handler called on connection/deconnection
                lambda x,y: y       # handler called when a diemessage is received
                )

        # starting the bus
        IvyStart("")
        IvyBindMsg(self.OnLogPacketReceive, "(^.* LOG_DATAPACKET .*)")
        IvyBindMsg(self.OnSettingConfirmation, "(^.* DL_VALUE .*)")
        #IvyBindMsg(self.OnSettingMsg, "dl DL_SETTING (.*)")

    def OnClose(self, event):
        IvyStop()
        self.Destroy()
