import wx

import sys
import time
import threading

from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)

from ivy_msg_interface import IvyMessagesInterface
from pprz_msg.message import PprzMessage

WIDTH = 450
LABEL_WIDTH = 166
DATA_WIDTH = 100
HEIGHT = 800
BORDER = 1


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


class MessagesFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        """Handle incoming messages

        Callback function for IvyMessagesInterface

        :param ac_id: aircraft id
        :type ac_id: int
        :param msg: message
        :type msg: PprzMessage
        """
        # only show messages of the requested class
        if msg.msg_class != self.msg_class:
            return
        if ac_id in self.aircrafts and msg.name in self.aircrafts[ac_id].messages:
            if time.time() - self.aircrafts[ac_id].messages[msg.name].last_seen < 0.2:
                return

        wx.CallAfter(self.gui_update, ac_id, msg)

    def find_page(self, book, name):
        if book.GetPageCount() < 1:
            return 0
        start = 0
        end = book.GetPageCount()

        while start < end:
            if book.GetPageText(start) >= name:
                return start
            start += 1
        return start

    def update_leds(self):
        wx.CallAfter(self.update_leds_real)

    def update_leds_real(self):
        for ac_id in self.aircrafts:
            aircraft = self.aircrafts[ac_id]
            for msg_str in aircraft.messages:
                message = aircraft.messages[msg_str]
                if message.last_seen + 0.2 < time.time():
                    aircraft.messages_book.SetPageImage(message.index, 0)

        self.timer = threading.Timer(0.1, self.update_leds)
        self.timer.start()

    def setup_image_list(self, notebook):
        imageList = wx.ImageList(24, 24)

        image = wx.Image(PPRZ_HOME + "/data/pictures/gray_led24.png")
        bitmap = wx.BitmapFromImage(image)
        imageList.Add(bitmap)

        image = wx.Image(PPRZ_HOME + "/data/pictures/green_led24.png")
        bitmap = wx.BitmapFromImage(image)
        imageList.Add(bitmap)

        notebook.AssignImageList(imageList)

    def add_new_aircraft(self, ac_id):
        self.aircrafts[ac_id] = Aircraft(ac_id)
        ac_panel = wx.Panel(self.notebook, -1)
        self.notebook.AddPage(ac_panel, str(ac_id))
        messages_book = wx.Notebook(ac_panel, style=wx.NB_LEFT)
        self.setup_image_list(messages_book)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(messages_book, 1, wx.EXPAND)
        ac_panel.SetSizer(sizer)
        sizer.Layout()
        self.aircrafts[ac_id].messages_book = messages_book

    def add_new_message(self, aircraft, msg_class, name):
        messages_book = aircraft.messages_book
        aircraft.messages[name] = Message(msg_class, name)
        field_panel = wx.Panel(messages_book)
        grid_sizer = wx.FlexGridSizer(len(aircraft.messages[name].fieldnames), 2)

        index = self.find_page(messages_book, name)
        messages_book.InsertPage(index, field_panel, name, imageId=1)
        aircraft.messages[name].index = index

        # update indexes of pages which are to be moved
        for message_name in aircraft.messages:
            aircraft.messages[message_name].index = self.find_page(messages_book, message_name)

        for field_name in aircraft.messages[name].fieldnames:
            name_text = wx.StaticText(field_panel, -1, field_name)
            size = name_text.GetSize()
            size.x = LABEL_WIDTH
            name_text.SetMinSize(size)
            grid_sizer.Add(name_text, 1, wx.ALL, BORDER)
            value_control = wx.StaticText(field_panel, -1, "42", style=wx.EXPAND)
            size = value_control.GetSize()
            size.x = LABEL_WIDTH
            value_control.SetMinSize(size)
            grid_sizer.Add(value_control, 1, wx.ALL | wx.EXPAND, BORDER)
            if wx.MAJOR_VERSION > 2:
                if grid_sizer.IsColGrowable(1):
                    grid_sizer.AddGrowableCol(1)
            else:
                grid_sizer.AddGrowableCol(1)
            aircraft.messages[name].field_controls.append(value_control)

        field_panel.SetAutoLayout(True)
        field_panel.SetSizer(grid_sizer)
        field_panel.Layout()

    def gui_update(self, ac_id, msg):
        if ac_id not in self.aircrafts:
            self.add_new_aircraft(ac_id)
        aircraft = self.aircrafts[ac_id]
        if msg.name not in aircraft.messages:
            self.add_new_message(aircraft, msg.msg_class, msg.name)

        aircraft.messages_book.SetPageImage(aircraft.messages[msg.name].index, 1)
        self.aircrafts[ac_id].messages[msg.name].last_seen = time.time()

        for index in range(0, len(msg.fieldvalues)):
            aircraft.messages[msg.name].field_controls[index].SetLabel(msg.get_field(index))

    def __init__(self, msg_class="telemetry"):
        wx.Frame.__init__(self, id=-1, parent=None, name=u'MessagesFrame', size=wx.Size(WIDTH, HEIGHT), style=wx.DEFAULT_FRAME_STYLE, title=u'Messages')
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.notebook = wx.Notebook(self)
        self.aircrafts = {}

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(self.notebook, 1, wx.EXPAND)
        self.SetSizer(sizer)
        sizer.Layout()
        self.timer = threading.Timer(0.1, self.update_leds)
        self.timer.start()
        self.msg_class = msg_class
        self.interface = IvyMessagesInterface(self.message_recv)

    def OnClose(self, event):
        self.timer.cancel()
        self.interface.shutdown()
        self.Destroy()
