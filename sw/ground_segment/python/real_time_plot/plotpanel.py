from __future__ import absolute_import, print_function, division

import wx
from ivy.std_api import *
from ivy.ivy import IvyIllegalStateError
import logging
from textdroptarget import *
import math
import random
import sys
from os import getenv, path
import messagepicker

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

import pprz_env
from pprzlink import messages_xml_map
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class PlotData:
    def __init__(self, ivy_msg_id, title, width, color=None, scale=1.0):
        self.id = ivy_msg_id
        self.title = title
        self.SetPlotSize(width)
        self.x_min = 1e32
        self.x_max = 1e-32

        self.avg = 0.0
        self.std_dev = 0.0
        self.real_time = False

        self.scale = scale
        self.offset = 0.0

        if color is not None:
            self.color = color
        else:
            r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
            self.color = wx.Colour(r, g, b)

    def SetRealTime(self, value):
        self.real_time = value

    def SetOffset(self, value):
        self.offset = value

    def SetScale(self, value):
        self.scale = value

    def SetPlotSize(self, size):
        self.size = size
        self.index = size - 1  # holds the index of the next point to add and the first point to draw
        self.data = []  # holds the list of points to plot
        for i in range(size):
            self.data.append(None)

        self.avg = 0.0
        self.std_dev = 0.0

    def AddPoint(self, point, x_axis):
        self.data[self.index] = point

        if self.real_time or (x_axis is not None):
            self.index = (self.index + 1) % self.size  # increment index to next point
            self.data[self.index] = None

    def DrawTitle(self, dc, margin, width, height):

        text = 'avg:%.2f std:%.2f %s' % (self.avg, self.std_dev, self.title)

        (w, h) = dc.GetTextExtent(text)
        dc.SetBrush(wx.Brush(self.color))
        dc.DrawRectangle(width - h - margin, height, h, h)
        dc.DrawText(text, width - 2 * margin - w - h, height)
        return h

    def DrawCurve(self, dc, width, height, margin, _max_, _min_, x_axis):
        if width != self.size:
            self.SetPlotSize(width)
            return

        if (not self.real_time) and (x_axis is None):
            self.index = (self.index + 1) % self.size  # increment index to next point
            self.data[self.index] = None

        if x_axis is not None:
            (x_min, x_max) = x_axis.GetXMinMax()

        dc.SetPen(wx.Pen(self.color, 1))
        if _max_ < _min_:
            (_min_, _max_) = (-1, 1)  # prevent divide by zero or inversion
        if _max_ == _min_:
            (_min_, _max_) = (_max_ - 0.5, _max_ + 0.5)
        delta = _max_ - _min_
        dy = (height - margin * 2) / delta

        n = 0
        sums = 0.0
        sum_squares = 0.0
        lines = []
        point_1 = None
        for i in range(self.size):
            ix = (i + self.index) % self.size
            point = self.data[ix]
            if point is None:
                continue
            n += 1
            sums = sums + point
            sum_squares = sum_squares + (point * point)

            if x_axis is not None:
                x = x_axis.data[ix]
                if x is None:
                    continue
                dx = (width - 1) / (x_max - x_min)
                x = int((x - x_min) * dx)

            else:
                x = i * width / self.size

            scaled_point = (point + self.offset) * self.scale
            y = height - margin - int((scaled_point - _min_) * dy)

            if point_1 is not None:
                line = (point_1[0], point_1[1], x, y)
                lines.append(line)
            point_1 = (x, y)
            dc.DrawLineList(lines)

            if n > 0:
                self.avg = sums / n
                self.std_dev = math.sqrt(math.fabs((sum_squares / n) - (self.avg * self.avg)))

    def GetXMinMax(self):
        x_min = 1e32
        x_max = -1e32

        for i in range(self.size):
            point = self.data[i]
            if point is None:
                continue
            x_min = min(x_min, point)
            x_max = max(x_max, point)

        if x_max < x_min:
            (x_min, x_max) = (-1, 1)  # prevent divide by zero or inversion
        if x_max == x_min:
            (x_min, x_max) = (x_max - 0.5, x_max + 0.5)

        self.x_max = x_max
        self.x_min = x_min
        return (x_min, x_max)


_IVY_APPNAME = 'RealtimePlot'
_IVY_STRING = '(%s %s .*$)'


# _IVY_STRING = '^([^ ]*) +(%s( .*|$))'  ## <-- from original ocaml (doesn't work here, just returns Sender field...)

def create(parent, frame):
    return PlotPanel(parent, frame)


class PlotPanel(object):
    def __init__(self, parent, frame):
        self.parent = parent  # we are drawing on our parent, so dc comes from this
        self.frame = frame  # the frame owns any controls we might need to update

        parent.SetDropTarget(TextDropTarget(self))  # calls self.OnDropText when drag and drop complete

        self.width = 800
        self.height = 200
        self.margin = min(self.height / 10, 20)
        self.font = wx.Font(self.margin / 2, wx.DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.pixmap = wx.EmptyBitmap(self.width, self.height)
        self.plot_size = self.width
        self.max = -1e32
        self.min = 1e32
        self.plot_interval = 200
        self.plots = {}
        self.auto_scale = True
        self.offset = 0.0
        self.scale = 1.0
        self.x_axis = None

        messages_xml_map.parse_messages()

        self.ivy_interface = IvyMessagesInterface(_IVY_APPNAME)

        # start the timer
        self.timer = wx.FutureCall(self.plot_interval, self.OnTimer)

    def SetPlotInterval(self, value):
        self.plot_interval = value
        self.timer.Restart(self.plot_interval)
        self.timer = wx.FutureCall(self.plot_interval, self.OnTimer)

    def SetAutoScale(self, value):
        self.auto_scale = value

    def SetMin(self, value):
        self.min = value

    def SetMax(self, value):
        self.max = value

    def Pause(self, pause):
        if pause:
            self.timer.Stop()
        else:
            self.timer = wx.FutureCall(self.plot_interval, self.OnTimer)

    def ResetScale(self):
        self.max = -1e32
        self.min = 1e32

    def OnClose(self):
        self.timer.Stop()
        try:
            IvyStop()
        except IvyIllegalStateError as e:
            print(e)

    def OnErase(self, event):
        pass

    def ShowMessagePicker(self, parent):
        frame = messagepicker.MessagePicker(parent, self.BindCurve, self.ivy_interface)
        frame.Show()

    def OnDropText(self, data):
        [ac_id, category, message, field, scale] = data.encode('ASCII').split(':')
        self.BindCurve(int(ac_id), message, field, scale=float(scale))

    def OnIvyMsg(self, agent, *larg):
        # print(larg[0])
        data = larg[0].split(' ')
        ac_id = int(data[0])
        message = data[1]

        if ac_id not in self.plots:
            return

        if message not in self.plots[ac_id]:
            return

        for field in self.plots[ac_id][message]:
            plot = self.plots[ac_id][message][field]
            ix = messages_xml_map.message_dictionary["telemetry"][message].index(field)
            point = float(data[ix + 2])

            if self.x_axis is None or self.x_axis.id != plot.id:
                if self.auto_scale:
                    scaled_point = (point + plot.offset) * plot.scale
                    self.max = max(self.max, scaled_point)
                    self.min = min(self.min, scaled_point)

            if self.x_axis is not None:
                plot.index = self.x_axis.index
            plot.AddPoint(point, self.x_axis)

    def BindCurve(self, ac_id, message, field, color=None, use_as_x=False, scale=1.0):
        # -- add this telemetry to our list of things to plot ...
        message_string = _IVY_STRING % (ac_id, message)
        # print('Binding to %s' % message_string)

        if ac_id not in self.plots:
            self.plots[ac_id] = {}

        if message not in self.plots[ac_id]:
            self.plots[ac_id][message] = {}

        if field in self.plots[ac_id][message]:
            self.plots[ac_id][message][field].color = wx.Color(random.randint(0, 255), random.randint(0, 255),
                                                               random.randint(0, 255))
            return

        ivy_id = self.ivy_interface.bind_raw(self.OnIvyMsg, str(message_string))
        title = '%i:%s:%s' % (ac_id, message, field)
        self.plots[ac_id][message][field] = PlotData(ivy_id, title, self.plot_size, color, scale)
        self.frame.AddCurve(ivy_id, title, use_as_x)
        if use_as_x:
            self.x_axis = self.plots[ac_id][message][field]

    def CalcMinMax(self, plot):
        if not self.auto_scale: return
        for x in plot.data:
            self.max = max(self.max, x)
            self.min = min(self.min, x)
            self.frame.SetMinMax(self.min, self.max)

    def FindPlotName(self, ivy_id):
        for ac_id in self.plots:
            for msg in self.plots[ac_id]:
                for field in self.plots[ac_id][msg]:
                    if self.plots[ac_id][msg][field].id == ivy_id:
                        return (ac_id, msg, field)
        return (None, None, None)

    def FindPlot(self, ivy_id):
        (ac_id, msg, field) = self.FindPlotName(ivy_id)
        if ac_id is None:
            return None

        return self.plots[ac_id][msg][field]

    def RemovePlot(self, ivy_id):
        (ac_id, msg, field) = self.FindPlotName(ivy_id)
        if ac_id is None:
            return

        if (self.x_axis is not None) and (self.x_axis.id == ivy_id):
            self.x_axis = None

        self.ivy_interface.unbind(ivy_id)
        del self.plots[ac_id][msg][field]
        if len(self.plots[ac_id][msg]) == 0:
            del self.plots[ac_id][msg]

    def OffsetPlot(self, ivy_id, offset):
        plot = self.FindPlot(ivy_id)
        if plot is None:
            return

        plot.SetOffset(offset)
        print('panel value: %.2f' % value)
        CalcMinMax(plot)

    def ScalePlot(self, ivy_id, offset):
        plot = self.FindPlot(ivy_id)
        if plot is None:
            return

        plot.SetScale(offset)
        CalcMinMax(plot)

    def SetRealTime(self, ivy_id, value):
        plot = self.FindPlot(ivy_id)
        if plot is None:
            return

        plot.SetRealTime(value)

    def SetXAxis(self, ivy_id):
        plot = self.FindPlot(ivy_id)
        if plot is None:
            return

        self.x_axis = plot

    def ClearXAxis(self):
        self.x_axis = None

    def OnSize(self, size):
        (width, height) = size
        if self.width == width and self.height == height:
            return

        self.pixmap = wx.EmptyBitmap(width, height)
        self.width = width
        self.height = height
        self.plot_size = width
        self.margin = min(self.height / 10, 20)
        self.font = wx.Font(self.margin / 2, wx.DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

    def OnTimer(self):
        self.timer.Restart(self.plot_interval)
        self.frame.SetMinMax(self.min, self.max)
        self.DrawFrame()

    def DrawFrame(self):
        dc = wx.ClientDC(self.parent)
        bdc = wx.BufferedDC(dc, self.pixmap)
        bdc.SetBackground(wx.Brush("White"))
        bdc.Clear()

        self.DrawBackground(bdc, self.width, self.height)

        title_y = 2
        for ac_id in self.plots:
            for message in self.plots[ac_id]:
                for field in self.plots[ac_id][message]:
                    plot = self.plots[ac_id][message][field]
                    if (self.x_axis is not None) and (self.x_axis.id == plot.id):
                        continue
                    title_height = plot.DrawTitle(bdc, 2, self.width, title_y)
                    plot.DrawCurve(bdc, self.width, self.height, self.margin, self.max, self.min, self.x_axis)

                    title_y += title_height + 2

    def DrawBackground(self, dc, width, height):

        # Time Graduations
        dc.SetFont(self.font)

        if self.x_axis is None:
            t = self.plot_interval * width
            t1 = "0.0s"
            t2 = "-%.1fs" % (t / 2000.0)
            t3 = "-%.1fs" % (t / 1000.0)
        else:
            x_max = self.x_axis.x_max
            x_min = self.x_axis.x_min
            t1 = "%.2f" % x_max
            t2 = "%.2f" % (x_min + (x_max - x_min) / 2.0)
            t3 = "%.2f" % x_min

        (w, h) = dc.GetTextExtent(t1)
        dc.DrawText(t1, width - w, height - h)
        # (w,h) = dc.GetTextExtent(t2)             #save time since h will be the same
        dc.DrawText(t2, width / 2, height - h)
        # (w,h) = dc.GetTextExtent(t3)             #save time since h will be the same
        dc.DrawText(t3, 0, height - h)

        # Y graduations
        if self.max == -1e32:
            return

        (_min_, _max_) = (self.min, self.max)
        if _max_ < _min_:  # prevent divide by zero or inversion
            (_min_, _max_) = (-1, 1)
        if _max_ == _min_:
            (_min_, _max_) = (_max_ - 0.5, _max_ + 0.5)

        delta = _max_ - _min_
        dy = (height - self.margin * 2) / delta
        scale = math.log10(delta)
        d = math.pow(10.0, math.floor(scale))
        u = d
        if delta < 2 * d:
            u = d / 5
        elif delta < 5 * d:
            u = d / 2
        tick_min = _min_ - math.fmod(_min_, u)
        for i in range(int(delta / u) + 1):
            tick = tick_min + float(i) * u
            s = str(tick)
            (w, h) = dc.GetTextExtent(s)
            y = height - self.margin - int((tick - _min_) * dy) - h / 2
            dc.DrawText(s, 0, y)
