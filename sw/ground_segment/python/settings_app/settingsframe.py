#Boa:Frame:PlotFrame

from __future__ import division

import wx
import sys
import os

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from settings_tool import IvySettingsInterface

def create(parent, ac_ids):
    return SettingsFrame(parent, ac_ids)

SLIDER_ID_OFFSET = 30000
BUTTON_ID_OFFSET = 31000
SLIDER_FACTOR = 100

# Wraps TextCtrl to provide added functionality
class TextCtrlSetting(wx.TextCtrl):
    update_callback = None
    def __init__(self, parent, setting):
        self.setting = setting
        wx.TextCtrl.__init__(self, parent=parent, name=setting.shortname, id=setting.index)
        self.Bind(wx.EVT_TEXT, self.onEvtText, self)

    def RegisterUpdateCallback(self, cb):
        self.update_callback = cb

    def onEvtText(self, event):
        index = int(self.GetId())
        try:
            value = float(self.GetValue())
            self.update_callback(index, value)
        except:
            return

    # helper function to toggle edit box boldness (bold = user-set, normal=downlink-received)
    def setBold(self, bold):
        font = self.GetFont()
        if (bold):
            font.SetWeight(wx.FONTWEIGHT_BOLD)
        else:
            font.SetWeight(wx.FONTWEIGHT_NORMAL)
        self.SetFont(font)

    def SetSettingValue(self, value):
        if (self.setting.step < 1):
            self.SetValue("%.2f" % float(value))
        else:
            self.SetValue("%i" % int(float(value)))

# Wraps slider
class SettingCtrl(wx.Slider):
    update_callback = None
    def __init__(self, parent, setting):
        self.setting = setting
        max_v = int(setting.max_value) * SLIDER_FACTOR
        min_v = int(setting.min_value) * SLIDER_FACTOR
        if (min_v >= max_v):
            max_v = max_v + 1
        wx.Slider.__init__(self, parent=parent, minValue=min_v, maxValue=max_v, style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS, size=(200, 30), id=setting.index + SLIDER_ID_OFFSET)
        self.SetLineSize(setting.step * SLIDER_FACTOR)
        self.Bind(wx.EVT_MOUSEWHEEL, self.sliderWheel, self)
        self.Bind(wx.EVT_SLIDER, self.OnEvtSlider, self)

    def RegisterUpdateCallback(self, cb):
        self.update_callback = cb

    def OnEvtSlider(self, event):
        slider = event.GetEventObject()
        self.update_callback(slider.GetSettingIndex(), slider.GetSettingValue())

    # Called on mouse wheel events (default handler seems backwards?)
    def sliderWheel(self, event):
        slider = event.GetEventObject()
        if (event.GetWheelRotation() > 0):
            slider.SetValue(slider.GetValue() + slider.GetLineSize())
        else:
            slider.SetValue(slider.GetValue() - slider.GetLineSize())
        self.update_callback(slider.GetSettingIndex(), slider.GetSettingValue())

    def GetSettingIndex(self):
        index = int(self.GetId())
        if index >= SLIDER_ID_OFFSET:
            index = index - SLIDER_ID_OFFSET
        return index

    def SetSettingValue(self, value):
        self.SetValue(int(float(value)) * SLIDER_FACTOR)

    def GetSettingValue(self):
        if (self.setting.step < 1):
            return float(self.GetValue()) / SLIDER_FACTOR
        else:
            return int(self.GetValue()) // SLIDER_FACTOR

class SettingsFrame(wx.Frame):
    edits = []
    sliders = []
    def __init__(self, parent, ac_ids):
        self.settings = IvySettingsInterface(ac_ids)
        title = "Settings %s (%s)" % (ac_ids, self.settings.GetACName())
        wx.Frame.__init__(self, name=u'SettingsFrame', parent=parent, title=title, size=(480, 320))
        self.book = wx.Notebook(self)
        self.updates = []
        self.Bind( wx.EVT_CLOSE, self.OnClose)

        for setting_group in self.settings.groups:
            page = wx.Panel(self.book)
            vert_box = wx.BoxSizer(orient=wx.VERTICAL)

            for setting in setting_group.member_list:
                horz_box = wx.BoxSizer(orient=wx.HORIZONTAL)

                text = wx.StaticText(page, label=setting.shortname, size=(100,30))

                # Edit
                edit = TextCtrlSetting(page, setting)
                edit.RegisterUpdateCallback(self.editUpdate)
                self.edits.append(edit)

                # Slider
                slider = SettingCtrl(page, setting)
                slider.RegisterUpdateCallback(self.updateEditFromSlider)
                self.sliders.append(slider)

                # Button
                button = wx.Button(page, id=setting.index + BUTTON_ID_OFFSET, label="Apply")
                self.Bind(wx.EVT_BUTTON, self.onButton)

                horz_box.AddWindow(text)
                horz_box.AddWindow(edit)
                horz_box.AddWindow(slider)
                horz_box.AddWindow(button)

                vert_box.AddWindow(horz_box)

                page.SetSizer(vert_box)

            self.book.AddPage(page, setting_group.name)

        self.settings.RegisterCallback(self.onUpdate)

    # Copy slider value into associated edit box
    def updateEditFromSlider(self, index, value):
        self.edits[index].ChangeValue(str(value))
        self.edits[index].setBold(True)

    # Called on edit box update
    def editUpdate(self, index, value):
        self.sliders[index].SetSettingValue(value)
        self.edits[index].setBold(True)

    # Called on button push
    def onButton(self, event):
        button = event.GetEventObject()
        index = int(button.GetId())
        if index >= BUTTON_ID_OFFSET:
            index = index - BUTTON_ID_OFFSET
            self.settings.lookup[index].value = self.sliders[index].GetSettingValue()
            self.settings.SendSetting(index)

    # Called for remote settings updates
    def onUpdate(self, index, value, fromRemote):
        # Schedule the call for later via wx (run after events)
        # to prevent crashy crashy
        wx.CallAfter(self.update_value, index, value, fromRemote)

    # Called to update GUI with new values
    def update_value(self, index, value, fromRemote):
        editCtrl = self.edits[index]
        if fromRemote and editCtrl.FindFocus() == editCtrl:
            # don't process remote updates if the control is focused
            return
        editCtrl.SetSettingValue(value)
        editCtrl.setBold(not fromRemote)
        self.sliders[index].SetSettingValue(value)

    def OnClose(self, event):
        # need to forward close to canvas so that ivy is shut down, otherwise ivy hangs the shutdown
        self.settings.OnClose()
        self.Destroy()
