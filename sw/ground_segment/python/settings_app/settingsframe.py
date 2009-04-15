#Boa:Frame:PlotFrame

import wx
import sys
import os

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/lib/python")

from settings_tool import IvySettingsInterface

def create(parent, ac_ids):
    return SettingsFrame(parent, ac_ids)

SLIDER_ID_OFFSET = 250000
BUTTON_ID_OFFSET = 2 * 250000
SLIDER_FACTOR = 100

class SettingsFrame(wx.Frame):
    edits = []
    sliders = []
    buttons = []
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
            edit = wx.TextCtrl(page, name=setting.shortname, id=setting.index)
            self.setBold(edit, True)
            edit.SetValue(str(setting.value))

            max_v = int(setting.max_value) * SLIDER_FACTOR
            min_v = int(setting.min_value) * SLIDER_FACTOR
            if (min_v >= max_v):
              max_v = max_v + 1
            slider = wx.Slider(page, minValue=min_v, maxValue=max_v, style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS, size=(200, 30), id=setting.index + SLIDER_ID_OFFSET)
            slider.SetLineSize(setting.step * SLIDER_FACTOR)

            button = wx.Button(page, id=setting.index + BUTTON_ID_OFFSET, label="Apply")

            self.Bind(wx.EVT_SLIDER, self.sliderUpdate, slider)
            slider.Bind(wx.EVT_MOUSEWHEEL, self.sliderWheel, slider)
            self.Bind(wx.EVT_TEXT, self.editUpdate, edit)
            self.Bind(wx.EVT_BUTTON, self.onButton)
            
            self.edits.append(edit)
            self.sliders.append(slider)
            self.buttons.append(button)

            horz_box.AddWindow(text)
            horz_box.AddWindow(edit)
            horz_box.AddWindow(slider)
            horz_box.AddWindow(button)
            
            vert_box.AddWindow(horz_box)
            
          page.SetSizer(vert_box)
          self.book.AddPage(page, setting_group.name)

        self.settings.RegisterCallback(self.onUpdate)

    # Called on mouse wheel events (default handler seems backwards?)
    def sliderWheel(self, event):
      slider = event.GetEventObject()
      if (event.GetWheelRotation() > 0):
        slider.SetValue(slider.GetValue() + slider.GetLineSize())
      else:
        slider.SetValue(slider.GetValue() - slider.GetLineSize())
      self.updateEditFromSlider(slider)

    # Called on slider update
    def sliderUpdate(self, event):
      slider = event.GetEventObject()
      self.updateEditFromSlider(slider)

    # Copy slider value into associated edit box
    def updateEditFromSlider(self, slider):
      index = int(slider.GetId())
      if index >= SLIDER_ID_OFFSET:
        index = index - SLIDER_ID_OFFSET
      if (self.settings.lookup[index].step < 1):
        value = float(slider.GetValue()) / SLIDER_FACTOR
      else:
        value = int(slider.GetValue()) / SLIDER_FACTOR
      self.edits[index].ChangeValue(str(value))
      self.setBold(self.edits[index], True)


    # Called on edit box update
    def editUpdate(self, event):
      edit = event.GetEventObject()
      index = int(edit.GetId())
      try:
        value = float(edit.GetValue())
      except:
        return
      self.sliders[index].SetValue(value * SLIDER_FACTOR)
      self.setBold(self.edits[index], True)

    # Called on button push
    def onButton(self, event):
      button = event.GetEventObject()
      index = int(button.GetId())
      if index >= BUTTON_ID_OFFSET:
        index = index - BUTTON_ID_OFFSET
      self.settings.lookup[index].value = float(self.sliders[index].GetValue()) / SLIDER_FACTOR
      self.settings.SendSetting(index)

    def SetAndSendValue(self, index, value):
      self.edits[index].SetValue("%.2f" % value)      
      button = self.buttons[index]
      click_event = wx.CommandEvent(wx.wxEVT_COMMAND_BUTTON_CLICKED, button.GetId())
      click_event.SetEventObject(button)
      button.Command(click_event)

    # Called for remote settings updates
    def onUpdate(self, index, value, fromRemote):
      # Schedule the call for later via wx (run after events)
      # to prevent crashy crashy
      wx.CallAfter(self.update_value, index, value, fromRemote)
     
    # helper function to toggle edit box boldness (bold = user-set, normal=downlink-received)
    def setBold(self, editCtrl, bold):
      font = editCtrl.GetFont()
      if (bold):
        font.SetWeight(wx.FONTWEIGHT_BOLD)
      else:
        font.SetWeight(wx.FONTWEIGHT_NORMAL)
      editCtrl.SetFont(font)

    # Called to update GUI with new values
    def update_value(self, index, value, fromRemote):
        editCtrl = self.edits[index]
        slider = self.sliders[index]
        if fromRemote and editCtrl.FindFocus() == editCtrl:
          # don't process remote updates if the control is focused
          return
        if (self.settings.lookup[index].step < 1):
          editCtrl.SetValue("%.2f" % float(value))
        else:
          editCtrl.SetValue("%i" % int(float(value)))
        self.setBold(editCtrl, not fromRemote)
        slider.SetValue(int(float(value) * SLIDER_FACTOR))
        
    def OnClose(self, event):
      # need to forward close to canvas so that ivy is shut down, otherwise ivy hangs the shutdown
      self.settings.OnClose()
      self.Destroy()
        
