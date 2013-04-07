#Boa:Frame:PlotFrame

from __future__ import division

import wx
import plotpanel


_INITIAL_TIME_VALUE_ = 0.2  # initial refresh rate in seconds

def create(parent):
    return PlotFrame(parent)

[wxID_PLOTFRAME, wxID_PLOTFRAMECHECKAUTOSCALE, wxID_PLOTFRAMEEDITMAX, wxID_PLOTFRAMEEDITMIN, wxID_PLOTFRAMEEDITTIME, wxID_PLOTFRAMEPANEL1, wxID_PLOTFRAMESLIDERTIME, wxID_PLOTFRAMESTATICTEXT1, wxID_PLOTFRAMESTATICTEXT2, wxID_PLOTFRAMESTATICTEXT3] = [wx.NewId() for _init_ctrls in range(10)]

[wxID_PLOTFRAMEMENU1ITEM_ADD, wxID_PLOTFRAMEMENU1ITEM_PAUSE, wxID_PLOTFRAMEMENU1ITEM_RESET] = [wx.NewId() for _init_coll_menuPlot_Items in range(3)]

class PlotFrame(wx.Frame):
    def _init_coll_boxSizer1_Items(self, parent):
        # generated method, don't edit

        parent.AddSizer(self.boxSizer2, 0, border=0, flag=0)
        parent.AddWindow(self.panel1, 1, border=0, flag=wx.EXPAND)

    def _init_coll_boxSizer2_Items(self, parent):
        # generated method, don't edit

        parent.AddWindow(self.checkAutoScale, 0, border=0, flag=wx.ALIGN_CENTER_VERTICAL)
        parent.AddWindow(self.staticText1, 0, border=0, flag=wx.ALIGN_CENTER_VERTICAL)
        parent.AddWindow(self.editMin, 0, border=0, flag=0)
        parent.AddWindow(self.staticText2, 0, border=0, flag=wx.ALIGN_CENTER_VERTICAL)
        parent.AddWindow(self.editMax, 0, border=0, flag=0)
        parent.AddWindow(self.staticText3, 0, border=0, flag=wx.ALIGN_CENTER_VERTICAL)
        parent.AddWindow(self.sliderTime, 0, border=0, flag=wx.ALIGN_CENTER_VERTICAL)
        parent.AddWindow(self.editTime, 0, border=0, flag=0)

    def _init_coll_menuBar1_Menus(self, parent):
        # generated method, don't edit

        parent.Append(menu=self.menuPlot, title=u'Plot')
        parent.Append(menu=self.menuCurves, title=u'Curves')

    def _init_coll_menuPlot_Items(self, parent):
        # generated method, don't edit

        parent.Append(help=u'Add plots', id=wxID_PLOTFRAMEMENU1ITEM_ADD, kind=wx.ITEM_NORMAL, text=u'&Add\tCtrl+A')
        parent.Append(help=u'Reset plot scale', id=wxID_PLOTFRAMEMENU1ITEM_RESET, kind=wx.ITEM_NORMAL, text=u'&Reset\tCtrl+L')
        parent.Append(help=u'Pause the plot', id=wxID_PLOTFRAMEMENU1ITEM_PAUSE, kind=wx.ITEM_CHECK, text=u'&Pause\tCtrl+P')
        self.Bind(wx.EVT_MENU, self.OnMenu1Item_addMenu, id=wxID_PLOTFRAMEMENU1ITEM_ADD)
        self.Bind(wx.EVT_MENU, self.OnMenu1Item_resetMenu, id=wxID_PLOTFRAMEMENU1ITEM_RESET)
        self.Bind(wx.EVT_MENU, self.OnMenu1Item_pauseMenu, id=wxID_PLOTFRAMEMENU1ITEM_PAUSE)

    def _init_sizers(self):
        # generated method, don't edit
        self.boxSizer1 = wx.BoxSizer(orient=wx.VERTICAL)

        self.boxSizer2 = wx.BoxSizer(orient=wx.HORIZONTAL)

        self._init_coll_boxSizer1_Items(self.boxSizer1)
        self._init_coll_boxSizer2_Items(self.boxSizer2)

        self.SetSizer(self.boxSizer1)

    def _init_utils(self):
        # generated method, don't edit
        self.menuPlot = wx.Menu(title='')

        self.menuCurves = wx.Menu(title='')

        self.menuBar1 = wx.MenuBar()

        self._init_coll_menuPlot_Items(self.menuPlot)
        self._init_coll_menuBar1_Menus(self.menuBar1)

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_PLOTFRAME, name=u'PlotFrame', parent=prnt, pos=wx.Point(476, 365), size=wx.Size(800, 225), style=wx.DEFAULT_FRAME_STYLE, title=u'Real Time Plot')
        self._init_utils()
        self.SetMenuBar(self.menuBar1)
        self.SetClientSize(wx.Size(800, 225))

        self.checkAutoScale = wx.CheckBox(id=wxID_PLOTFRAMECHECKAUTOSCALE, label=u'Auto scale', name=u'checkAutoScale', parent=self, pos=wx.Point(0, 2), size=wx.Size(93, 22), style=0)
        self.checkAutoScale.SetValue(True)
        self.checkAutoScale.Bind(wx.EVT_CHECKBOX, self.OnCheckAutoScaleCheckbox, id=wxID_PLOTFRAMECHECKAUTOSCALE)

        self.staticText1 = wx.StaticText(id=wxID_PLOTFRAMESTATICTEXT1, label=u'min', name='staticText1', parent=self, pos=wx.Point(93, 5), size=wx.Size(68, 17), style=wx.ALIGN_RIGHT)

        self.editMin = wx.TextCtrl(id=wxID_PLOTFRAMEEDITMIN, name=u'editMin', parent=self, pos=wx.Point(161, 0), size=wx.Size(80, 27), style=0, value=u'')
        self.editMin.Enable(False)
        self.editMin.Bind(wx.EVT_TEXT, self.OnEditMinText, id=wxID_PLOTFRAMEEDITMIN)

        self.staticText2 = wx.StaticText(id=wxID_PLOTFRAMESTATICTEXT2, label=u'max', name='staticText2', parent=self, pos=wx.Point(241, 5), size=wx.Size(68, 17), style=wx.ALIGN_RIGHT)

        self.editMax = wx.TextCtrl(id=wxID_PLOTFRAMEEDITMAX, name=u'editMax', parent=self, pos=wx.Point(309, 0), size=wx.Size(80, 27), style=0, value=u'')
        self.editMax.Enable(False)
        self.editMax.Bind(wx.EVT_TEXT, self.OnEditMaxText, id=wxID_PLOTFRAMEEDITMAX)

        self.staticText3 = wx.StaticText(id=wxID_PLOTFRAMESTATICTEXT3, label=u'interval', name='staticText3', parent=self, pos=wx.Point(389, 5), size=wx.Size(68, 17), style=wx.ALIGN_RIGHT)

        self.sliderTime = wx.Slider(id=wxID_PLOTFRAMESLIDERTIME, maxValue=1000, minValue=1, name=u'sliderTime', parent=self, pos=wx.Point(457, 4), size=wx.Size(200, 19), style=wx.SL_HORIZONTAL, value=_INITIAL_TIME_VALUE_ * 1000)
        self.sliderTime.SetLabel(u'')
        self.sliderTime.Bind(wx.EVT_COMMAND_SCROLL, self.OnSliderTimeCommandScroll, id=wxID_PLOTFRAMESLIDERTIME)

        self.editTime = wx.TextCtrl(id=wxID_PLOTFRAMEEDITTIME, name=u'editTime', parent=self, pos=wx.Point(657, 0), size=wx.Size(80, 27), style=wx.TE_PROCESS_ENTER, value="%0.2f" % _INITIAL_TIME_VALUE_)
        self.editTime.Bind(wx.EVT_TEXT_ENTER, self.OnEditTimeTextEnter, id=wxID_PLOTFRAMEEDITTIME)

        self.panel1 = wx.Panel(id=wxID_PLOTFRAMEPANEL1, name='panel1', parent=self, pos=wx.Point(0, 27), size=wx.Size(800, 200), style=wx.TAB_TRAVERSAL)

        self._init_sizers()

    def __init__(self, parent):
        self._init_ctrls(parent)

        self.canvas = plotpanel.create(self.panel1, self)
        self.dynamic_menus = {}

        self.Bind( wx.EVT_CLOSE, self.OnClose)
        self.Bind( wx.EVT_ERASE_BACKGROUND, self.OnErase)
        self.panel1.Bind( wx.EVT_RIGHT_DOWN, self.OnRightDown)
        self.panel1.Bind( wx.EVT_SIZE, self.OnSize)

    def OnRightDown(self, event):
        self.PopupMenu(self.menuPlot, event.GetPosition())

    def AddPlot(self, ac_id, message, field, color = None, x_axis = False):
        self.canvas.BindCurve(ac_id, message, field, color, x_axis)

    def SetMinMax(self, min_, max_):
        self.editMin.SetValue(str(min_))
        self.editMax.SetValue(str(max_))

    def OnClose(self, event):
        # need to forward close to canvas so that ivy is shut down, otherwise ivy hangs the shutdown
        self.canvas.OnClose()
        self.Destroy()

    def OnErase(self, event):
        pass

    def OnSize(self, event):
        self.canvas.OnSize( event.GetSize())

    def OnSliderTimeCommandScroll(self, event):
        value = event.GetPosition()
        self.canvas.SetPlotInterval(value)
        self.editTime.SetValue( '%.3f' % (value/1000.0))

    def OnEditTimeTextEnter(self, event):
        try:
            value = int(float(event.GetString()) * 1000.0)
        except:
            value = 0
        if value < 1 or value > 1000:
            value = '%.3f' % (self.sliderTime.GetValue() / 1000.0)
            self.editTime.SetValue( value)
            return
        self.canvas.SetPlotInterval(value)
        self.sliderTime.SetValue(value)

    def OnCheckAutoScaleCheckbox(self, event):
        value = self.checkAutoScale.GetValue()
        self.editMin.Enable( not value)
        self.editMax.Enable( not value)
        self.canvas.SetAutoScale(value)

    def OnMenu1Item_addMenu(self, event):
        self.canvas.ShowMessagePicker(self)

    def OnMenu1Item_resetMenu(self, event):
        self.canvas.ResetScale()

    def OnMenu1Item_pauseMenu(self, event):
        self.canvas.Pause(event.IsChecked())

    def AddCurve(self, menu_id, title, use_as_x = False):
        curveMenu = wx.Menu(title='')

        curveMenu.Append(help=u'Delete plot', id=menu_id*10, kind=wx.ITEM_NORMAL, text=u'&Delete')
        curveMenu.Append(help=u'Offset plot', id=menu_id*10+1, kind=wx.ITEM_NORMAL, text=u'&Offset')
        curveMenu.Append(help=u'Scale plot', id=menu_id*10+2, kind=wx.ITEM_NORMAL, text=u'&Scale')
        curveMenu.Append(help=u'Plot data as messages are received rather than async', id=menu_id*10+3, kind=wx.ITEM_CHECK, text=u'&Real time plot')
        curveMenu.Append(help=u'Use this curve as the X-axis rather than a time based scale', id=menu_id*10+4, kind=wx.ITEM_CHECK, text=u'&Use as X-axis')

        curveMenu.Check(id=menu_id*10+4, check=bool(use_as_x))

        self.Bind(wx.EVT_MENU, self.OnMenuDeleteCurve, id=menu_id*10)
        self.Bind(wx.EVT_MENU, self.OnMenuOffsetCurve, id=menu_id*10+1)
        self.Bind(wx.EVT_MENU, self.OnMenuScaleCurve, id=menu_id*10+2)
        self.Bind(wx.EVT_MENU, self.OnMenuRealTime, id=menu_id*10+3)
        self.Bind(wx.EVT_MENU, self.OnMenuUseAsXAxis, id=menu_id*10+4)

        self.dynamic_menus[menu_id] = self.menuCurves.AppendSubMenu(submenu=curveMenu, text=title)

    def OnMenuDeleteCurve(self, event):
        menu_id = event.GetId() // 10
        item = self.dynamic_menus[menu_id]
        self.canvas.RemovePlot(menu_id)
        self.menuCurves.DestroyItem(item)
        del self.dynamic_menus[menu_id]

    def OnMenuOffsetCurve(self, event):
        menu_id = (event.GetId()-1) // 10

        default_value = str(self.canvas.FindPlot(menu_id).offset)
        value = wx.GetTextFromUser("Enter a value to offset the plot", "Offset", default_value)
        try:
            value = float(value)
            self.canvas.OffsetPlot(menu_id, value)
        except:
            pass

    def OnMenuScaleCurve(self, event):
        menu_id = (event.GetId()-2) // 10

        default_value = str(self.canvas.FindPlot(menu_id).scale)
        value = wx.GetTextFromUser("Enter a factor to scale the plot", "Scale", default_value)
        try:
            value = float(value)
            self.canvas.ScalePlot(menu_id, value)
        except:
            pass

    def OnMenuRealTime(self,event):
        menu_id = (event.GetId()-3) // 10
        self.canvas.SetRealTime(menu_id, event.IsChecked())

    def OnMenuUseAsXAxis(self,event):
        event_id = event.GetId()
        menu_id = (event_id-4) // 10
        value = event.IsChecked()

        if value:
            # go through and clear the checks from any other curves
            for i in self.dynamic_menus:
                for item in self.dynamic_menus[i].GetSubMenu().GetMenuItems():
                    if item.GetText() == u'_Use as X-axis' and event_id != item.GetId():
                        item.Check(False)
            self.canvas.SetXAxis(menu_id)
        else:
            self.canvas.ClearXAxis()


    def OnEditMinText(self, event):
        try:
            value = float(event.GetString())
            self.canvas.SetMin(value)
        except:
            pass

    def OnEditMaxText(self, event):
        try:
            value = float(event.GetString())
            self.canvas.SetMax(value)
        except:
            pass
