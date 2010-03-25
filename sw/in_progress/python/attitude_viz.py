#! /usr/bin/python

# Tool for visualizing quaternion as rotated cube

import wx
import wx.glcanvas
from OpenGL.GLUT import *
from OpenGL.GLU import *
from  OpenGL.GL import *
import sys
import math
from ivy.std_api import *
import logging
import getopt

_NAME = 'attitude_viz'

DEFAULT_X = 320
DEFAULT_Y = 240

class MyGLCanvas(wx.glcanvas.GLCanvas):
    def __init__(self, parent):
      wx.glcanvas.GLCanvas.__init__(self, parent,-1)
      self.Bind( wx.EVT_PAINT, self.OnPaint)
      self.init = False
      self.qi = 1
      self.qx = 0
      self.qy = 0
      self.qz = 0

    def onmsgproc(self, agent, *larg):
      data = str(larg[0]).split(' ')
      quat_start = DATA_INDEX
      self.qi = float(data[quat_start + 0])
      self.qx = float(data[quat_start + 1])
      self.qy = float(data[quat_start + 2])
      self.qz = float(data[quat_start + 3])
  
    def OnPaint(self,event):
      if not self.init:
        self.SetCurrent()
        self.InitGL()

      try:
        self.Draw()
      except:
        raise 
      finally:
        self.SwapBuffers()
        self.Refresh()
	
    def InitGL(self):
      self.init = True
      
      glutInit()
      # set viewing projection

      glEnable(GL_LINE_SMOOTH)
      glEnable(GL_DEPTH_TEST)
      glClearColor(1.0, 1.0, 1.0, 1.0)
      glClearDepth(1.0)
      
      glPointSize(3.0)

      glMatrixMode(GL_PROJECTION)
      glLoadIdentity()
      gluPerspective(50.0, 1.0, 1.0, 30.0)

      glMatrixMode(GL_MODELVIEW)
      glLoadIdentity()
      gluLookAt(0.0, 0.0, 10.0,
                0.0, 0.0, 0.0,
                0.0, 1.0, 0.0)

    def Draw(self):
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

      glMatrixMode(GL_MODELVIEW)
      glLoadIdentity()
      gluLookAt(0.0, 0.0, 10.0,
                0.0, 0.0, 0.0,
                0.0, 1.0, 0.0)

      glRotate(ROTATION_DIRECTION * 360 * math.acos(self.qi) / math.pi, self.qy, -self.qz, -self.qx)
      if (ROTATE_180):
	glRotate(180, 0, 1, 0)
  
      span = 3 * 0.8
      height = 3 * 0.5
      length = 3 * 0.3
      
      glBegin (GL_QUADS)
      glColor3f( 0.9, 0.0, 0.0)
      glVertex3f( span, length, height)
      glVertex3f( -span, length, height)
      glVertex3f( -span, -length, height)
      glVertex3f( span, -length, height)
      glEnd()

      glBegin (GL_QUADS)
      glColor3f( 0.0, 0.8, 0.8)
      glVertex3f( span, length, -height)
      glVertex3f( -span, length, -height)
      glVertex3f( -span, -length, -height)
      glVertex3f( span, -length, -height)
      glEnd()

      glBegin (GL_QUADS)
      glColor3f( 0.0, 0.6, 0.6)
      glVertex3f( span, length, height)
      glVertex3f( span, -length, height)
      glVertex3f( span, -length, -height)
      glVertex3f( span, length, -height)
      glEnd()

      glBegin (GL_QUADS)
      glColor3f( 0.6, 0.0, 0.0)
      glVertex3f( -span, length, height)
      glVertex3f( -span, -length, height)
      glVertex3f( -span, -length, -height)
      glVertex3f( -span, length, -height)
      glEnd()

      glBegin (GL_QUADS)
      glColor3f( 0.0, 0.6, 0.0)
      glVertex3f(  span, length, height)
      glVertex3f( -span, length, height)
      glVertex3f( -span, length, -height)
      glVertex3f( span, length, -height)
      glEnd()

      glBegin (GL_QUADS)
      glColor3f( 0.6, 0.0, 0.6)
      glVertex3f(  span, -length, height)
      glVertex3f( -span, -length, height)
      glVertex3f( -span, -length, -height)
      glVertex3f( span, -length, -height)
      glEnd()

class MainWindow(wx.Frame):
  """ simple wx.Frame derived class. """
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, wx.ID_ANY, title, size=(200,100),
                      style=wx.DEFAULT_FRAME_STYLE|wx.NO_FULL_REPAINT_ON_RESIZE)

    self.GenerateGui()
    self.InitLayout()

    # bind painting events
    self.Bind( wx.EVT_CLOSE, self.OnClose)
    self.Bind( wx.EVT_PAINT, self.OnPaint)

    # display the window
    self.Show( True)

    # listen to Ivy
    logging.getLogger('Ivy').setLevel(logging.WARN)
    IvyInit(_NAME, 
      "", 
      0, 
      lambda x,y: y, 
      lambda x,z: z 
    )
    IvyStart("")
    bind_string = "(^.*" + MESSAGE_NAME + ".*$)"
    IvyBindMsg(self.panel.onmsgproc, bind_string)
  def OnPaint(self, event):
    pass
	
  def GenerateGui( self):
    # generate panel and edit box
    self.size_x = DEFAULT_X
    self.size_y = DEFAULT_X

    self.edit = wx.TextCtrl(self)
    self.panel = MyGLCanvas(self)
    self.panel.SetClientSize( (self.size_x, self.size_y))
  def InitLayout( self):
      box = wx.BoxSizer(wx.VERTICAL)
      box.Add( self.panel, 0, wx.EXPAND)
      box.Add( self.edit, 0, wx.EXPAND)
      box.Fit( self)
      self.SetSizer( box)
      self.Fit()
  def OnClose(self, event):
      IvyStop()
      self.Destroy()

      
if __name__ == "__main__":
  global DATA_INDEX
  DATA_INDEX = 6
  ROTATION_DIRECTION = 1
  ROTATE_180 = 0
  MESSAGE_NAME = "BOOZ2_AHRS_REF_QUAT"
  TITLE = "Attitude_Viz"
  try:
    opts, args = getopt.getopt(sys.argv[1:], "braod:t:m:",
      ["double-buffer", 
       "reverse",
       "rot",
       "title",
       "message",
       "data-index"])
    for o,a in opts:
      if o in ("-b", "--double-buffer"):
        doubleBuffer = True
      elif o in ("-d", "--data-index"):
        DATA_INDEX = int(a)
      elif o in ("-r", "--reverse"):
	ROTATION_DIRECTION=-1
      elif o in ("-o", "--rot"):
	print "rot"
	ROTATE_180=1
      elif o in ("-t", "--title"):
	TITLE=a
      elif o in ("-m", "--message"):
	MESSAGE_NAME=a
      elif o in ("-a", "--alt-message"):
	MESSAGE_NAME = "AHRS_DEBUG"
  except getopt.error, msg:
      print msg
      print """usage:
-b, --double-buffer           enable double buffer
-d, --data-index              set data index
-r, --reverse                 reverse rotation direction
-o, --rot                     rotate 180 in roll
-t, --title                   set window title
-m, --message                 set message name
-a, --alt-message             use alternate message name
"""

  app = wx.PySimpleApp()
  frame = MainWindow( None, -1, TITLE)
  frame.Center()
  app.MainLoop()
