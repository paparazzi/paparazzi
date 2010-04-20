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

DEFAULT_X = 800
DEFAULT_Y = 600

class TelemetryQuat:
  def __init__(self, message_name, index, name):
    self.message_name = message_name
    self.index = index
    self.name = name
    self.qi = 1
    self.qx = 0
    self.qy = 0
    self.qz = 0

class TelemetryValue:
  def __init__(self, message_name, index, name, offset, scale, max):
    self.message_name = message_name
    self.index = index
    self.name = name
    self.offset = offset
    self.scale = scale
    self.max = max
    self.value = 0

class MyGLCanvas(wx.glcanvas.GLCanvas):
    def __init__(self, parent):
      wx.glcanvas.GLCanvas.__init__(self, parent,-1)
      self.Bind( wx.EVT_PAINT, self.OnPaint)
      self.init = False
      self.quats = []
      self.graph_values = []
      self.throttle = 0.0
      self.mode = 0.0
      self.airspeed = 0.0
      for message_name, index, name in VEHICLE_QUATS:
	self.quats.append(TelemetryQuat(message_name, index, name))
      for message_name, index, name, offset, scale, max in BAR_VALUES:
	self.graph_values.append(TelemetryValue(message_name, index, name, offset, scale, max))


    def onmsgproc(self, agent, *larg):
      data = str(larg[0]).split(' ')
      for telemetry_quat in self.quats:
	if (telemetry_quat.message_name == data[1]):
	  telemetry_quat.qi = float(data[telemetry_quat.index + 0])
	  telemetry_quat.qx = float(data[telemetry_quat.index + 1])
	  telemetry_quat.qy = float(data[telemetry_quat.index + 2])
	  telemetry_quat.qz = float(data[telemetry_quat.index + 3])

      for graph_value in self.graph_values:
	if (graph_value.message_name == data[1]):
	  graph_value.value = (float(data[graph_value.index + 0]) + graph_value.offset) / graph_value.scale
  
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
      glEnable(GL_LINE_SMOOTH)
      glEnable(GL_DEPTH_TEST)
      glEnable(GL_LIGHTING)
      glEnable(GL_LIGHT0)
      glEnable(GL_BLEND)
      glShadeModel (GL_SMOOTH)
      glClearColor(1.0, 1.0, 1.0, 1.0)
      glClearDepth(1.0)
      
      glPointSize(3.0)

      glMatrixMode(GL_PROJECTION)
      glLoadIdentity()
      gluPerspective(7.0, 1.0, 95.0, 105.0)

      glMatrixMode(GL_MODELVIEW)

      glLight(GL_LIGHT0, GL_POSITION, [5, 30, -20])
      glLight(GL_LIGHT0, GL_AMBIENT, [0.5, 0.5, 0.5])
      glLight(GL_LIGHT0, GL_SPECULAR, [0.0, 0.0, 0.0])
      glLight(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8])
      glEnable(GL_COLOR_MATERIAL)
      glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)

    def DrawCircle(self, radius):
      glBegin(GL_TRIANGLE_FAN)
      glVertex3f(0, 0, 0)
      for angle in range (0, 361, 8):
	glVertex3f( math.sin(math.radians(angle)) * radius, math.cos(math.radians(angle)) * radius, 0)
      glEnd()
  

    # draw quad centered at origin, z = 0
    def DrawQuad(self, width, height):
      glBegin (GL_QUADS)
      glVertex3f( width, height, 0)
      glVertex3f( -width, height, 0)
      glVertex3f( -width, -height, 0)
      glVertex3f( width, -height, 0)
      glEnd()

    def DrawBox(self, width, height, depth):
      glPushMatrix()
      glTranslate(0, 0, depth)
      self.DrawQuad(width, height)
      glTranslate(0, 0, -2 * depth)
      self.DrawQuad(width, height)
      glPopMatrix()

      glPushMatrix()
      glRotate(90, 1, 0, 0)
      glTranslate(0, 0, height)
      self.DrawQuad(width, depth)
      glTranslate(0, 0, -2 * height)
      self.DrawQuad(width, depth)
      glPopMatrix()

      glPushMatrix()
      glRotate(90, 0, 1, 0)
      glTranslate(0, 0, width)
      self.DrawQuad(depth, height)
      glTranslate(0, 0, -2 * width)
      self.DrawQuad(depth, height)
      glPopMatrix()

    def DrawVehicle(self, name):
	wingspan = 2.7
	separation = 0.7
	chord = 0.35
	thickness = 0.08
	strutcount = 5
	discradius = 0.45
	discseparation = 0.01

	#wings
	glColor3f(0.1, 0.1, 0.9)
	glPushMatrix()
	glTranslate(0, 0, separation)
	self.DrawBox(wingspan, chord, thickness)
	glColor3f(0.0, 0.0, 0.0)
	glTranslate(-wingspan, -0.2, thickness + 0.01)
	glScale(0.004, 0.004, 0.004)
	glutStrokeString(GLUT_STROKE_ROMAN, name)
	glPopMatrix()

	glPushMatrix()
	glTranslate(0, 0, -separation)
	glColor3f(0.6, 0.6, 0.2)
	self.DrawBox(wingspan, chord, thickness)
	glColor3f(0.0, 0.0, 0.0)
	glTranslate(wingspan, -0.2, -0.01 - thickness)
	glScale(0.004, 0.004, 0.004)
	glRotate(180, 0, 1, 0)
	glutStrokeString(GLUT_STROKE_ROMAN, name)
	glPopMatrix()

	# struts
	glColor3f(0.4, 0.4, 0.4)
	glPushMatrix()
	glTranslate(-wingspan, 0, 0)
	glRotate(90, 0, 1, 0)
	for x in range (0, strutcount):
	  self.DrawBox(separation, chord - .01, thickness)
	  glTranslate(0, 0, 2 * wingspan/(strutcount - 1))
	glPopMatrix()

	#rotors
	glColor3f(0.9, 0.1, 0.1)
	glPushMatrix()
	glRotate(90, 1, 0, 0)
	glTranslate(-wingspan, separation, -(chord + .01))
	for x in range (0, strutcount):
	  if (x != strutcount/2):
	    self.DrawCircle(discradius)
	  glTranslate(2 * wingspan/(strutcount - 1), 0, 0)
	glPopMatrix()

	glPushMatrix()
	glRotate(90, 1, 0, 0)
	glTranslate(-wingspan, -separation, -(chord + .01))
	for x in range (0, strutcount):
	  if (x != strutcount/2):
	    self.DrawCircle(discradius)
	  glTranslate(2 * wingspan/(strutcount - 1), 0, 0)
	glPopMatrix()

    def DrawBar(self, name, value):
      bar_height = 0.12
      bar_length = 3
      glPushMatrix()
      glColor3f(0, 0, 0)
      glTranslate(-bar_length, -0.09, 0.02)
      glScale(0.0015, 0.0015, 0.0015)
      glutStrokeString(GLUT_STROKE_ROMAN, name)
      glPopMatrix()
      glColor3f(0.92, 0.92, 0.92)
      glPushMatrix()
      glTranslate(0, 0, 0)
      self.DrawQuad(bar_length, bar_height)
      glPopMatrix()
      glPushMatrix()
      glTranslate(bar_length * value - bar_length, 0, 0.01)
      glColor3f(0.6, 0.6, 0.6)
      self.DrawQuad(bar_length * value, bar_height)
      glPopMatrix()

    def Draw(self):
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

      glMatrixMode(GL_MODELVIEW)
      glLoadIdentity()
      gluLookAt(0.0, 0.0, 100.0,
                0.0, 0.0, 0.0,
                0.0, 1.0, 0.0)

      height = 5

      glDisable(GL_LIGHTING)
      glPushMatrix()
      for graph_value in self.graph_values:
	self.DrawBar(graph_value.name % (graph_value.value), graph_value.value / graph_value.max)
	glTranslate(0, 0.35, 0)
      glPopMatrix()
      glEnable(GL_LIGHTING)

      glTranslate(0, -height + (height / len(self.quats) + 1),  0)
      for telemetry_quat in self.quats:
        glPushMatrix()
        glRotate(360 * math.acos(telemetry_quat.qi) / math.pi, telemetry_quat.qy, -telemetry_quat.qz, -telemetry_quat.qx)
	self.DrawVehicle(telemetry_quat.name)
	glPopMatrix()
        glTranslate(0,  2 * height / (len(self.quats)), 0)

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

    # list of all message names
    messages = []

    # append all message names
    for vehicle_quat in VEHICLE_QUATS:
      messages.append(vehicle_quat[0])
    for bar_value in BAR_VALUES:
      messages.append(bar_value[0])
    
    # bind to set of messages (ie, only bind each message once)
    for message_name in set(messages): 
      bind_string = "(^.*" + message_name + ".*$)"
      IvyBindMsg(self.panel.onmsgproc, bind_string)
  def OnPaint(self, event):
    pass
	
  def GenerateGui( self):
    # generate panel and edit box
    self.size_x = DEFAULT_X
    self.size_y = DEFAULT_X

    self.panel = MyGLCanvas(self)
    self.panel.SetClientSize( (self.size_x, self.size_y))
  def InitLayout( self):
      box = wx.BoxSizer(wx.VERTICAL)
      box.Add( self.panel, 0, wx.EXPAND)
      box.Fit( self)
      self.SetSizer( box)
      self.Fit()
  def OnClose(self, event):
      IvyStop()
      self.Destroy()

      
if __name__ == "__main__":
  global VEHICLE_QUATS, BAR_VALUES
  VEHICLE_QUATS = [ ["AHRS_DEBUG_QUAT", 2, "JOBY"], ["AHRS_DEBUG_QUAT", 10, "POINE"], ["AHRS_DEBUG_QUAT", 6, "XSENS Estimation"], ["BOOZ2_AHRS_REF_QUAT", 2, "Reference"]]
  BAR_VALUES = [ ["AIRSPEED", 3, "Airspeed (m/s) %i", 0, 1, 40], ["BOOZ2_RADIO_CONTROL", 5, "Throttle (%%) %i", 9600, 96 * 2, 100], ["BOOZ2_RADIO_CONTROL", 6, "Mode %i", -9600, -9600, 2]]
  window_title = "Attitude_Viz"
  try:
    opts, args = getopt.getopt(sys.argv[1:], "t:",
      ["title"])
    for o,a in opts:
      if o in ("-t", "--title"):
	window_title = a
  except getopt.error, msg:
      print msg
      print """usage:
-t, --title                   set window title
"""

  app = wx.PySimpleApp()
  frame = MainWindow( None, -1, window_title)
  frame.Center()
  app.MainLoop()
