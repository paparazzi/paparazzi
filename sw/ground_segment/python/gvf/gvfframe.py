import wx
import time

from scipy import linalg as la
from matplotlib.path import Path
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import matplotlib.pyplot as pl
import matplotlib.patches as patches
import numpy as np

import sys
from os import path, getenv
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

WIDTH = 800
HEIGHT = 800

class GVFFrame(wx.Frame):
    def __init__(self, ac_id):

        wx.Frame.__init__(self, id=-1, parent=None, \
                name=u'GVF', size=wx.Size(WIDTH, HEIGHT), \
                style=wx.DEFAULT_FRAME_STYLE, title=u'Guidance Vector Field')

        # Vehicle variables
        self.ac_id = ac_id
        self.course = 0
        self.yaw = 0
        self.XY = np.array([0, 0])

        # Desired trajectory
        self.timer_traj = 0 # We do not update the traj every time we receive a msg
        self.timer_traj_lim = 7 # (7+1) * 0.25secs
        self.s = 0
        self.ke = 0
        self.map_gvf = map2d(np.array([0, 0]), 150000)
        self.traj = None

        # Frame
        self.canvas = FigureCanvas(self, -1, self.map_gvf.fig)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRedrawTimer, self.redraw_timer)
        self.redraw_timer.Start(100)

        # Ivy
        self.interface = IvyMessagesInterface("GVF")
        self.interface.subscribe(self.message_recv)
        settings = PaparazziACSettings(ac_id)

    def message_recv(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            if msg.name == 'GPS':
                self.course = int(msg.get_field(3))*np.pi/1800
            if msg.name == 'NAVIGATION':
                self.XY[0] = float(msg.get_field(2))
                self.XY[1] = float(msg.get_field(3))
            if msg.name == 'ATTITUDE':
                self.yaw = float(msg.get_field(1))
            if msg.name == 'GVF':
                self.gvf_error = float(msg.get_field(0))
                # Straight line
                if int(msg.get_field(1)) == 0 \
                        and self.timer_traj == self.timer_traj_lim:
                    self.s = int(msg.get_field(2))
                    self.ke = float(msg.get_field(3))
                    param = [float(x) for x in msg.get_field(4)]
                    a = param[0]
                    b = param[1]
                    c = param[2]

                    self.traj = traj_line(np.array([-100,100]), a, b, c)
                    self.traj.vector_field(self.traj.XYoff, self.map_gvf.area, \
                            self.s, self.ke)

                # Ellipse
                if int(msg.get_field(1)) == 1 \
                        and self.timer_traj == self.timer_traj_lim:
                    self.s = int(msg.get_field(2))
                    self.ke = float(msg.get_field(3))
                    param = [float(x) for x in msg.get_field(4)]
                    ex = param[0]
                    ey = param[1]
                    ea = param[2]
                    eb = param[3]
                    ealpha = param[4]
                    self.traj = traj_ellipse(np.array([ex, ey]), ealpha, ea, eb)
                    self.traj.vector_field(self.traj.XYoff, \
                            self.map_gvf.area, self.s, self.ke)

                # Sin
                if int(msg.get_field(1)) == 2 \
                        and self.timer_traj == self.timer_traj_lim:
                    self.s = int(msg.get_field(2))
                    self.ke = float(msg.get_field(3))
                    param = [float(x) for x in msg.get_field(4)]
                    a = param[0]
                    b = param[1]
                    alpha = param[2]
                    w = param[3]
                    off = param[4]
                    A = param[5]
                    self.traj = traj_sin(np.array([-100, 100]), a, b, alpha, \
                            w, off, A)
                    self.traj.vector_field(self.traj.XYoff, \
                            self.map_gvf.area, self.s, self.ke)

                self.timer_traj = self.timer_traj + 1
                if self.timer_traj > self.timer_traj_lim:
                    self.timer_traj = 0

    def draw_gvf(self, XY, yaw, course):
        if self.traj is not None:
            self.map_gvf.draw(XY, yaw, course, self.traj)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_gvf(self.XY, self.yaw, self.course)
        self.canvas.draw()

class map2d:
    def __init__(self, XYoff, area):
        self.XYoff = XYoff
        self.area = area
        self.fig, self.ax = pl.subplots()
        self.ax.set_xlabel('South [m]')
        self.ax.set_ylabel('West [m]')
        self.ax.set_title('2D Map')
        self.ax.annotate('HOME', xy = (0, 0))
        self.ax.set_xlim(XYoff[0]-0.5*np.sqrt(area), XYoff[0]+0.5*np.sqrt(area))
        self.ax.set_ylim(XYoff[1]-0.5*np.sqrt(area), XYoff[1]+0.5*np.sqrt(area))
        self.ax.axis('equal')

    def vehicle_patch(self, XY, yaw):
        Rot = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])

        apex = 45*np.pi/180 # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        a = b*np.sin(apex/2)
        h = b*np.cos(apex/2)

        z1 = np.array([a/2, -h*0.3])
        z2 = np.array([-a/2, -h*0.3])
        z3 = np.array([0, h*0.6])

        z1 = Rot.dot(z1)
        z2 = Rot.dot(z2)
        z3 = Rot.dot(z3)

        verts = [(XY[0]+z1[0], XY[1]+z1[1]), \
                 (XY[0]+z2[0], XY[1]+z2[1]), \
                 (XY[0]+z3[0], XY[1]+z3[1]), \
                 (0, 0)]

        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
        path = Path(verts, codes)

        return patches.PathPatch(path, facecolor='red', lw=2)

    def draw(self, XY, yaw, course, traj):
        self.ax.clear()
        self.ax.plot(traj.traj_points[0, :], traj.traj_points[1, :])
        self.ax.quiver(traj.mapgrad_X, traj.mapgrad_Y, \
                traj.mapgrad_U, traj.mapgrad_V, color='Teal', \
                pivot='mid', width=0.002)
        self.ax.add_patch(self.vehicle_patch(XY, yaw)) # In radians
        apex = 45*np.pi/180 # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        h = b*np.cos(apex/2)
        self.ax.arrow(XY[0], XY[1], \
                h*np.sin(course), h*np.cos(course),\
                head_width=5, head_length=10, fc='k', ec='k')
        self.ax.annotate('HOME', xy = (0, 0))
        if isinstance(traj, traj_ellipse):
            self.ax.annotate('ELLIPSE', xy = (traj.XYoff[0], traj.XYoff[1]))
            self.ax.plot(0, 0, 'kx', ms=10, mew=2)
            self.ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
        elif isinstance(traj, traj_sin):
            self.ax.annotate('SIN', xy = (traj.XYoff[0], traj.XYoff[1]))
            self.ax.plot(0, 0, 'kx', ms=10, mew=2)
            self.ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
        elif isinstance(traj, traj_line):
            self.ax.annotate('LINE', xy = (traj.XYoff[0], traj.XYoff[1]))
            self.ax.plot(0, 0, 'kx', ms=10, mew=2)
            self.ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)

        self.ax.set_xlabel('South [m]')
        self.ax.set_ylabel('West [m]')
        self.ax.set_title('2D Map')
        self.ax.set_xlim(self.XYoff[0]-0.5*np.sqrt(self.area), \
                self.XYoff[0]+0.5*np.sqrt(self.area))
        self.ax.set_ylim(self.XYoff[1]-0.5*np.sqrt(self.area), \
                self.XYoff[1]+0.5*np.sqrt(self.area))
        self.ax.axis('equal')
        self.ax.grid()

class traj_line:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, Xminmax, a, b, alpha):
        self.XYoff = np.array([a, b])
        self.Xminmax = Xminmax
        self.a, self.b, self.alpha = a, b, alpha
        self.traj_points = np.zeros((2, 200))
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        i = 0
        for t in self.float_range(0, 1, 0.005):
            x = (self.Xminmax[1]-self.Xminmax[0])*t + self.Xminmax[0]
            i = i + 1

        xtr = np.linspace(-200, 200, 400)

        xl =  xtr*np.sin(self.alpha) + a
        yl =  xtr*np.cos(self.alpha) + b

        self.traj_points = np.vstack((xl, yl))

    def param_point(self, t):
        i = 0

    def vector_field(self, XYoff, area, s, ke):
        self.mapgrad_X, self.mapgrad_Y = np.mgrid[XYoff[0]-0.5*np.sqrt(area):\
                XYoff[0]+0.5*np.sqrt(area):30j, \
                XYoff[1]-0.5*np.sqrt(area):\
                XYoff[1]+0.5*np.sqrt(area):30j]

        nx = -np.cos(self.alpha)
        ny =  np.sin(self.alpha)
        tx = s*ny
        ty = -s*nx

        ke = 1e-2*ke

        e = (self.mapgrad_X-self.a)*nx + (self.mapgrad_Y-self.b)*ny

        self.mapgrad_U = tx -ke*e*nx
        self.mapgrad_V = ty -ke*e*ny

        norm = np.sqrt(self.mapgrad_U**2 + self.mapgrad_V**2)

        self.mapgrad_U = self.mapgrad_U/norm
        self.mapgrad_V = self.mapgrad_V/norm

class traj_ellipse:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, XYoff, rot, a, b):
        self.XYoff = XYoff
        self.a, self.b = a, b
        self.rot = rot
        self.traj_points = np.zeros((2, 200))
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        i = 0
        for t in self.float_range(0, 1, 0.005):
            self.traj_points[:, i] = self.param_point(t)
            i = i + 1

    def param_point(self, t):
        angle = 2*np.pi*t
        return self.XYoff \
                + np.array([self.a*np.cos(angle)*np.cos(-self.rot) - \
                self.b*np.sin(angle)*np.sin(-self.rot), \
                self.a*np.cos(angle)*np.sin(-self.rot) + \
                self.b*np.sin(angle)*np.cos(-self.rot)])

    def vector_field(self, XYoff, area, s, ke):
        self.mapgrad_X, self.mapgrad_Y = np.mgrid[XYoff[0]-0.5*np.sqrt(area):\
                XYoff[0]+0.5*np.sqrt(area):30j, \
                XYoff[1]-0.5*np.sqrt(area):\
                XYoff[1]+0.5*np.sqrt(area):30j]

        Xel = (self.mapgrad_X-self.XYoff[0])*np.cos(self.rot) \
                - (self.mapgrad_Y-self.XYoff[1])*np.sin(self.rot)

        Yel = (self.mapgrad_X-self.XYoff[0])*np.sin(self.rot) \
                + (self.mapgrad_Y-self.XYoff[1])*np.cos(self.rot)

        nx = 2*Xel*np.cos(self.rot)/self.a**2 \
                + 2*Yel*np.sin(self.rot)/self.b**2
        ny = -2*Xel*np.sin(self.rot)/self.a**2 \
                + 2*Yel*np.cos(self.rot)/self.b**2

        tx = s*ny
        ty = -s*nx

        e = (Xel/self.a)**2 + (Yel/self.b)**2 - 1

        self.mapgrad_U = tx -ke*e*nx
        self.mapgrad_V = ty -ke*e*ny

        norm = np.sqrt(self.mapgrad_U**2 + self.mapgrad_V**2)

        self.mapgrad_U = self.mapgrad_U/norm
        self.mapgrad_V = self.mapgrad_V/norm

class traj_sin:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, Xminmax, a, b, alpha, w, off, A):
        self.XYoff = np.array([a, b])
        self.Xminmax = Xminmax
        self.a, self.b, self.alpha, self.w, self.off, self.A = \
                a, b, alpha, w, off, A
        self.traj_points = np.zeros((2, 200))
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        i = 0
        for t in self.float_range(0, 1, 0.005):
            x = (self.Xminmax[1]-self.Xminmax[0])*t + self.Xminmax[0]
            i = i + 1

        xtr = np.linspace(-200, 200, 400)
        ytr = self.A*np.sin(self.w*xtr + self.off)

        xsin = -xtr*np.sin(self.alpha) + ytr*np.cos(self.alpha) + a
        ysin =  xtr*np.cos(self.alpha) + ytr*np.sin(self.alpha) + b

        self.traj_points = np.vstack((xsin, ysin))

    def param_point(self, t):
        i = 0

    def vector_field(self, XYoff, area, s, ke):
        self.mapgrad_X, self.mapgrad_Y = np.mgrid[XYoff[0]-0.5*np.sqrt(area):\
                XYoff[0]+0.5*np.sqrt(area):30j, \
                XYoff[1]-0.5*np.sqrt(area):\
                XYoff[1]+0.5*np.sqrt(area):30j]

        xs =  (self.mapgrad_X-self.XYoff[0])*np.sin(self.alpha) \
                - (self.mapgrad_Y-self.XYoff[1])*np.cos(self.alpha)

        ys =  -(self.mapgrad_X-self.XYoff[0])*np.cos(self.alpha) \
                - (self.mapgrad_Y-self.XYoff[1])*np.sin(self.alpha)

        ang = self.w*xs + self.off

        nx =  -np.cos(self.alpha) - \
                self.A*self.w*np.cos(ang)*np.sin(self.alpha)
        ny =  -np.sin(self.alpha) + \
                self.A*self.w*np.cos(ang)*np.cos(self.alpha)
        tx = s*ny
        ty = -s*nx

        ke = 1e-2*ke

        e = ys - self.A*np.sin(ang)

        self.mapgrad_U = tx -ke*e*nx
        self.mapgrad_V = ty -ke*e*ny

        norm = np.sqrt(self.mapgrad_U**2 + self.mapgrad_V**2)

        self.mapgrad_U = self.mapgrad_U/norm
        self.mapgrad_V = self.mapgrad_V/norm
