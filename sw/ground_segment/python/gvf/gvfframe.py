import wx
import time

from scipy import linalg as la
from matplotlib.path import Path
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pl
import matplotlib.patches as patches
import numpy as np

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
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
        self.altitude = 0
        self.ground_altitude = -1

        # Desired trajectory
        self.s = 0
        self.ke = 0
        self.gvf_error = 0
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
                self.altitude = float(msg.get_field(4))/1000
            if msg.name == 'NAVIGATION':
                self.XY[0] = float(msg.get_field(2))
                self.XY[1] = float(msg.get_field(3))
            if msg.name == 'NAVIGATION_REF':
                self.ground_altitude = float(msg.get_field(3))
            if msg.name == 'ATTITUDE':
                self.yaw = float(msg.get_field(1))
            if msg.name == 'GVF':
                self.gvf_error = float(msg.get_field(0))
                # Straight line
                if int(msg.get_field(1)) == 0:
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
                elif int(msg.get_field(1)) == 1:
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
                elif int(msg.get_field(1)) == 2:
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

            if msg.name == 'GVF_PARAMETRIC':
                # Trefoil 2D
                if int(msg.get_field(0)) == 0:
                    self.s = int(msg.get_field(1))
                    self.wb = float(msg.get_field(2))
                    param = [float(x) for x in msg.get_field(3)]
                    xo = param[0]
                    yo = param[1]
                    w1 = param[2]
                    w2 = param[3]
                    ratio = param[4]
                    r = param[5]
                    alpha = param[6]
                    phi = [float(x) for x in msg.get_field(4)]
                    phi_x = phi[0]
                    phi_y = phi[1]
                    self.traj = traj_param_trefoil_2D(np.array([xo, yo]), w1, w2, ratio, r, alpha, self.wb)

                # Ellipse 3D
                elif int(msg.get_field(0)) == 1:
                    self.s = int(msg.get_field(1))
                    self.wb = float(msg.get_field(2))
                    param = [float(x) for x in msg.get_field(3)]
                    xo = param[0]
                    yo = param[1]
                    r = param[2]
                    zl = param[3]
                    zh = param[4]
                    alpha = param[5]
                    phi = [float(x) for x in msg.get_field(4)]
                    phi_x = phi[0]
                    phi_y = phi[1]
                    phi_z = phi[2]
                    self.traj = traj_param_ellipse_3D(np.array([xo,yo]), r, zl, zh, alpha, self.wb)

                # Lissajous 3D
                elif int(msg.get_field(0)) == 2:
                    self.s = int(msg.get_field(1))
                    self.wb = float(msg.get_field(2))
                    param = [float(x) for x in msg.get_field(3)]
                    xo = param[0]
                    yo = param[1]
                    zo = param[2]
                    cx = param[3]
                    cy = param[4]
                    cz = param[5]
                    wx = param[6]
                    wy = param[7]
                    wz = param[8]
                    dx = param[9]
                    dy = param[10]
                    dz = param[11]
                    alpha = param[12]
                    phi = [float(x) for x in msg.get_field(4)]
                    phi_x = phi[0]
                    phi_y = phi[1]
                    phi_z = phi[2]
                    self.traj = traj_param_lissajous_3D(np.array([xo,yo]), zo, cx, cy, cz, \
                            wx, wy, wz, dx, dy, dz, alpha, self.wb)

    def draw_gvf(self, XY, yaw, course, altitude):
        if self.traj is not None:
            self.map_gvf.draw(XY, yaw, course, altitude, self.traj)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_gvf(self.XY, self.yaw, self.course, self.altitude-self.ground_altitude)
        self.canvas.draw()

class map2d:
    def __init__(self, XYoff, area):
        self.XYoff = XYoff
        self.area = area
        self.fig = pl.figure()

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

    def draw(self, XY, yaw, course, altitude, traj):
        self.fig.clf()
        if traj.dim == 2:
            ax = self.fig.add_subplot(111)
            ax.plot(traj.traj_points[0, :], traj.traj_points[1, :])
            ax.quiver(traj.mapgrad_X, traj.mapgrad_Y, \
                    traj.mapgrad_U, traj.mapgrad_V, color='Teal', \
                    pivot='mid', width=0.002)
            ax.add_patch(self.vehicle_patch(XY, yaw)) # In radians
            apex = 45*np.pi/180 # 30 degrees apex angle
            b = np.sqrt(2*(self.area/2000) / np.sin(apex))
            h = b*np.cos(apex/2)
            ax.arrow(XY[0], XY[1], \
                    h*np.sin(course), h*np.cos(course),\
                    head_width=5, head_length=10, fc='k', ec='k')
            ax.annotate('HOME', xy = (0, 0))
            if isinstance(traj, traj_ellipse):
                ax.annotate('ELLIPSE', xy = (traj.XYoff[0], traj.XYoff[1]))
                ax.plot(0, 0, 'kx', ms=10, mew=2)
                ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
            elif isinstance(traj, traj_sin):
                ax.annotate('SIN', xy = (traj.XYoff[0], traj.XYoff[1]))
                ax.plot(0, 0, 'kx', ms=10, mew=2)
                ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
            elif isinstance(traj, traj_line):
                ax.annotate('LINE', xy = (traj.XYoff[0], traj.XYoff[1]))
                ax.plot(0, 0, 'kx', ms=10, mew=2)
                ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
            elif isinstance(traj, traj_param_trefoil_2D):
                ax.annotate('TREFOIL', xy = (traj.XYoff[0], traj.XYoff[1]))
                ax.plot(0, 0, 'kx', ms=10, mew=2)
                ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
                ax.plot(traj.wpoint[0], traj.wpoint[1], 'rx', ms=10, mew=2)

            ax.set_xlabel('South [m]')
            ax.set_ylabel('West [m]')
            ax.set_title('2D Map')
            ax.set_xlim(self.XYoff[0]-0.5*np.sqrt(self.area), \
               self.XYoff[0]+0.5*np.sqrt(self.area))
            ax.set_ylim(self.XYoff[1]-0.5*np.sqrt(self.area), \
               self.XYoff[1]+0.5*np.sqrt(self.area))
            ax.axis('equal')
            ax.grid()

        if traj.dim == 3:
            a3d = self.fig.add_subplot(2,2,1, projection='3d')
            axy = self.fig.add_subplot(2,2,2)
            axz = self.fig.add_subplot(2,2,3)
            ayz = self.fig.add_subplot(2,2,4)

            a3d.set_title('3D Map')
            axy.set_title('XY Map')
            axz.set_title('XZ Map')
            ayz.set_title('YZ Map')

            # 3D
            a3d.plot(traj.traj_points[0, :], traj.traj_points[1, :], traj.traj_points[2, :])
            if altitude != -1:
                a3d.plot([XY[0]], [XY[1]], [altitude], marker='o', markerfacecolor='r', markeredgecolor='r')
                a3d.plot([traj.wpoint[0]], [traj.wpoint[1]], [traj.wpoint[2]], marker='x', markerfacecolor='r', markeredgecolor='r')

            #a3d.axis('equal')
            if traj.deltaz < 0:
                a3d.set_zlim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                a3d.set_zlim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)

            # XY
            axy.plot(traj.traj_points[0, :], traj.traj_points[1, :])
            axy.add_patch(self.vehicle_patch(XY, yaw)) # In radians
            apex = 45*np.pi/180 # 30 degrees apex angle
            b = np.sqrt(2*(self.area/2000) / np.sin(apex))
            h = b*np.cos(apex/2)
            axy.arrow(XY[0], XY[1], \
                    h*np.sin(course), h*np.cos(course),\
                    head_width=5, head_length=10, fc='k', ec='k')
            axy.annotate('HOME', xy = (0, 0))
            axy.plot(traj.wpoint[0], traj.wpoint[1], 'rx', ms=10, mew=2)
            if isinstance(traj, traj_param_ellipse_3D):
                axy.annotate('ELLIPSE_3D', xy = (traj.XYoff[0], traj.XYoff[1]))
                axy.plot(0, 0, 'kx', ms=10, mew=2)
                axy.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
            elif isinstance(traj, traj_param_lissajous_3D):
                axy.annotate('LISSA_3D', xy = (traj.XYoff[0], traj.XYoff[1]))
                axy.plot(0, 0, 'kx', ms=10, mew=2)
                axy.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)

            axy.axis('equal')

            # XZ
            axz.plot(traj.traj_points[0, :], traj.traj_points[2, :])
            if altitude != -1:
                axz.plot([XY[0]], [altitude], 'ro')
                axz.plot(traj.wpoint[0], traj.wpoint[2], 'rx', ms=10, mew=2)
            if traj.deltaz < 0:
                axz.set_ylim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                axz.set_ylim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)
            # YZ
            ayz.plot(traj.traj_points[1, :], traj.traj_points[2, :])
            if altitude != -1:
                ayz.plot([XY[1]], [altitude], 'ro')
                ayz.plot(traj.wpoint[1], traj.wpoint[2], 'rx', ms=10, mew=2)
            if traj.deltaz < 0:
                axz.set_ylim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                ayz.set_ylim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)

class traj_line:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, Xminmax, a, b, alpha):
        self.dim = 2
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
        self.dim = 2
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
        self.dim = 2
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

class traj_param_trefoil_2D:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, XYoff, w1, w2, ratio, r, alpha, wb):
        self.dim = 2
        self.XYoff, self.w1, self.w2, self.ratio, self.r, self.alpha, self.wb = XYoff, w1, w2, ratio, r, alpha, wb
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        self.alpha = alpha*np.pi/180

        self.wpoint = self.param_point(self.wb)

        num_points = 1000
        self.traj_points = np.zeros((2, num_points))

        i = 0
        range_points = 1000.0
        for t in self.float_range(0, range_points, range_points/num_points):
            self.traj_points[:, i] = self.param_point(t)
            i = i + 1
            if i >= num_points:
                break

    def param_point(self, t):
        xnr = np.cos(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio)
        ynr = np.sin(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio)

        x = np.cos(self.alpha)*xnr - np.sin(self.alpha)*ynr
        y = np.sin(self.alpha)*xnr + np.cos(self.alpha)*ynr

        return self.XYoff + np.array([x,y])

class traj_param_ellipse_3D:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, XYoff, r, zl, zh, alpha, wb):
        self.dim = 3
        self.XYoff, self.r, self.zl, self.zh, self.alpha, self.wb = XYoff, r, zl, zh, alpha, wb
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        self.deltaz = self.zh - self.zl # For the 3D plot
        self.zo = self.zl + self.deltaz # For the 3D plot
        self.alpha = self.alpha*np.pi/180

        self.wpoint = self.param_point(self.wb)

        num_points = 100
        self.traj_points = np.zeros((3, num_points))

        i = 0
        range_points = 2*np.pi + 0.1
        for t in self.float_range(0, range_points, range_points/num_points):
            self.traj_points[:, i] = self.param_point(t)
            i = i + 1
            if i >= num_points:
                break

    def param_point(self, t):
        x = self.r * np.cos(t) + self.XYoff[0]
        y = self.r * np.sin(t) + self.XYoff[1]
        z = 0.5 * (self.zh + self.zl + (self.zl - self.zh) * np.sin(self.alpha - t))

        return np.array([x,y,z])

class traj_param_lissajous_3D:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, XYoff, zo, cx, cy, cz, wx, wy, wz, dx, dy, dz, alpha, wb):
        self.dim = 3
        self.XYoff, self.zo, self.cx, self.cy, self.cz, self.wx, self.wy, self.wz, self.dx, self.dy, self.dz, \
                self.alpha, self.wb = XYoff, zo, cx, cy, cz, wx, wy, wz, dx, dy, dz, alpha, wb
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []

        self.deltaz = self.cz # For the 3D plot
        self.alpha = self.alpha*np.pi/180
        self.dx = self.dx*np.pi/180
        self.dy = self.dy*np.pi/180
        self.dz = self.dz*np.pi/180

        self.wpoint = self.param_point(self.wb)

        smallest_w = min([self.wx, self.wy, self.wz])

        num_points = 100
        self.traj_points = np.zeros((3, num_points))

        i = 0
        range_points = 3*np.pi / smallest_w
        for t in self.float_range(0, range_points, range_points/num_points):
            self.traj_points[:, i] = self.param_point(t)
            i = i + 1
            if i >= num_points:
                break

    def param_point(self, t):
        xnr = self.cx*np.cos(self.wx*t + self.dx)
        ynr = self.cy*np.cos(self.wy*t + self.dy)
        z = self.cz*np.cos(self.wz*t + self.dz) + self.zo

        x = np.cos(self.alpha)*xnr - np.sin(self.alpha)*ynr + self.XYoff[0]
        y = np.sin(self.alpha)*xnr + np.cos(self.alpha)*ynr + self.XYoff[1]

        return np.array([x,y,z])
