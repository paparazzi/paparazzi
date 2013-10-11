#! /usr/bin/python

# Tool for visualizing quaternion as rotated cube

from OpenGL.GLUT import *
from OpenGL.GLU import *
from  OpenGL.GL import *
import sys
import math
from ivy.std_api import *
import logging
import getopt

import pygame
import time
import platform
import os

_NAME = 'attitude_viz'


class TelemetryQuat:
    def __init__(self, message_name, index, name, integer):
        self.message_name = message_name
        self.index = index
        self.name = name
        self.qi = 1
        self.qx = 0
        self.qy = 0
        self.qz = 0
        # optional scaling for fixed point telemetry
        if integer:
            self.scale = 0.00003051757812
        else:
            self.scale = 1.0


class TelemetryValue:
    def __init__(self, message_name, index, name, offset, scale, max):
        self.message_name = message_name
        self.index = index
        self.name = name
        self.offset = offset
        self.scale = scale
        self.max = max
        self.value = 0


class Visualization:
    def __init__(self, parent):
        self.quats = []
        self.graph_values = []
        self.throttle = 0.0
        self.mode = 0.0
        self.airspeed = 0.0
        self.display_list = None
        self.display_dirty = True
        self.rotate_theta = parent.rotate_theta

        for message_name, index, name, bfp in VEHICLE_QUATS:
            self.quats.append(TelemetryQuat(message_name, index, name, bfp))
        for message_name, index, name, offset, scale, max in BAR_VALUES:
            self.graph_values.append(TelemetryValue(message_name, index, name, offset, scale, max))

    def onmsgproc(self, agent, *larg):
        data = str(larg[0]).split(' ')
        for telemetry_quat in self.quats:
            if telemetry_quat.message_name == data[1]:
                self.display_dirty = True
                telemetry_quat.qi = float(data[telemetry_quat.index + 0])
                telemetry_quat.qx = float(data[telemetry_quat.index + 1])
                telemetry_quat.qy = float(data[telemetry_quat.index + 2])
                telemetry_quat.qz = float(data[telemetry_quat.index + 3])

        for graph_value in self.graph_values:
            if graph_value.message_name == data[1]:
                self.display_dirty = True
                graph_value.value = (float(data[graph_value.index + 0]) + graph_value.offset) / graph_value.scale

    def DrawCircle(self, radius):
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, 0)
        for angle in range(0, 361, 12):
            glVertex3f(math.sin(math.radians(angle)) * radius, math.cos(math.radians(angle)) * radius, 0)
        glEnd()

    # draw quad centered at origin, z = 0
    def DrawQuad(self, width, height):
        glBegin(GL_QUADS)
        glVertex3f(width, height, 0)
        glVertex3f(-width, height, 0)
        glVertex3f(-width, -height, 0)
        glVertex3f(width, -height, 0)
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
        strutcount = 3
        discradius = 0.45
        discseparation = 0.01

        #wings
        glColor3f(0.1, 0.1, 0.9)
        glPushMatrix()
        glTranslate(0, 0, 0.05)
        self.DrawBox(wingspan, chord, thickness)
        glColor3f(0.0, 0.0, 0.0)
        glTranslate(-wingspan, -0.2, thickness + 0.01)
        glScale(0.004, 0.004, 0.004)
        for c in name:
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()

        glPushMatrix()
        glTranslate(0, 0, -0.05)
        glColor3f(0.6, 0.6, 0.2)
        self.DrawBox(wingspan, chord, thickness)
        glColor3f(0.0, 0.0, 0.0)
        glTranslate(wingspan, -0.2, -0.01 - thickness)
        glScale(0.004, 0.004, 0.004)
        glRotate(180, 0, 1, 0)
        for c in name:
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()

        if self.display_list is None:
            self.display_list = glGenLists(1)
            glNewList(self.display_list, GL_COMPILE)
            # struts
            glColor3f(0.4, 0.4, 0.4)
            glPushMatrix()
            glTranslate(-wingspan / 2, 0, separation / 2)
            glRotate(90, 0, 1, 0)
            for x in range(0, strutcount - 1):
                self.DrawBox(separation / 2, chord - .01, thickness)
                glTranslate(0, 0, wingspan)
            glTranslate(separation, 0, -5 * wingspan / 2)
            for x in range(0, strutcount - 1):
                self.DrawBox(separation / 2, chord - .01, thickness)
                glTranslate(0, 0, 2 * wingspan)
            glPopMatrix()

            #rotors
            glColor3f(0.9, 0.1, 0.1)
            glPushMatrix()
            glRotate(90, 1, 0, 0)
            glTranslate(-wingspan / 2, separation, -(chord + .01))
            for x in range(0, strutcount):
                if (x != strutcount / 2):
                    self.DrawCircle(discradius)
                glTranslate(2 * wingspan / (strutcount + 1), 0, 0)
            glPopMatrix()

            glPushMatrix()
            glRotate(90, 1, 0, 0)
            glTranslate(-wingspan, -separation, -(chord + .01))
            for x in range(0, strutcount):
                if (x != strutcount / 2):
                    self.DrawCircle(discradius)
                glTranslate(2 * wingspan / (strutcount - 1), 0, 0)
            glPopMatrix()
            glEndList()

        glCallList(self.display_list)

    def DrawBar(self, name, value):
        bar_height = 0.12
        bar_length = 3
        glPushMatrix()
        glColor3f(0, 0, 0)
        glTranslate(-bar_length, -0.09, 0.02)
        glScale(0.0015, 0.0015, 0.0015)
        for c in name:
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
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

        glPushMatrix()

        height = 5

        glDisable(GL_LIGHTING)
        glPushMatrix()
        for graph_value in self.graph_values:
            self.DrawBar(graph_value.name % (graph_value.value), graph_value.value / graph_value.max)
            glTranslate(0, 0.35, 0)
        glPopMatrix()
        glEnable(GL_LIGHTING)

        glTranslate(0, -height + (height / len(self.quats) + 1), 0)
        for telemetry_quat in self.quats:
            glPushMatrix()
            try:
                scaled_quat = [telemetry_quat.qi * telemetry_quat.scale, telemetry_quat.qx * telemetry_quat.scale,
                               telemetry_quat.qy * telemetry_quat.scale, telemetry_quat.qz * telemetry_quat.scale]
                glRotate(360 * math.acos(scaled_quat[0]) / math.pi, scaled_quat[2], -scaled_quat[3], -scaled_quat[1])
                glRotate(self.rotate_theta, 1, 0, 0)

                self.DrawVehicle(telemetry_quat.name)
            except Exception:
                raise Exception
            finally:
                glPopMatrix()
                glTranslate(0, 2 * height / (len(self.quats)), 0)
        glPopMatrix()


class Visualizer:
    def __init__(self, rotate_theta):
        self.rotate_theta = rotate_theta
        self.visualization = Visualization(self)

        # listen to Ivy
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyInit(_NAME,
                "",
                0,
                lambda x, y: y,
                lambda x, z: z)

        if os.getenv('IVY_BUS') is not None:
            IvyStart(os.getenv('IVY_BUS'))
        else:
            if platform.system() == 'Darwin':
                IvyStart("224.255.255.255:2010")
            else:
                IvyStart()

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
            IvyBindMsg(self.visualization.onmsgproc, bind_string)

    def Draw(self):
        if self.visualization.display_dirty:
            self.visualization.Draw()
            self.visualization.display_dirty = False

    def OnClose(self):
        IvyStop()


SCREEN_SIZE = (800, 800)


def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, float(width / height), .1, 100.)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glutInit()
    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_BLEND)
    glShadeModel(GL_SMOOTH)
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

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0.0, 0.0, 100.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0)


def run():
    global VEHICLE_QUATS, BAR_VALUES
    VEHICLE_QUATS = [["AHRS_REF_QUAT", 6, "Estimate", True], ["AHRS_REF_QUAT", 2, "Reference", True]]
    BAR_VALUES = [["ROTORCRAFT_RADIO_CONTROL", 5, "Throttle (%%) %i", 0, 100, 100]]
    window_title = "Attitude_Viz"
    rotate_theta = -90
    try:
        opts, args = getopt.getopt(sys.argv[1:], "t:r:", ["title", "rotate_theta"])
        for o, a in opts:
            if o in ("-t", "--title"):
                window_title = a
            if o in ("-r", "--rotate_theta"):
                rotate_theta = int(a)
    except getopt.error as msg:
        print(msg)
        print("""usage:
-t, --title                   set window title
-r, --rotate_theta           rotate the quaternion by n degrees over the pitch axis (default: -90)
""")
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE, pygame.OPENGL | pygame.DOUBLEBUF)
    #resize(*SCREEN_SIZE)
    init()
    visualizer = Visualizer(rotate_theta)

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    visualizer.OnClose()
                    return
                if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
                    visualizer.OnClose()
                    return
            visualizer.Draw()
            pygame.display.flip()
            time.sleep(.02)
    except KeyboardInterrupt:
        visualizer.OnClose()
        return


if __name__ == "__main__":
    run()
