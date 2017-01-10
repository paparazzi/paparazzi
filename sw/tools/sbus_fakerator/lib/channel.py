'''
 Copyright (C) 2016 Kason Bennett, Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>

 This file is part of paparazzi.

 paparazzi is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2, or (at your option)
 any later version.

 paparazzi is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with paparazzi; see the file COPYING.  If not, see
 <http://www.gnu.org/licenses/>.

 Sbus fakerator: simulated SBUS radio for HITL testing
'''
from Tkinter import *
from lxml import etree
from _hotshot import resolution
from Tkconstants import HORIZONTAL, VERTICAL
import os

c_types = ["Bi-switch", "Tri-switch", "Trigger-switch", "Dial-360", "Dial-180", "Throttle", "Hor-stick", "Vert-stick"]

class Channel(Scale):
    '''
    @var value: current value of Channel

    '''

    def __init__(self, child, number, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.bind = ""
        self.number = number
        self.c_type = ""
        kw['to'] = -100
        kw['from_'] = 100
        kw['variable'] = self.value
        Scale.__init__(self, cnf, kw)

    def add_value(self, val):
        '''
        Add a number to val. To subtract, enter a negative number.
        @param val: value to add to Channel value
        '''
        self.value += val
        self.set(self.value)

    def rebind(self, key):
        self.bind = key

    def set_type(self, new_type):
        if new_type in c_types:
            self.c_type = new_type
        else:
            print "Type not known for {}".format(self.name)

    def repr_xml(self):
        c_root = etree.Element("Channel")
        c_root.attrib['key'] = self.bind
        c_root.attrib['name'] = self.name
        c_root.attrib['type'] = self.c_type
        return c_root

    def parse_xml(self, xml):
        self.bind = xml['key']
        self.name = xml['name']
        self.set_type(xml['type'])


class bi_switch(Channel):

    __name__ = "bi_switch"

    def __init__(self, number, orientation, name, scale_length, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.value = IntVar(value=-100)
        self.state = 0
        self.bind = ""
        self.number = number
        self.c_type = ""
        self.name = name
        Channel.__init__(self, number, master, cnf, label=name, length = scale_length, orient = orientation, variable=self.value)
        self.set(-100)


    def add_value(self, num):
        self.state = (self.state + 1) % 2
        if self.state == 0:
            self.value.set(-100)
        elif self.state == 1:
            self.value.set(100)

    def key_off(self):
        pass


class tri_switch(Channel):

    __name__ = "tri_switch"

    def __init__(self, number, orientation, name, scale_length, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.value = IntVar(value=-100)
        self.state = 0
        self.bind = ""
        self.number = number
        self.c_type = ""
        self.name = name
        Channel.__init__(self, number, master, cnf, label = name, length = scale_length, resolution=100, orient = orientation, variable=self.value)

    def add_value(self, in_state):
        '''
        @param val: value to add to Channel value
        '''
        # 0=-100
        # 1=0
        # 2=100

        if in_state > 0:
            if self.value.get() == -100:
                self.value.set(0)
            elif self.value.get() == 0:
                self.value.set(100)
        elif in_state < 0:
            if self.value.get() == 100:
                self.value.set(0)
            elif self.value.get() == 0:
                self.value.set(-100)

    def key_off(self):
        pass

class trigger_switch(Channel):

    __name__ = "trigger_switch"

    def __init__(self, number, orientation, name, scale_length, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.value = IntVar(value=-100)
        self.state = 0
        self.bind = ""
        self.number = number
        self.c_type = ""
        self.name = name
        Channel.__init__(self, number, master, cnf, label = name, length = scale_length, orient=orientation, variable=self.value)

    def add_value(self, state):
        '''
        @param state: state of the switch (i.e., flipped or not)
        '''
        os.system('xset r off')
        if state < 0:
            self.value.set(-100)
        else:
            self.value.set(100)

    def key_off(self):
        os.system('xset r on')
        self.add_value(-1)

class dial(Channel):

    __name__ = "dial"
    def __init__(self, number, total_degrees, orientation, name, scale_length, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.value = DoubleVar(0)
        self.t_degrees = total_degrees
        self.resolution = 200.0 / total_degrees
        self.bind = ""
        self.number = number
        self.c_type = ""
        self.name = name
        Channel.__init__(self, number, master, cnf, label = name, length = scale_length, orient = orientation, resolution=self.resolution, variable=self.value)

    def add_value(self, in_value):
        '''
        @param state: state of the switch (i.e., flipped or not)
        '''
        self.value.get()
        self.value.set((in_value * self.resolution) + self.value.get())

    def key_off(self):
        pass

class stick(Channel):
    __name__ = "stick"
    def __init__(self, number, orientation, name, scale_length, master=None, cnf={}, **kw):
        '''
        Constructor
        @param number: Channel number
        '''
        self.value = IntVar(0)
        self.bind = ""
        self.number = number
        self.c_type = ""
        Channel.__init__(self, number, master, cnf, orient = orientation, label = name, length = scale_length, variable=self.value)

    def add_value(self, value):
        '''
        @param state: state of the switch (i.e., flipped or not)
        '''
        self.value.set(self.value.get() + value)

    def key_off(self):
        pass

