'''
 Copyright (C) 2016 Kason Benett, Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>

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
import Tkinter as tk

_key_bindings_ = dict()

class Controller(tk.Frame):

    def __init__(self, parent, *args, **kwargs):
        '''
        Constructor
        '''
        tk.Frame.__init__(self, parent, *args, width = 1000, height = 1000)
        self.num_of_channels = 0
        self.channels = []

    def add_channel(self, channel_type, *args, **kwargs):
        '''
        @param orientation = tk.VERTICAL : Either tk.VERTICAL or tk.HORIZONTAL
        @param name = "" : name of the channel
        @param x = 0 : x location in the frame
        @param y = 0: y location in the frame
        @param length = 100: length of the representation of the scale
        @param add_key_binding = 'a' : which button press adds to the channel
        @param sub_key_binding = 'z' : which button press subtracts from the channel
        @param key_binding = 'q' : which button press activates the button
        '''
        print "adding channels {}".format(channel_type.__name__)
        #kwarg unpacking
        orientation = 0
        name = ""
        x = 0
        y = 0
        length = 100
        if 'orientation' in kwargs:
            orientation = kwargs['orientation']
        else:
            orientation = tk.VERTICAL
        if 'name' in kwargs:
            name = kwargs['name']
        if 'x' in kwargs:
            x = kwargs['x']
        if 'y' in kwargs:
            y = kwargs['y']
        if 'length' in kwargs:
            length = kwargs['length']


        #channel adding
        if channel_type.__name__ == 'bi_switch':
            self.channels.append(channel_type(self.num_of_channels, orientation, name, length, self))
            if 'key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['key_binding']
                _key_bindings_[kwargs['key_binding']] = (self.num_of_channels, 1)
        elif channel_type.__name__ == 'tri_switch':
            self.channels.append(channel_type(self.num_of_channels, orientation, name, length, self))
            if 'add_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['add_key_binding']
                _key_bindings_[kwargs['add_key_binding']] = (self.num_of_channels, 1)
            if 'sub_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['sub_key_binding']
                _key_bindings_[kwargs['sub_key_binding']] = (self.num_of_channels, -1)
        elif channel_type.__name__ == 'trigger_switch':
            self.channels.append(channel_type(self.num_of_channels, orientation, name, length, self))
            if 'key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['key_binding']
                _key_bindings_[kwargs['key_binding']] = (self.num_of_channels, 1)
        elif channel_type.__name__ == 'dial':
            self.channels.append(channel_type(self.num_of_channels, args[0], orientation, name, length, self))
            if 'add_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['add_key_binding']
                _key_bindings_[kwargs['add_key_binding']] = (self.num_of_channels, 1)
            if 'sub_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['sub_key_binding']
                _key_bindings_[kwargs['sub_key_binding']] = (self.num_of_channels, -1)
        elif channel_type.__name__ == 'stick':
            self.channels.append(channel_type(self.num_of_channels, orientation, name, length, self))
            if 'add_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['add_key_binding']
                _key_bindings_[kwargs['add_key_binding']] = (self.num_of_channels, 1)
            if 'sub_key_binding' in kwargs:
                self.channels[self.num_of_channels].bind = kwargs['sub_key_binding']
                _key_bindings_[kwargs['sub_key_binding']] = (self.num_of_channels, -1)

        self.channels[self.num_of_channels].pack()
        self.channels[self.num_of_channels].place(x = x, y=y)
        self.num_of_channels += 1


    def add_value_of_channel(self, channel_no, *amount):

        if self.channels[channel_no].__name__ == 'bi_switch':
            self.channels[channel_no].add_value()
        elif self.channels[channel_no].__name__ == 'tri_switch':
            self.channels[channel_no].add_value(amount[0])
        elif self.channels[channel_no].__name__ == 'trigger_switch':
            self.channels[channel_no].add_value(amount[0])
        elif self.channels[channel_no].__name__ == 'dial':
            self.channels[channel_no].add_value(amount[0])

        self.channels[channel_no].pack()

    def handle_button_press(self, key):
        if key in _key_bindings_:
            self.channels[_key_bindings_[key][0]].add_value(_key_bindings_[key][1])
        else:
            print key, " not bound"

    def handle_button_release(self, key):
        if key in _key_bindings_:
            self.channels[_key_bindings_[key][0]].key_off()

    def generate_package(self):
        '''
        OldRange = (OldMax - OldMin)
        NewRange = (NewMax - NewMin)
        NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin

        range of sbus control = 172 - 2047
        '''
        packet = []
        for x in self.channels:
            base_max = 100.0
            base_min = -100.0
            limit_max = 2047
            limit_min = 172.0
            old_range = (base_max - (base_min))
            new_range = (limit_max - limit_min)
            new_value = (((x.value.get() - base_min) * new_range) / old_range) + limit_min
            packet.append(int(new_value))
        return packet


