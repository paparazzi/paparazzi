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
import serial
import struct
from threading import Thread, Lock
import sys


import time

import Tkinter as tk

from lib import controller
from lib import channel
from lib import sbus

ser_rc = serial.Serial(None)

options = []
msg = []
delay = 0

latency_packet_behind = 0
latency_packet_sent = 0


locker = Lock()


def new_controller():
    pass

root = tk.Tk()

menubar = tk.Menu(root)

file_menu = tk.Menu(menubar, tearoff=0)
file_menu.add_command(label="Exit", command=root.quit())

new_menu = tk.Menu(file_menu, tearoff=1)
new_menu.add_command(label="Controller", command=new_controller)

menubar.add_cascade(label="File", menu=file_menu)
menubar.add_cascade
root.config(menu=menubar)

w = controller.Controller(root)
w.add_channel(channel.stick, name="Throttle", x=110, y=500, add_key_binding = 'w', sub_key_binding = 's')
w.add_channel(channel.stick, name="Roll", orientation = tk.HORIZONTAL, x = 100, y = 440, add_key_binding = 'a', sub_key_binding = 'd')
w.add_channel(channel.stick, name="Pitch", x = 500, y = 500, add_key_binding = 'i', sub_key_binding = 'k')
w.add_channel(channel.stick, name="Yaw", orientation=tk.HORIZONTAL, x = 490, y = 440, add_key_binding = 'j', sub_key_binding = 'l')
w.add_channel(channel.tri_switch, name="SA", length = 50, x = 100, y= 200, add_key_binding = "r", sub_key_binding = "f")
w.add_channel(channel.tri_switch, name="SB", length = 50, x = 180, y = 200, add_key_binding = "v", sub_key_binding = "c")
w.add_channel(channel.dial, 230, orientation = tk.HORIZONTAL, length = 150, name="S1", x = 100, y = 250, add_key_binding = "q", sub_key_binding = "e")
w.add_channel(channel.tri_switch, name="SC", length = 50, x = 440, y = 200, add_key_binding = "y", sub_key_binding = "h")
w.add_channel(channel.tri_switch, name="SD", length = 50, x = 520, y = 200, add_key_binding = "m", sub_key_binding = "n")
w.add_channel(channel.dial, 230, orientation = tk.HORIZONTAL, name="S2", length = 150, x = 440, y = 250, add_key_binding = "u", sub_key_binding = "o")
w.add_channel(channel.bi_switch, name = "SF", length = 50, x = 100, y = 0, key_binding = '1')
w.add_channel(channel.tri_switch, name="SE", length = 50, x = 100, y = 50, add_key_binding = '2', sub_key_binding = '3') 
w.add_channel(channel.trigger_switch, name="SH", length = 50, x = 520, y = 0, key_binding = 'p')
w.add_channel(channel.tri_switch, name="SG", length = 50, x = 520, y = 50, add_key_binding = '[', sub_key_binding = ']')
w.add_channel(channel.dial, 180, name = "Slide_L", length = 300, x = 0, y = 0, add_key_binding = '4', sub_key_binding = '5')
w.add_channel(channel.dial, 180, name = "Slide_R", length = 300, x = 600, y = 0, add_key_binding = '6', sub_key_binding = '7')


def keypress(event):
    x = event.char
    w.handle_button_press(x)
        
def keyrelease(event):
    x = event.char
    w.handle_button_release(x)

main_frame = tk.Frame(root)
w.bind("<Key>", keypress)
w.bind("<KeyRelease>", keyrelease)

def sbus_loop():

    while True:
        locker.acquire()
        encoded = sbus.sbus_encode(w.generate_package())
        locker.release()
        msg.append(encoded)
        write_sbus()
        time.sleep(.05)

def write_sbus():
    global latency_packet_sent
    if 'serial' in options:
        for x in msg[latency_packet_sent]:
            ser_rc.write(struct.pack('<B', x))
        if not 'ivy' in options:
            del msg[0]
    if 'ivy' in options and latency_packet_sent-latency_packet_behind >=0:
        print "IVY:    {}".format(msg[0][1:])
        i_interface.send_ivy_msg('ground_dl', 'SBUS', msg[0][1:])
        del msg[0]
    if latency_packet_behind-latency_packet_sent <=0:
        latency_packet_sent = latency_packet_behind - 1
    latency_packet_sent += 1

if __name__ == '__main__':
    w.pack(side="top", fill="both", expand=True)
    w.focus_set()
    main_frame.pack()
    
    t = Thread(target=sbus_loop)
    t.start()
    print sys.argv
    if not ('-i' in sys.argv  or '-p' in sys.argv):
        print "ERROR: no input found"
        sys.exit()
    for x in range(1, len(sys.argv)):
        if sys.argv[x] == '-i':
            import os
            #Add the lib folder to the system path.
            sys.path.append(os.getcwd() + "/../../lib/")
            from sim_vars import i_interface
            options.append('ivy')
        
        elif sys.argv[x] == '-l':
            x += 1
            delay = float(sys.argv[x])
            latency_packet_behind = int(delay/.05)
            
        elif sys.argv[x] == '-p':
            options.append('serial')
            x += 1
            port = str(sys.argv[x])
            baud = 100000
            
            ser_rc.port = port
            ser_rc.baudrate = baud
            ser_rc.bytesize = serial.EIGHTBITS
            ser_rc.timeout = 1
            ser_rc.parity = serial.PARITY_EVEN
            ser_rc.stopbits = serial.STOPBITS_TWO
            ser_rc.open()
    
    root.mainloop()
