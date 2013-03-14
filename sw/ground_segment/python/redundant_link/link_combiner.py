#!/usr/bin/env python

# This file is part of paparazzi.

# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.

# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

"""This program listens to ivy messages from the link agent (see link.ml) 
when the link's -name arguement is set. It combines messages received from
any number of link agents and sends ivy messages to the Server and other 
agents."""

#To-do:
# 1. Implement timestamp on buffer
# 2. Record and print statistics. Necessary to test functionality. 
# 3. Move the regex into the message class to make things more modular
# 4. Implement command line options
# 5. Read the link status messages, create new link status data, and send it out over the ivy bus

import logging
import sys
import os
from ivy.std_api import *

#Options (will be implemented as command line options later):
BUFFER_SIZE = 10        #The number of elements messages to be stored in the circular buffer for each link. 

class Circular_Buffer:
    def __init__(self, size):
        self.buffer = [""]*size
        self.index = 0
        self.size = size

    def add(self, contents):
        self.buffer[self.index] = contents
        self.incr()

    def contains(self, contents):
        counter = self.index
        while 1:
            if self.buffer[counter] == contents:
                return 1
            else:
                if counter <= 0:
                    counter = self.size-1    #Moving the counter in the reverse direction of the index in order to test the most recent contents first (since they're most likely to match)
                else:
                    counter -= 1
                if counter == self.index:
                    return 0

    def incr(self):
        if self.index >= (self.size - 1):
            self.index = 0
        else:
            self.index += 1

    def remove(self, contents):
        counter = self.index
        while 1:
            if self.buffer[counter] == contents:
                self.buffer[counter] = ""
            else:
                if counter <= 0:
                    counter = self.size-1
                else:
                    counter -= 1
                if counter == self.index:
                    return

    def displayContents(self):
        for counter in xrange(0,self.size):
            if self.index != counter:
                print("   %s" %self.buffer[counter])
            else:
                print("-> %s" %self.buffer[counter])

class Message:
    def __init__(self, sender, link_name, raw_message):

        self.link_name = link_name
        self.raw_message = " ".join(raw_message.split(";"))

    def linkName(self):
        return self.link_name

    def message(self):
        return self.raw_message


class Link:
    def __init__(self, name, buffer_size=10, verbose=1):
        self.buffer = Circular_Buffer(buffer_size)
        self.name = name
        self.verbose = verbose
        pass

    def checkBuffer(self,message):
        return self.buffer.contains(message.message())

    def addToBuffer(self,message):
        self.buffer.add(message.message())
        if self.verbose:
            print("%s Buffer:" % self.name)
            self.buffer.displayContents();

    def removeFromBuffer(self,message):
        self.buffer.remove(message.message())

class Link_Combiner:

    def __init__(self):

        self.links = {}

        self.initIvy()

    def initIvy(self):
        # initialising the bus
        IvyInit("Link_Combiner",    # application name for Ivy
                "READY",            # ready message
                0,                  # main loop is local (ie. using IvyMainloop)
                lambda x,y: y,      # handler called on connection/deconnection
                lambda x,y: y       # handler called when a diemessage is received
                )

        # starting the bus
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyStart("")
        IvyBindMsg(self.onIvyMessage, "^([^ ]+) TELEMETRY_MESSAGE ([^ ]+) ([^ ]+) ([^ ]+)$")

    def onIvyMessage(self, agent, *larg):

        message = Message(larg[1], larg[2], larg[3])

        if message.linkName() not in self.links: #Adding a new link
            self.links[message.linkName()] = Link(message.linkName(), BUFFER_SIZE)
            print("Link Combiner Detected a New Link: %s" %message.linkName())

        #Processing messages from an already added link
        sent = self.sendMessage(message)
        self.bufferMessage(message)

    def sendMessage(self, message):

        in_buffer = self.checkBuffers(message)
        if not in_buffer:
            IvySendMsg(message.message())
            return True
        else:
            return False

    def checkBuffers(self, message):
        #The returned value is the best guess at whether the message is a duplicate (True), or not (False).
        #If the message is already in this link's buffer, then taking it as not a duplicate. So returning False. But also, removing it from all buffers. So that when they receive it, they don't do the same.
        #If the message is not in this link's buffer, then checking all other buffers and only if it's not in any of them, counting the message as not a duplicate.

        match = self.links[message.linkName()].checkBuffer(message) 
        if match:   #Removing the message from all buffers
            for link_name in self.links:
                self.links[link_name].removeFromBuffer(message)
            return False
        else:       #Checking all other links' buffers
            for link_name in self.links:
                if link_name == message.linkName():
                    continue
                else:
                    match = self.links[link_name].checkBuffer(message)
                    if match:
                        return True

            return False



        if match_count == 0:
            return False
        elif match_count == length(self.links):
            for link_name in self.links:
                self.links[link_name].removeFromBuffer(message)

    def bufferMessage(self, message):
        self.links[message.linkName()].addToBuffer(message)            


def main():
    link = Link_Combiner()


if __name__ == '__main__':
    main()