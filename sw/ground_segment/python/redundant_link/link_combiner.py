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

from __future__ import print_function
import logging
import sys
import os
import argparse
from ivy.std_api import *

PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
sys.path.append(PPRZ_HOME + "/sw/lib/python")
from pprz_msg import messages_xml_map

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
                print("   %s" %self.buffer[counter], file=sys.stderr)
            else:
                print("-> %s" %self.buffer[counter], file=sys.stderr)

class Message:
    def __init__(self, sender, link_name, raw_message):

        self.link_name = link_name
        self.raw_sender = sender
        self.raw_message = " ".join(raw_message.split(";"))

        if self.name() not in messages_xml_map.message_dictionary['telemetry']:
            raise(Exception("Error in link_combiner: unknown message name: %s" %self.name()))

        value_names = messages_xml_map.message_dictionary['telemetry'][self.name()]
        values = self.raw_message.split(" ")[1:-1]
        self.raw_values = {}
        map(lambda k, v: self.raw_values.update({k: v}), value_names, values)
        # self.raw_values = dict(zip(value_names, values))  #This doesn't work for some reason. It gives weird errors, seemingly related to the ivy bus

    def linkName(self):
        return self.link_name

    def message(self):
        return self.raw_message

    def sender(self):
        return self.raw_sender

    def name(self):
        return self.raw_message.split(" ")[1]

    def values(self):
        return self.raw_values



class Link:
    def __init__(self, name, ac_id, buffer_size=10, verbose=0):
        self.buffer = Circular_Buffer(buffer_size)
        self.name = name
        self.verbose = verbose
        self.acs = [ac_id]  #Storing a list of the aircrafts that use this link. Usually it's just one.

    def checkBuffer(self,message):
        return self.buffer.contains(message.message())

    def addToBuffer(self,message):
        self.buffer.add(message.message())
        if self.verbose:
            print("%s Buffer:" % self.name, file=sys.stderr)
            self.buffer.displayContents();

    def removeFromBuffer(self,message):
        self.buffer.remove(message.message())

    def acAc(self, ac_id):
        self.acs = self.acs + [ac_id]

    def aircrafts(self):
        return self.acs



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
            self.links[message.linkName()] = Link(message.linkName(), message.sender(), BUFFER_SIZE)
            # print("NEW LINK DETECTED: %s" %message.linkName(), file=sys.stderr)

        #Processing messages from an already added link
        link = self.links[message.linkName()]
        self.sendMessage(message)
        self.bufferMessage(message)
        if message.sender() not in link.aircrafts():
            link.addAc(message.sender())


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
    messages_xml_map.parse_messages()


    #Command line options
    parser = argparse.ArgumentParser(description="Link_Combiner listens to the ivy messages received from multiple Link agents (set each of their -id options to a unique number), and sends a combined stream of messages to the other agents.")
    parser.add_argument("-b", "-buffer_size", "--buffer_size", help="The number of elements messages to be stored in the circular buffer for each link", default=10)
    args = parser.parse_args()

    global BUFFER_SIZE
    BUFFER_SIZE = int(args.buffer_size)            #The number of elements messages to be stored in the circular buffer for each link.

    link_combiner = Link_Combiner()


if __name__ == '__main__':
    main()

