#!/usr/bin/env python

from __future__ import absolute_import, print_function

import os
import sys
import getopt

messages_path = '%s/conf/messages.xml' % os.getenv("PAPARAZZI_HOME")

message_dictionary = {}
message_dictionary_types = {}
message_dictionary_id_name = {}
message_dictionary_name_id = {}

def Usage(scmd):
    lpathitem = scmd.split('/')
    fmt = '''Usage: %s [-h | --help] [-f FILE | --file=FILE]
where
\t-h | --help print this message
\t-f FILE | --file=FILE where FILE is path to messages.xml
'''
    print(fmt % lpathitem[-1])

def GetOptions():
    try:
        optlist, left_args = getopt.getopt(sys.argv[1:],'hf:', ['help','file='])
    except getopt.GetoptError:
        # print help information and exit:
        Usage(sys.argv[0])
        sys.exit(2)
    for o, a in optlist:
        if o in ("-h", "--help"):
            Usage(sys.argv[0])
            sys.exit()
        elif o in ("-f", "--file"):
            messages_path = a


def ParseMessages():
    from lxml import etree
    tree = etree.parse( messages_path)
    for the_class in tree.xpath("//msg_class[@name]"):
        class_name = the_class.attrib['name']
        if class_name not in message_dictionary:
            message_dictionary_id_name[class_name] = {}
            message_dictionary_name_id[class_name] = {}
            message_dictionary[class_name] = {}
            message_dictionary_types[class_name] = {}
        for the_message in the_class.xpath("message[@name]"):
            message_name = the_message.attrib['name']
            if 'id' in the_message.attrib:
                message_id = the_message.attrib['id']
            else:
                message_id = the_message.attrib['ID']
            if (message_id[0:2] == "0x"):
                message_id = int(message_id, 16)
            else:
                message_id = int(message_id)

            message_dictionary_id_name[class_name][message_id] = message_name
            message_dictionary_name_id[class_name][message_name] = message_id

            # insert this message into our dictionary as a list with room for the fields
            message_dictionary[class_name][message_name] = []
            message_dictionary_types[class_name][message_id] = []

            for the_field in the_message.xpath('field[@name]'):
                # for now, just save the field names -- in the future maybe expand this to save a struct?
                message_dictionary[class_name][message_name].append( the_field.attrib['name'])
                message_dictionary_types[class_name][message_id].append( the_field.attrib['type'])

def test():
    GetOptions()
    ParseMessages()

if __name__ == '__main__':
    test()
