#!/usr/bin/env python

from __future__ import absolute_import, print_function

import os

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))

default_messages_file = '%s/conf/messages.xml' % PPRZ_HOME

message_dictionary = {}
message_dictionary_types = {}
message_dictionary_id_name = {}
message_dictionary_name_id = {}


class MessagesNotFound(Exception):
    def __init__(self, filename):
        self.filename = filename

    def __str__(self):
        return "messages file " + repr(self.filename) + " not found"


def parse_messages(messages_file=''):
    if not messages_file:
        messages_file = default_messages_file
    if not os.path.isfile(messages_file):
        raise MessagesNotFound(messages_file)
    from lxml import etree
    tree = etree.parse(messages_file)
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
            if message_id[0:2] == "0x":
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
                message_dictionary[class_name][message_name].append(the_field.attrib['name'])
                message_dictionary_types[class_name][message_id].append(the_field.attrib['type'])


def get_msgs(msg_class):
    if not message_dictionary:
        parse_messages()
    if msg_class in message_dictionary:
        return message_dictionary[msg_class]
    else:
        print("Error: msg_class %s not found." % msg_class)
    return []


def get_msg_fields(msg_class, msg_name):
    if not message_dictionary:
        parse_messages()
    if msg_class in message_dictionary:
        if msg_name in message_dictionary[msg_class]:
            return message_dictionary[msg_class][msg_name]
        else:
            print("Error: msg_name %s not found in msg_class %s." % (msg_name, msg_class))
    else:
        print("Error: msg_class %s not found." % msg_class)
    return []


def get_msg_id(msg_class, msg_name):
    if not message_dictionary:
        parse_messages()
    try:
        return message_dictionary_name_id[msg_class][msg_name]
    except KeyError:
        print("Error: msg_name %s not found in msg_class %s." % (msg_name, msg_class))
        return 0


def get_msg_fieldtypes(msg_class, msg_id):
    if not message_dictionary:
        parse_messages()
    if msg_class in message_dictionary_types:
        if msg_id in message_dictionary_types[msg_class]:
            return message_dictionary_types[msg_class][msg_id]
        else:
            print("Error: message with ID %d not found in msg_class %s." % (msg_id, msg_class))
    else:
        print("Error: msg_class %s not found." % msg_class)
    return []


def test():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="path to messages.xml file")
    parser.add_argument("-l", "--list", help="list parsed messages", action="store_true", dest="list_messages")
    parser.add_argument("-c", "--class", help="message class", dest="msg_class", default="telemetry")
    args = parser.parse_args()
    parse_messages(args.file)
    if args.list_messages:
        print("Listing %i messages in '%s' msg_class" % (len(message_dictionary[args.msg_class]), args.msg_class))
        for msg in message_dictionary[args.msg_class]:
            print(msg)

if __name__ == '__main__':
    test()
