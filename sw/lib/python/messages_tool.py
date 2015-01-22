from __future__ import absolute_import, print_function

import messages_xml_map
from ivy.std_api import *
import logging
import time
import os
import re


class Message:
    def __init__(self, class_name, name):
        messages_xml_map.parse_messages()
        self.field_value = []
        self.field_names = messages_xml_map.message_dictionary[class_name][name]
        self.field_controls = []
        self.index = None
        self.last_seen = time.clock()
        self.name = name


class Aircraft:
    def __init__(self, ac_id):
        self.ac_id = ac_id
        self.messages = {}
        self.messages_book = None
