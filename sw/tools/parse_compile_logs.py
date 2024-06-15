#!/usr/bin/env python

# Parse compiler logs and generate a summary of the errors and warnings
# Load log file from var/compiler.log

import os
import sys
import re

def parse_log(log_file):
    with open(log_file, 'r') as f:
        lines = f.readlines()

    errors = []
    conf = ''
    airframe = ''
    module = ''

    for line in lines:
        if 'Testing all aircrafts in conf: ' in line:
            conf = line.split('Testing all aircrafts in conf: ')[1].strip()
            #print(conf)
        elif 'tests/modules/test_modules.py' in line:
            conf = 'modules'
        elif 'compiling AIRCRAFT: [' in line:
            airframe = line.split('compiling AIRCRAFT: [')[1].strip().strip(']').replace('] TARGET: [', ' --- ' )
            #print('\t-',airframe)
        if conf == 'modules':
            if ('ok ' in line) and ('_0' in line):
                airframe = line.strip()

        if conf and airframe:
            if 'error:' in line:
                errors.append((conf, airframe, line.strip()))
            if 'warning:' in line:
                errors.append((conf, airframe, line.strip()))

    return errors

def print_errors(errors):
    last_conf = ''
    last_airframe = ''
    if len(errors) == 0:
        print(' - everything looks OK!')
        return
    for conf, airframe, error in errors:
        if conf != last_conf:
            print('')
            print(conf+':')
            print('-' * len(conf))
            last_conf = conf
            last_airframe = ''
        
        if airframe != last_airframe:
            print(' - ',airframe)
            last_airframe = airframe

        print('\t- [ ] ```',error, '```')


if __name__ == '__main__':
    print('=========================')
    print('||      SUMMARY        ||')
    print('=========================')
    log_file = './var/compile.log'
    errors = parse_log(log_file)
    print_errors(errors)


