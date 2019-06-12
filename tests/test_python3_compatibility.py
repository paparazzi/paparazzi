#!/usr/bin/env python3

import os
import ast
import sys

def test_source_code_compatible(code_data):
    try:
        return ast.parse(code_data)
    except SyntaxError as exc:
        return False

paparazzi_home = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../')))

for root, dirs, files in os.walk(paparazzi_home):
    for f in files:
        file = os.path.join(root, f)
        if file.endswith('.py') and os.access(file, os.X_OK) and file.find('ext') == -1 and file.find('test_python3_compatibility.py') == -1:
            if 'pygtk' in open(file).read():
                print("%s uses pygtk which is not be Python3 compatible" % (file))
            ast_tree = test_source_code_compatible(open(file).read())
            if not ast_tree:
                print("%s might not be Python3 compatible" % (file))

input("Press Enter to continue with trying to run scripts, many will not work simply due to bad inputs...")
for root, dirs, files in os.walk(paparazzi_home):
    for f in files:
        file = os.path.join(root, f)
        if file.endswith('.py') and os.access(file, os.X_OK) and file.find('ext') == -1 and file.find('test_python3_compatibility.py') == -1:
            print("\n\nTesting " + file)
            os.system('python3 ' + file)
            
