#!/usr/bin/python
import sys
import re
from os import path, getenv
from lxml import etree
import fileinput

# Get the paparazzi home dir (if not set assume this folder is correct)
home_dir = getenv("PAPARAZZI_HOME", path.normpath(path.join(
        path.dirname(path.abspath(__file__)), '../../')))


# Check if we have enough arguments
if len(sys.argv) != 4:
    print("Usage: qt_project.py AIRCRAFT CONF_XML AP_SRCR_LIST_FILE")
    print("Not enough arguments!\n")
    sys.exit(1)

# Check if the first argument is a file
if not path.isfile(sys.argv[3]):
    print("Usage: qt_project.py AP_SRCR_LIST_FILE")
    print("%s is not a file!\n" % sys.argv[1])
    sys.exit(1)

# Parse the conf file for the aircraft xmls
aircraft = sys.argv[1]
#print("Parsing conf.xml file for %s..." % aircraft)
conf_tree = etree.parse(sys.argv[2])
ac_node = conf_tree.xpath('/conf/aircraft[@name="%s"]' % aircraft)
if (len(ac_node) != 1):
    print("Aircraft %s not found." % aircraft)
    sys.exit(1)
ac_node = ac_node[0]

# Parse the ap_srcs.list file
#print("Parsing the ap_srcs.list file...")
ap_srcs_list = open(sys.argv[3], "r")

# Print the target
target = re.match(r"TARGET:  ([A-Za-z_]+)", ap_srcs_list.readline()).groups(1)[0]
#print("Target is: %s" % target)

# Find all defines and generate config file
#print("Generating the config file...")
config_file = open(path.join(home_dir, target + ".config"), "w")
cflags = ap_srcs_list.readline().split(":  ")[1]
defines = re.findall(r'-D([^= ]+)[=]{0,1}([^ ]*)', cflags)
for define in defines:
    if define[1] == '':
        config_file.write("#define %s 1\n" % define[0])
    else:
        config_file.write("#define %s %s\n" % define)
config_file.close()

# Generate the includes file
#print("Generating the includes file...")
includes_file = open(path.join(home_dir, target + ".includes"), "w")
includes = re.findall(r'-I([^ ]+)', cflags)
for include in includes:
    includes_file.write(path.join("sw", "airborne", include) + "\n")
includes_file.close()

#skip the LDFLAGS
ap_srcs_list.readline()

# Generate the files file
#print("Generating the files file...")
files_file = open(path.join(home_dir, target + ".files"), "w")

# Parse the conf XML files
files_file.write(path.join("conf", ac_node.attrib['airframe']) + "\n")
files_file.write(path.join("conf", ac_node.attrib['radio']) + "\n")
files_file.write(path.join("conf", ac_node.attrib['telemetry']) + "\n")
files_file.write(path.join("conf", ac_node.attrib['flight_plan']) + "\n")
ac_settings = ac_node.attrib['settings'].split(" ")
for ac_setting in ac_settings:
    if ac_setting[0] != '[':
        files_file.write(path.join("conf", ac_setting) + "\n")
ac_settings_modules = ac_node.attrib['settings_modules'].split(" ")
for ac_settings_module in ac_settings_modules:
    if ac_settings_module[0] != '[':
        files_file.write(path.join("conf", ac_settings_module) + "\n")


#manually add msg def xml:
files_file.write("sw/ext/pprzlink/message_definitions/v1.0/messages.xml\n" )
#manually active conf xml:
files_file.write(sys.argv[2] +"\n")


# Parse the source files
srcs_all = ap_srcs_list.readline().split(":  ")[1]
srcs = re.findall(r'([^ \t\n\r]+)', srcs_all)
for src in srcs:
    files_file.write(path.join("sw", "airborne", src) + "\n")
# Parse the header files
headers_all =  ""
line = "1"
for line in ap_srcs_list:    
    line = line.strip()
    if ":" in line:
        line = line.split(":")[-1]
    if not line.endswith('h'): #remove escape \ at end of line
        line = line[:-2]
    headers_all = headers_all + line + " "

headers = re.findall(r'([^ \t\n\r]+)', headers_all)
for header in headers:
    files_file.write(path.join("sw", "airborne", header) + "\n")
files_file.close()

# Generate the project file
#print("Generating the creator file...")
creator_file = open(path.join(home_dir, target + ".creator"), "w")
creator_file.close()

# Close the ap_srcs.list
ap_srcs_list.close()

#Create a .user file, which contains user settings such as editor settings and build settings
with open(path.join(home_dir , "sw/tools/qtc.creator.user_template")) as input:
    with open(path.join(home_dir, target + ".creator.user"), 'w') as output:
        for s in input:
            if "!TARGET1!" in s:
                output.write(s.replace("!TARGET1!", aircraft + " " + target + ".compile"))
            elif "!TARGET2!" in s:
                output.write(s.replace("!TARGET2!", aircraft))
            elif "!TARGET3!" in s:
                output.write(s.replace("!TARGET3!", aircraft + " " + target + ".upload"))
            elif "!PPRZHOME!" in s:
                output.write(s.replace("!PPRZHOME!", getenv("PAPARAZZI_HOME")))
            else:
                output.write(s.replace("!PPRZSRC!", getenv("PAPARAZZI_SRC")))


print("Done generating the Qt Creator project!")
