#!/usr/bin/env python

from __future__ import print_function

import webbrowser
import os
import datetime
from fnmatch import fnmatch
import subprocess
PIPE = subprocess.PIPE

import paparazzi

import xml.etree.ElementTree

class Airframe:
    name = ""
    id = ""
    xml = ""
    flight_plan = ""
    release = ""
    def __init__(self):
        name = ""
        ac_id = ""
        xml = ""
        flight_plan = ""
        release = ""

class AirframeFile:
    name = ""
    description = ""
    firmware = []
    boards = []
    xml = ""
    includes = []
    def __init__(self):
        name = ""
        description = ""
        firmware = []
        boards = []
        xml = ""
        includes = []

class PaparazziOverview(object):

    def RepresentsInt(self, s):
        try: 
            v=int(s)
            return v
        except ValueError:
            return -1

    def maximize_text_size(self, txt):
        if len(txt) > 500:
            return txt[:500] + '...'
        else:
            return txt

    def git_behind(self, sha):
        process = subprocess.Popen(['git', 'rev-list', sha+"..HEAD", '--count'], stdout=PIPE, stderr=PIPE)
        stdoutput, stderroutput = process.communicate()
        return self.RepresentsInt(stdoutput)

    def git_ahead(self, sha):
        process = subprocess.Popen(['git', 'rev-list', "HEAD.."+sha, '--count'], stdout=PIPE, stderr=PIPE)
        stdoutput, stderroutput = process.communicate()
        return self.RepresentsInt(stdoutput)

    def get_last_commit_sha(self, file):
        process = subprocess.Popen(['git', 'log', '-n', 1, '--pretty=format:%H', '--', sha+"..HEAD"], stdout=PIPE, stderr=PIPE)
        stdoutput, stderroutput = process.communicate()
        return stdoutput


    def find_xml_files(self, folder):
        airframe_files = []
        pattern = "*.xml"
        confn = "*conf[._-]*xml"
        controlpanel = "*control_panel[._-]*xml"

        for path, subdirs, files in os.walk(os.path.join(paparazzi.conf_dir,folder)):
            for name in files:
                if fnmatch(name, confn):
                    continue
                if fnmatch(name, controlpanel):
                    continue
                if fnmatch(name, pattern):
                    filepath = os.path.join(path, name)
                    entry = os.path.relpath(filepath, paparazzi.conf_dir)
                    airframe_files.append(entry)
        airframe_files.sort()
        return airframe_files

    def find_airframe_files(self):
        return self.find_xml_files('airframes/')

    def find_flightplan_files(self):
        return self.find_xml_files('flight_plans/')

    def list_airframes_in_conf(self, conf):
        if conf is None:
            return []
        list_of_airframes = []

        afile = os.path.join(paparazzi.conf_dir, conf)
        if os.path.exists(afile):
            e = xml.etree.ElementTree.parse(afile).getroot()
            for atype in e.findall('aircraft'):
                release = ""
                if (not atype.get('release') is None) & (not atype.get('release') == ""):
                     release = atype.get('release')
                af = Airframe()
                af.name = atype.get('name')
                af.ac_id = atype.get('ac_id')
                af.xml = atype.get('airframe')
                af.flight_plan = atype.get('flight_plan')
                af.release = release
                list_of_airframes.append(af)
        return list_of_airframes

    def airframe_details(self, xmlname):
        airframe = AirframeFile()
        airframe.xml = xmlname
        airframe.firmware = []
        airframe.includes = []
        airframe.board = []
        if xml is None:
            return airframe
        afile = os.path.join(paparazzi.conf_dir, xmlname)
        if os.path.exists(afile):
            try:
                e = xml.etree.ElementTree.parse(afile).getroot()
                for atype in e.findall('firmware'):
                    if (not atype.get('name') is None) & (not atype.get('name') == "") & (not atype.get('name') in airframe.firmware):
                        airframe.firmware.append(atype.get('name'))
                    for btype in atype.findall('target'):
                        if (not btype.get('board') is None) & (not btype.get('board') == "") & (not btype.get('board') in airframe.board):
                            airframe.board.append( btype.get('board') )
                for atype in e.findall('include'):
                    if (not atype.get('href') is None) & (not atype.get('href') == ""):
                        airframe.includes.append( atype.get('href') )
                for atype in e.findall('description'):
                    airframe.description = atype.text
            except xml.etree.ElementTree.ParseError as e:
                print("Could not parse {}: {}".format(afile, e))
            
        return airframe

    def flightplan_includes(self, xmlname):
        includes = []        
        print(xmlname)
        if xml is None:
            return includes
        afile = os.path.join(paparazzi.conf_dir, xmlname)
        if os.path.exists(afile):
            e = xml.etree.ElementTree.parse(afile).getroot()
            for atype in e.findall('include'):
                if (not atype.get('procedure') is None) & (not atype.get('procedure') == ""):
                     includes.append( atype.get('procedure') )
        return includes


    # Constructor Functions
    def __init__(self, verbose):
        self.exclude_backups = 1
        self.verbose = verbose

    def run(self):
        # find all airframe XML's
        afs = self.find_airframe_files()
        fps = self.find_flightplan_files()
        #brds = self.find_boards()
        # write all confs
        with open('var/paparazzi.html','w') as f:
            f.write('<!DOCTYPE html>\n<html lang="en">\n<head>\n<title>Paparazzi</title>\n<meta charset="utf-8"/>\n<meta http-equiv="Cache-Control" content="no-cache" />\n')
            f.write('<style>\n.conf {\n\tfloat: left;\n\tmargin: 10px;\n\tpadding: 5px;\n}\n\n.uav {\n\tfloat: left;\n\tmargin: 10px;\n\tpadding: 5px;\n\twidth: 250px;\n\tborder: 1px solid black;\n\tbackground-color:#fef9e7;\n}\n</style>\n\n</head>\n')
            f.write('<body>\n')
            conf_files = paparazzi.get_list_of_conf_files()
            for conf in conf_files:
                airframes = self.list_airframes_in_conf(conf)
                f.write('<div class="conf"><h2>' + conf + '</h2>')
                for ac in airframes:
                    f.write('<div class="uav" title="'+ ac.name + ': ' + ac.xml +'"><h4>' + ac.name + ' (' + ac.ac_id + ')</h4	>')
                    sha = ac.release
                    xml = ac.xml
                    name = ac.name
                    # remove airframe xml from list
                    if xml in afs:
                        afs.remove(xml)
                    if ac.flight_plan in fps:
                        fps.remove(ac.flight_plan)
                    if (not sha is None) and (not sha == ""):
                        f.write('<p>Last flown with <a href="https://github.com/paparazzi/paparazzi/commit/' + sha + '">' + sha[:6] + '...</a></p>')
                        behind = self.git_behind(sha)
                        color = 'orange'
                        if behind < 200:
                            color = 'green'
                        if behind > 2000:
                            color = 'red'
                        if behind > 0:
                            f.write( '<p><font color="' + color + '">Is <b>' + str(behind) + '</b> commits behind</font></p>')
                        else:
                            f.write( '<p><font color="red">Is <b>not available</b> on this machine</font></p>')
                        outside = self.git_ahead(sha)
                        if outside > 0:
                            f.write( '<p><font color="red">Using <b>' + str(outside) + '</b> commits not in master</font></p>')
                    af = self.airframe_details(xml)
                    f.write('<p>' + ", ".join(af.firmware) + ' on ' + ", ".join(af.board) + ' <a href="file:////' + os.path.realpath('./conf/'+ac.xml) + '">[E]</a></p>')
                    f.write('<p><font color="gray"><i>' + self.maximize_text_size(af.description) + '</i></font></p>')
                    if self.verbose:
                        f.write('<p><a href="https://github.com/paparazzi/paparazzi/blob/master/conf/' + ac.xml + '"/>' + ac.xml + '</a></p>')
                    if self.verbose:
                        f.write('<p><a href="https://github.com/paparazzi/paparazzi/blob/master/conf/' + ac.flight_plan + '"/>' + ac.flight_plan + '</a></p>')
                        #fp_inc = self.flightplan_includes(ac.flight_plan)
                    if len(af.includes) > 0:
                        for i in af.includes:
                            inc_name = i[5:].replace('$AC_ID',ac.ac_id)
                            if inc_name in afs:
                                afs.remove(inc_name)
                        if self.verbose:
                            f.write('<p>Includes: ' + ", ".join(af.includes) + '</p>')
                    f.write('</div>\n\n')
                f.write('</div>\n')
            f.write('<hr><div class="conf"><h1>Airframe xml that are not tested by any conf:</h1>')
            for af in afs:
                f.write('<li>' + af)
            f.write('</div><div class="conf"><h1>Flight_plan xml that are not tested by any conf:</h1>')
            for af in fps:
                f.write('<li>' + af)

            f.write('</div></body>\n</html>\n')
            webbrowser.open('file://' + os.path.realpath('./var/paparazzi.html'))

if __name__ == "__main__":
    import sys
    brief = 0
    if len(sys.argv) > 1:
        brief = 1
    
    obj = PaparazziOverview(brief)
    obj.run()

