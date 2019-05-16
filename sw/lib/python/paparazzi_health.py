#!/usr/bin/env python

from __future__ import print_function

import webbrowser
import os
import datetime
from fnmatch import fnmatch
import re
import subprocess
PIPE = subprocess.PIPE

import paparazzi

import xml.etree.ElementTree
from lxml import etree

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
    last_commit = ""
    modules = []
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

    def get_last_commit_date(self, file):
        process = subprocess.Popen(['git', 'log', '-1', '--date=format:%d-%m-%Y', '--format=%cd ', file], bufsize=-1, stdout=PIPE, stderr=PIPE)
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

    def find_makefiles(self, folder):
        board_files = []
        pattern = "*.makefile"

        for path, subdirs, files in os.walk(os.path.join(paparazzi.conf_dir,folder)):
            for name in files:
                if fnmatch(name, pattern):
                    filepath = os.path.join(path, name)
                    entry = os.path.relpath(filepath, paparazzi.conf_dir)
                    board_files.append(entry)
        board_files.sort()
        return board_files

    def find_airframe_files(self):
        return self.find_xml_files('airframes/')

    def find_flightplan_files(self):
        return self.find_xml_files('flight_plans/')

    def find_board_files(self):
        return self.find_makefiles('boards/')

    def list_includes_in_flightplan(self, fp):
        afile = os.path.join(paparazzi.conf_dir, fp)

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
        airframe.boards = []
        airframe.modules = []
        if xml is None:
            return airframe
        afile = os.path.join(paparazzi.conf_dir, xmlname)
        if os.path.exists(afile):
            try:
                e = xml.etree.ElementTree.parse(afile).getroot()
                for atype in e.findall('firmware'):
                    for ctype in atype.findall('module'):
                        module_name_type = self.get_module_name_type(ctype)
                        airframe.modules.append(module_name_type)
                    if (not atype.get('name') is None) & (not atype.get('name') == "") & (not atype.get('name') in airframe.firmware):
                        airframe.firmware.append(atype.get('name'))
                    for btype in atype.findall('target'):
                        if (not btype.get('board') is None) & (not btype.get('board') == "") & (not btype.get('board') in airframe.boards):
                            airframe.boards.append( btype.get('board') )
                        for ctype in btype.findall('module'):
                            module_name_type = self.get_module_name_type(ctype)
                            airframe.modules.append(module_name_type)
                for atype in e.findall('include'):
                    if (not atype.get('href') is None) & (not atype.get('href') == ""):
                        airframe.includes.append( atype.get('href') )
                for atype in e.findall('description'):
                    airframe.description = atype.text

            except xml.etree.ElementTree.ParseError as e:
                print("Could not parse {}: {}".format(afile, e))
            
        return airframe

    def get_module_name_type(self, ctype):
        module_name = re.sub(r'(\.xml)$', "", ctype.get('name'))
        if ctype.get('type') is not None:
            module_type = re.sub(r'(\.xml)$', "", ctype.get('type'))
        else:
            module_type = ""
        module = (module_name, module_type)
        return module

    def flightplan_includes(self, xmlname):
        includes = []        
        if xml is None:
            return includes
        afile = os.path.join(paparazzi.conf_dir, xmlname)
        parser = etree.XMLParser(recover=True)
        if os.path.exists(afile):
            try:
                e = etree.parse(afile, parser).getroot()
                for atype in e.findall('include'):
                    if (not atype.get('procedure') is None) & (not atype.get('procedure') == ""):
                        if atype.get('procedure')[0:7] == "include_":
                            includes.append(atype.get('procedure')[8:])  # .lstrip('include_') )
                        else:
                            includes.append(atype.get('procedure'))
                for btype in e.findall('includes'):
                    for atype in btype.findall('include'):
                        if (not atype.get('procedure') is None) & (not atype.get('procedure') == ""):
                            if atype.get('procedure')[0:7] == "include_":
                                includes.append(atype.get('procedure')[8:])#.lstrip('include_') )
                            else:
                                includes.append(atype.get('procedure'))
            except etree.ParseError as e:
                print("Could not parse {}: {}".format(afile, e))
        return includes

    def generate_sorted_list(self, lst):
        # [:-2] needed to remove \n from output of get_last_commit_date
        commit_dates = [self.get_last_commit_date(paparazzi.conf_dir + elm)[:-2] for elm in lst]
        file_date_lst = sorted(zip(lst, commit_dates), key=lambda x: datetime.datetime.strptime(x[1], '%d-%m-%Y'), reverse=True)
        return file_date_lst


    # Constructor Functions
    def __init__(self, verbose):
        self.exclude_backups = 1
        self.verbose = verbose

    def run(self):
        # find all airframe, flightplan and module  XML's
        afs = self.find_airframe_files()
        fps = self.find_flightplan_files()
        bs  = self.find_board_files()
        mods = paparazzi.get_list_of_modules()

        #check if flight plans are included in other flight plans
        fps_usage = dict()
        for fp in fps:
            fps_usage[os.path.split(fp)[1]] = ""
        for fp in fps:
            includes = self.flightplan_includes(fp)
            for include in includes:
                try:
                    fps_usage[include] = fps_usage[include] + str(os.path.split(fp)[1]) + ". "
                except KeyError:
                    print(include + " in " + fp + ": Does not seem to have an xml file associated with it")

        # Create usage dictionary for modules
        mods_usage = dict.fromkeys(mods, 0)
        for ac in afs:
            af = self.airframe_details(ac)
            for af_mod in af.modules:
                if af_mod[1] == "":
                    mod_str = af_mod[0]
                else:
                    mod_str = af_mod[0] + "_" + af_mod[1]
                try:
                    mods_usage[mod_str] = mods_usage[mod_str] + 1
                except KeyError:
                    print(mod_str + " in " + af.xml + ": Does not seem to have an xml file associated with it")

        #brds = self.find_boards()
        # write all confs
        with open('var/paparazzi.html','w') as f:
            f.write('<!DOCTYPE html>\n<html lang="en">\n<head>\n<title>Paparazzi</title>\n<meta charset="utf-8"/>\n<meta http-equiv="Cache-Control" content="no-cache" />\n')
            f.write('<style>\n.conf {\n\tfloat: left;\n\tmargin: 10px;\n\tpadding: 5px;\n}\n\n.uav {\n\tfloat: left;\n\tmargin: 10px;\n\tpadding: 5px;\n\twidth: 250px;\n\tborder: 1px solid black;\n\tbackground-color:#fef9e7;\n}\nth {\n\ttext-align:left;\n}\n</style>\n\n</head>\n')
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
                    for board in af.boards:
                        pattern = 'boards/' + board + '.makefile'
                        if pattern in bs:
                            bs.remove(pattern)
                    f.write('<p>' + ", ".join(af.firmware) + ' on ' + ", ".join(af.boards) + ' <a href="file:////' + os.path.realpath('./conf/'+ac.xml) + '">[E]</a></p>')
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

            # Generate table with airframe files that are not in any config
            f.write('</div><div class="conf"><h1>Airframe xml that are not tested by any conf:</h1>')
            f.write('<table><tr><th> Filename </th><th> Last commit date </th></tr>')
            afs_sorted = self.generate_sorted_list(afs)
            for af in afs_sorted:
                f.write('<tr><td><li>' + af[0] + '</td><td>' + af[1] + '</td></tr>')
            f.write('</table>')

            # Generate table with flightplan files that are not in any config
            f.write('</div><div class="conf"><h1>Flight_plan xml that are not tested by any conf:</h1>')
            f.write('<table><tr><th> Filename </th><th> Last commit date </th><th> Included in: </th></tr>')
            fps_sorted = self.generate_sorted_list(fps)
            for af in fps_sorted:
                name = os.path.split(af[0])[1]
                f.write('<tr><td><li>' + af[0] + '</td><td>' + af[1] + '</td><td>' + fps_usage[name] + '</td></tr>')
            f.write('</table>')

            # Generate table with board files that are not in any config
            f.write('</div><div class="conf"><h1>Board makefiles that are not tested by any conf:</h1>')
            f.write('<table><tr><th> Filename </th><th> Last commit date </th></tr>')
            bs_sorted = self.generate_sorted_list(bs)
            for b in bs_sorted:
                f.write('<tr><td><li>' + b[0] + '</td><td>' + b[1] + '</td></tr>')
            f.write('</table>')

            # Generate table with all modules and module usage
            f.write('</div><div class="conf"><h1>Module xml:</h1>')
            f.write('<table><tr><th> Filename </th><th> Number of airframes used in </th></tr>')
            for mod in mods:
                f.write('<tr><td><li>' + mod + '</td><td>' + str(mods_usage[mod]) + '</td></tr>')
            f.write('</table>')

            f.write('</div></body>\n</html>\n')
            webbrowser.open('file://' + os.path.realpath('./var/paparazzi.html'))


if __name__ == "__main__":
    import sys
    brief = 0
    if len(sys.argv) > 1:
        brief = 1
    
    obj = PaparazziOverview(brief)
    obj.run()

