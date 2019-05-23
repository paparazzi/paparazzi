#!/usr/bin/env python

from __future__ import print_function

import webbrowser
import os
import datetime
from fnmatch import fnmatch
import re
import numpy as np
from collections import Counter
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
    commit_processes = []
    modules = []
    def __init__(self):
        name = ""
        description = ""
        firmware = []
        boards = []
        xml = ""
        includes = []

class Module:
    """ Stores data related to single module
    Data class for storage of information related to a single module xml file.
    """
    module_airborne_dir = paparazzi.PAPARAZZI_HOME + "/sw/airborne/"
    name = ""
    last_commit = ""
    xml = ""
    usage = 0
    files = []
    missing_files = []

    def __init__(self, name, file_list):
        self.files = dict()
        self.missing_files = []
        self.name = name
        self.xml = paparazzi.modules_dir + name + ".xml"
        self.find_files(file_list)
        self.last_commit = ""
        self.calc_last_commit()

    def find_files(self, file_dict):
        """ Finds files mentioned in xml
        Finds all files mentioned in the xml and looks them up in the files in the sw folder
        """
        parser = etree.XMLParser(recover=True)
        e = etree.parse(self.xml, parser)
        for atype in e.findall('.//file'):
            file_name = atype.get('name')
            if (not file_name is None) & (not file_name == ""):
                if file_name.rfind('/') != -1:
                    file_name = file_name[file_name.rfind('/')+1:]
                self.files[file_name] = ""

        self.missing_files = list(self.files.iterkeys())
        for mod_file in self.files.iterkeys():
            if mod_file in file_dict:
                self.files[mod_file] = file_dict[mod_file]
                if mod_file in self.missing_files:
                    self.missing_files.remove(mod_file)
        return

    def calc_last_commit(self):
        """ Calculates the most recent commit from any of the files related to this module."""
        commit_processes = [self.open_commit_log_process(self.xml)]
        commit_list = []
        for key, value in self.files.iteritems():
            commit_processes.append(self.open_commit_log_process(value))
        for process in commit_processes:
            out = process.communicate()
            if out[0][:-2]:
                commit_list.append(out[0][:-2])
        commit_list = sorted(commit_list, key=lambda x: datetime.datetime.strptime(x, '%d-%m-%Y'), reverse=True)
        self.last_commit = commit_list[0]
        return

    def open_commit_log_process(self, file):
        """ Opens subprocess to retrieve last commit date"""
        process = subprocess.Popen(['git', 'log', '-1', '--date=format:%d-%m-%Y', '--format=%cd ', file], stdout=PIPE, stderr=PIPE)
        return process

    def get_comments(self):
        """
        Formats the comment field which gets printed to the html file.
        """
        output = "Last commit: " + self.last_commit + " <br />"
        if len(self.missing_files):
            output = output + "The following files could not be found: "
            for missing_file in self.missing_files:
                output = output + missing_file + "   "

        return output

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
        commit_dates = [re.sub(r"( \n)$", "", self.get_last_commit_date(paparazzi.conf_dir + elm)) for elm in lst]
        file_date_lst = sorted(zip(lst, commit_dates), key=lambda x: datetime.datetime.strptime(x[1], '%d-%m-%Y'), reverse=True)
        return file_date_lst

    def airframe_module_overview(self, selected_conf):
        """
        Creates a nested dictionary of the airframe names and the modules they use to generate an html table
        that provides an overview of module usage of the airframes of interest
        """
        # Looks at airframes in selected conf (not the currently active conf)
        airframes = self.list_airframes_in_conf(selected_conf)

        # Structure of nested dictionary: {af name: {module name: module type}}
        afs_mods = {}
        for ac in airframes:
            afs_mods[ac.name] = {'xml': [ac.xml]}
            af = self.airframe_details(ac.xml)
            for mod in af.modules:
                if mod[0] in afs_mods[ac.name]:
                    afs_mods[ac.name][mod[0]].append(mod[1])
                else:
                    afs_mods[ac.name][mod[0]] = [mod[1]]

        # Generates Counter object of all module names used by airframes in conf
        unique_mods_ctr = Counter()
        for mods in afs_mods.values():
            unique_mods_ctr.update(mods.keys())
        del unique_mods_ctr['xml']

        # Table initialization, airframe names are ordered alphabetically, module names by most used
        ac_mod_table = np.zeros((len(afs_mods.keys()) + 1, len(unique_mods_ctr.keys()) + 1), dtype=object)
        ac_mod_table[0, 0] = "Name \\ Modules"
        ac_mod_table[1:, 0] = sorted(afs_mods.keys(), key=lambda s: s.lower())
        ac_mod_table[0, 1:] = [i for i, _ in unique_mods_ctr.most_common()]

        for i in range(1, len(ac_mod_table[:, 0])):
            for j in range(1, len(ac_mod_table[0, :])):
                ac_name = ac_mod_table[i, 0]
                module_name = ac_mod_table[0, j]
                if ac_mod_table[0, j] not in afs_mods[ac_name]:
                    ac_mod_table[i, j] = "\\"
                else:
                    ac_mod_table[i, j] = ""
                    for module_type in afs_mods[ac_name][module_name]:
                        if module_type == "":
                            ac_mod_table[i, j] = "default"
                        else:
                            ac_mod_table[i, j] += module_type + "\n"

        with open('var/airframe_module_overview.html', 'w') as f:
            f.write('<!DOCTYPE html>\n<html lang="en">\n<head>\n<title>Module usage overview</title>\n \
                    <meta charset="utf-8"/>\n<meta http-equiv="Cache-Control" content="no-cache" />\n')
            f.write('<style>\ntable, th, td {\n\tborder: 1px solid black;\n\tborder-collapse: collapse;\n\t \
                    padding:5px;\n}\ntr:nth-child(even) {\n\tbackground: #CCC;\n}\ntr:nth-child(odd) {\n\t \
                    background: #FFF;\n}\nth {\n\ttext-align:left;\n}\n</style>\n\n</head>\n')
            f.write('<body>\n')

            f.write('<table><tr>\n\t')
            for column_header in ac_mod_table[0, :]:
                f.write('<th>{}</th>\n\t'.format(column_header))
            f.write('</tr>\n')
            for row in ac_mod_table[1:, :]:
                f.write('<tr>\n\t')
                for element in row:
                    f.write('<td>{}</td>\n\t'.format(element))
                f.write('</tr>\n')
            f.write('</table>\n</body>\n</html>\n')

            webbrowser.open('file://' + os.path.realpath('./var/airframe_module_overview.html'))

    # Constructor Functions
    def __init__(self, verbose):
        self.exclude_backups = 1
        self.verbose = verbose

    def run(self):
        # find all airframe, flightplan and module  XML's
        afs = self.find_airframe_files()
        fps = self.find_flightplan_files()
        bs  = self.find_board_files()

        # Generation of list of all files in the sw directory, for checking with the modules.
        module_sw_dir = paparazzi.PAPARAZZI_HOME + "/sw/"
        file_dict = dict()
        for root, dirs, dir_files in os.walk(module_sw_dir):
            for dir_file in dir_files:
                file_dict[dir_file] = os.path.join(root, dir_file)

        mods = {name: Module(name, file_dict) for name in paparazzi.get_list_of_modules()}

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
        for ac in afs:
            af = self.airframe_details(ac)
            for af_mod in af.modules:
                if af_mod[1] == "":
                    mod_str = af_mod[0]
                else:
                    mod_str = af_mod[0] + "_" + af_mod[1]
                try:
                    mods[mod_str].usage = mods[mod_str].usage + 1
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
            f.write('<table><tr><th> Filename </th><th> Number of airframes used in </th><th> Comments </th></tr>')
            for name, mod in mods.iteritems():
                f.write('<tr><td><li>' + mod.name + '</td><td>' + str(mod.usage) + '</td><td>' + mod.get_comments() + '</td></tr>')
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

