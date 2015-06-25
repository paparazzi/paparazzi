#!/usr/bin/env python
from __future__ import print_function

from optparse import OptionParser
import xml.etree.ElementTree as ET

import os
import sys
import glob
import re
import copy
import string


def dox_new_page(name, title):
    return "/** @page " + name + " " + title + "\n\n"


def dox_section(name, title):
    return "@section " + name + " " + title + "\n"


def dox_list_file(f):
    s = "- @ref " + f
    #s = "- " + f
    return s + "\n"


def get_module_dir(module):
    return module.get("dir", module.get("name")).strip()


def modules_category_list(category, modules):
    s = "@subsection modules_category_" + category.lower() + " " + category.title() + " modules\n\n"
    for (fname, m) in sorted(modules.items()):
        page_name = "module__" + fname[:-4].lower()
        s += "- " + fname + " @subpage " + page_name + "\n"
    return s + "\n\n"


def modules_overview_page(modules_dict):
    s = dox_new_page("onboard_modules", "Onboard Modules")
    s += "@tableofcontents\n"
    s += "The modules allow to add new code in a flexible way with initialisation, periodic and event functions without modifying the main AP loop.\n"
    s += "Also see http://wiki.paparazziuav.org/wiki/Modules\n\n"
    s += "@section modules_list List of available modules\n\n"

    # dict with dirs and modules under that dir
    dirs = {}
    for (mfile, m) in modules_dict.items():
        mdir = get_module_dir(m)
        if mdir not in dirs:
            #print("New module dir: " + mdir)
            dirs[mdir] = {mfile: m}
        else:
            dirs[mdir][mfile] = m
        #print(" {0}: adding file {1}".format(mdir, mfile))

    misc = {}
    for d in sorted(dirs.keys()):
        # print extra subsection if dir contains multiple modules
        if len(dirs[d]) > 1:
            s += modules_category_list(d, dirs[d])
        else:
            # if there is only one module in the dir, add module to misc dict
            (mfile, m) = dirs[d].popitem()
            misc[mfile] = m
    s += modules_category_list('misc', misc)

    # end overview page
    s += "\n */\n\n"
    return s


def module_page(filename, module):
    (brief, details) = get_module_description(module)
    keyword = filename[:-4].lower()
    page_name = "module__" + keyword
    s = dox_new_page(page_name, brief)
    s += "Module XML file: @c " + filename + "\n\n"
    s += details + "\n"
    s += get_xml_example(filename, module)
    s += module_configuration(module)
    s += module_functions(module)
    s += "@section files Files\n\n"
    s += headers_list(module)
    s += sources_list(module)
    s += "\n@subsection module_xml__{0} Raw {1} file:\n@include {1}\n".format(keyword, filename)
    s += "\n */\n\n"
    return s


def get_xml_example(filename, module):
    opts = module.findall(".doc/define") + module.findall(".doc/configure")
    s = "\n@section module_load_example__{0} Example for airframe file\n".format(filename[:-4].lower())
    if opts:
        s += "This example contains all possible configuration options, not all of them are mandatory!\n"
    s += "@code{.xml}\n"
    s += "<modules>\n"
    if opts:
        s += '  <load name="{0}">\n'.format(filename)
        for o in opts:
            e = copy.deepcopy(o)
            if 'description' in e.attrib:
                del e.attrib['description']
            s += "    " + string.strip(ET.tostring(e)) + "\n"
        s += "  </load>\n"
    else:
        s += '  <load name="{0}"/>\n'.format(filename)
    s += "</modules>\n"
    s += "@endcode\n"
    return s


def get_doc_config_option(module, type):
    s = ""
    try:
        confs = module.findall("./doc/" + type)
        if confs:
            s += "@subsection {0} {1} Options\n\n".format(type, type.title())
            for c in confs:
                s += "- @b name: @c {0} @b value: <em>{1}</em>".format(c.get('name'), c.get('value'))
                desc = c.get('description', '')
                if desc:
                    desc = " \\n\n  Description: " + desc
                s += desc + "\n"
            s += "\n"
    except:
        print("Error: Could not parse module config.")
    return s


def get_doc_sections(module):
    s = ""
    mname = module.get('name', '')
    try:
        secs = module.findall("./doc/section")
        if secs:
            s += "@subsection af_section Airframe file section\n\n"
            for sec in secs:
                sname = sec.get('name')
                s += "- @b section name: @c " + sname
                p = sec.get('prefix', '')
                if p:
                    s += " prefix: @c " + p
                s += "\n"
                defs = sec.findall("./define")
                #print("module {0} has {1} defines in section {2}".format(mname, len(defs), sname))
                for d in defs:
                    s += "  - @b name @c {0} @b value: <em>{1}</em>".format(d.get('name'), d.get('value'))
                    desc = d.get('description', '')
                    if desc:
                        desc = " \\n\n    Description: " + desc
                    s += desc + "\n"
                s += "\n"
    except:
        print("Error: Could not parse doc/section of module " + mname)
    return s


def module_configuration(module):
    doc = get_doc_config_option(module, 'configure')
    doc += get_doc_config_option(module, 'define')
    doc += get_doc_sections(module)
    if doc:
        return "@section configuration Module configuration options\n\n" + doc
    else:
        return ""


def get_module_description(module):
    desc = module.find("./doc/description")
    details = "No detailed description...\n"
    if desc is None or desc.text is None:
        brief = module.get('name').replace('_', ' ').title()
    else:
        # treat first line until dot as brief
        d = re.split(r'\.|\n', desc.text.strip(), 1)
        brief = d[0].strip()
        if len(d) > 1:
            details = d[1].strip()+"\n"
    return brief, details


def get_headers(module):
    return [f.get('name') for f in module.findall("./header/file")]


def headers_list(module):
    headers = get_headers(module)
    if headers:
        s = "@subsection headers Header Files\n"
        s += "The following headers are automatically included in modules.h\n\n"
        for h in headers:
            s += dox_list_file(h)
        return s + "\n"
    else:
        return ""


def get_source_files(module):
    default_dir = os.path.join("modules", get_module_dir(module))
    sources = [os.path.join(f.get("dir", default_dir), f.get("name")) for f in module.findall("./makefile/file")]
    arch_sources = ["arch dependent: " + os.path.join(f.get("dir", default_dir), f.get("name")) for f in module.findall("./makefile/file_arch")]
    return sources + arch_sources


def sources_list(module):
    files = get_source_files(module)
    if files:
        s = "@subsection sources Source Files\n\n"
        #s += "List of source files\n\n"
        for f in files:
            s += dox_list_file(f)
        return s + "\n"
    else:
        return ""


def get_init_functions(module):
    s = ""
    inits = [f.get('fun') for f in module.findall("./init")]
    if inits:
        s += "@subsection init_functions Init Functions\n\n"
        s += "These initialization functions are called once on startup.\n\n"
        for init in inits:
            s += "- " + init + "\n"
        s += "\n"
    return s


def get_event_functions(module):
    s = ""
    events = [f.get('fun') for f in module.findall("./event")]
    if events:
        s += "@subsection event_functions Event Functions\n\n"
        s += "These event functions are called in each cycle of the module event loop.\n\n"
        for event in events:
            s += "- " + event + "\n"
        s += "\n"
    return s


def get_periodic_functions(module):
    s = ""
    periodics = module.findall("./periodic")
    if periodics:
        s += "@subsection periodic_functions Periodic Functions\n\n"
        s += "These functions are called periodically at the specified frequency from the module periodic loop.\n\n"
        for p in periodics:
            s += "- {0}\n".format(p.get('fun', ''))
            if p.get('period', ''):
                s += "  - Period in seconds: @a {0}\n".format(p.get('period'))
            elif p.get('freq', ''):
                s += "  - Frequency in Hz: @a {0}\n".format(p.get('freq'))
            else:
                s += "  - Running at maximum module frequency.\n"
            if p.get('delay', ''):
                s += "  - Delay: @a {0} \\n\n".format(p.get('delay'))
                s += "    Integer to impose a sequence (between 0 and main_freq/function_freq)\n"
            # default for autorun is LOCK
            autorun = p.get('autorun', 'LOCK')
            s += "  - Autorun: @a {0} \\n\n".format(autorun)
            if autorun == "TRUE":
                s += "    Periodic function automatically starts after init.\n"
            elif autorun == "FALSE":
                s += "    Periodic function is started by user command.\n"
            else:
                s += "    Periodic function automatically starts after init and can't be stopped.\n"
            if p.get('start', ''):
                s += "  - Start function: {0}\\n\n".format(p.get('start'))
                s += "    Executed before the periodic function starts.\n"
            if p.get('stop', '') and autorun != "LOCK":
                s += "  - Stop function: {0}\\n\n".format(p.get('stop'))
                s += "    Executed after the periodic function stops.\n"
    return s


def module_functions(module):
    fdoc = get_init_functions(module)
    fdoc += get_event_functions(module)
    fdoc += get_periodic_functions(module)
    if fdoc:
        return "@section functions Module functions\n\n" + fdoc + "\n"
    else:
        return ""


def read_module_file(file):
    try:
        tree = ET.parse(file)
    except ET.ParseError:
        print("Error. Xml file {0} is not well formed.".format(file))
        return None

    root = tree.getroot()
    if root.tag != "module":
        print("Error. Xml file {0} doesn't have 'module' as root node.".format(file))
        return None
    else:
        return root


if __name__ == '__main__':
    usage = "Usage: %prog [options] modules/dir" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-i", "--inputdir", dest="input_dir",
                      help="read input from DIR [default: PAPARAZZI_HOME/conf/modules]", metavar="DIR")
    parser.add_option("-o", "--outputdir", dest="output_dir",
                      help="write output to DIR [default: PAPARAZZI_HOME/doc/manual/generated]", metavar="DIR")
    parser.add_option("-p", "--parents",
                      action="store_true", dest="create_parent_dirs",
                      help="Create parent dirs of output dir if they don't exist.")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()

    # if PAPARAZZI_HOME not set, then assume the tree containing this
    # file is a reasonable substitute
    paparazzi_home = os.getenv(
        "PAPARAZZI_HOME",
        os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../')))

    if options.input_dir:
        modules_dir = options.input_dir
    else:
        modules_dir = os.path.join(paparazzi_home, "conf/modules")
    if not os.path.isdir(modules_dir):
        print("Input directory with modules " + modules_dir + " not found.")
        sys.exit(1)

    if options.output_dir:
        output_dir = options.output_dir
    else:
        output_dir = os.path.join(paparazzi_home, "doc/manual/generated")

    if not os.path.isdir(output_dir):
        if options.create_parent_dirs:
            print("Output directory " + output_dir + " doesn't exit yet. Creating it.")
            os.makedirs(output_dir)
        else:
            print("Output directory " + output_dir + " not valid.")
            sys.exit(1)

    if options.verbose:
        print("Generating module documentation in " + output_dir)

    # dictionary containing all modules
    modules = {}

    # get all xml files in modules_dir
    os.chdir(modules_dir)
    for file in glob.glob("*.xml"):
        module = read_module_file(file)
        if module is not None:
            modules[file] = module

    # generate overview
    outstring = modules_overview_page(modules)

    # generate each module subpage
    for (n, m) in modules.items():
        outstring += module_page(n, m)

    #print(outstring)

    outfile_name = os.path.join(output_dir, "onboard_modules.dox")
    with open(outfile_name, 'w') as outfile:
        outfile.write(outstring)
    if options.verbose:
        print("Done.")
