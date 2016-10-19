#!/usr/bin/env python
#
# Copyright (C) 2012-2014 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from __future__ import print_function
import argparse
import os
import sys
import subprocess
import distutils.dir_util
import git  #sudo pip install gitpython

PACKAGES_FILE = "./.packages"


class Package(object):
    def __init__(self, nr, name):
        self._temp_dir_ = "./var/pkgman/"

        self.nr = nr
        self.name = name
        self.url = ""
        self.lpath = ""
        self.lcommit = ""
        self.rpath = ""
        self.rcommit = ""

    def print(self,verbose):
        print(self.nr, ": ", self.name)
        print("     - ", self.url)
        print("     -", self.rpath, "->", self.lpath)
        if verbose:
            print("     -", self.rcommit, "->", self.lcommit)
            print("     -", self.git_temp())

    # local git working directory
    def git_temp(self):
        return self._temp_dir_ + self.name.replace(" ","_").replace("/","_")

packages = []

def copy(src, dst):
    print(" - copy",src, dst)
    try:
        distutils.dir_util.copy_tree(src, dst)
    except:
        raise

def read():
    p = None
    p_id = 1
    with open(PACKAGES_FILE) as f:
        for line in f:
            if "[package" in line:
                p = Package(p_id, line.replace("[package","").replace("]","").replace("\"","").strip())
                packages.append(p)
            elif p is not None:
                if "lpath" in line:
                    p.lpath = line.replace("lpath","").replace("=","").strip()
                if "rpath" in line:
                    p.rpath = line.replace("rpath","").replace("=","").strip()
                if "url" in line:
                    p.url = line.replace("url","").replace("=","").strip()
                if "lcommit" in line:
                    p.lcommit = line.replace("lcommit","").replace("=","").strip()
                if "rcommit" in line:
                    p.rcommit = line.replace("rcommit","").replace("=","").strip()

def store_commit(old_sha, new_sha):
    with open(PACKAGES_FILE, "r+") as f:
        old = f.read()
        f.seek(0)
        f.write(old.replace(old_sha, new_sha))

read()

def pkgman_clean(args):
    print("Clean:\n-----\n")
    cmd = ["rm", "-rf", Package(0,'')._temp_dir_]
    if args.verbose:
        print(cmd)
    out = subprocess.call(cmd)
    print(out)

def verify(p,args):
    print("Package:", p.name)
    if not os.path.exists(p.git_temp()):
        print(" - creating", p.git_temp())
        os.makedirs(p.git_temp())

        print(" - git clone", p.url) 
        git.Repo.clone_from(p.url, p.git_temp())


def pkgman_list(args):
    print("List:\n----\n")

    for p in packages:
        p.print(args.verbose)

def pkgman_status(args):
    for p in packages:
        verify(p,args)

        g1 = git.Repo(p.git_temp()).git
        txt = g1.rev_list( p.rcommit+"..HEAD", "--count")
        print(" - remote commits since last merge:\t",txt)

        if args.verbose:
            print(' - git log:')
            print(g1.log(p.rcommit+"..HEAD", '--pretty=format:   * %h %s, %cr, %aN',  '--abbrev-commit'))

        g2 = git.Repo("./").git
        txt = g2.rev_list(p.lcommit+"..HEAD",  "--count")
        print(" - local commits since last merge:\t",txt)

        if args.verbose:
            print(' - git log:')
            print(g2.log(p.lcommit+"..HEAD", '--pretty=format:   * %h %s, %cr, %aN',  '--abbrev-commit'))

def pkgman_update(args):
    print("Update:\n------\n")

    for p in packages:
        verify(p,args)

        print(" - git update",p.git_temp())
        g = git.Repo(p.git_temp()).git
        txt = g.pull()
        if (args.verbose):
            print(txt)

        # copy remote to local
        copy( p.git_temp() + "/" + p.rpath, p.lpath)

        # store new sha
        sha = g.rev_parse("HEAD")
        print(" - git status",sha)
        store_commit(p.rcommit,sha)
        p.rcommit = sha


# Parse the arguments
parser = argparse.ArgumentParser(description='Package Manager. Use pkgman.py -h for help')
parser.add_argument('-v', '--verbose', action='store_true', help='Show extra information')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('clean', help='Remove temp folders')
subparsers.add_parser('list', help='List Packages')
subparsers.add_parser('status', help='Show the Status of all the Packages')
subparser_update = subparsers.add_parser('update', help='Download Remote Package Updates')
subparser_update.add_argument('-i', '--id', metavar='id', default='0', type=int, help='Package Number')

# Process arguments or show help if none are given
if len(sys.argv[1:])==0:
    parser.print_help()
    parser.exit()
args = parser.parse_args()
if args.verbose:
    print("paparazzi_pkgman",args)

# Call commands
if args.command == 'list':
    pkgman_list(args)
elif args.command == 'clean':
    pkgman_clean(args)
elif args.command == 'update':
    pkgman_update(args)
elif args.command == 'status':
    pkgman_status(args)

