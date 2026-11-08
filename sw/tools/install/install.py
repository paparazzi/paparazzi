#!/usr/bin/env python3
"""Paparazzi installer: installs the components of components.txt (./install.sh --help).

./install.sh starts it with /usr/bin/python3 (sw/tools/install/install.sh first installs python3 when it is
missing, e.g. in a bare Docker image). Python 3.8 and the standard library only: it runs before anything else
is installed.

The window and the terminal menu are in ui.py, the documentation in docs.py, the special steps of the recipe
are the bash scripts in steps/ (each also runs by itself: steps/<name>.sh --help).

Sections: settings, the recipe, the installer (this computer, available, installed, needs, apt, Python modules,
steps, install, list, documentation), on demand (--for), the command line.
"""

import argparse
import datetime
import filecmp
import glob
import grp
import importlib.util
import json
import os
import shutil
import subprocess
import sys

# ── Settings ──────────────────────────────────────────────────────────────────

INSTALL_DIR = os.path.dirname(os.path.abspath(__file__))
PPRZ_DIR = os.path.normpath(os.path.join(INSTALL_DIR, '..', '..', '..'))
RECIPE = os.path.join(INSTALL_DIR, 'components.txt')
STEPS_DIR = os.path.join(INSTALL_DIR, 'steps')
ASKPASS = os.path.join(INSTALL_DIR, 'askpass.py')
VENV_NAME = 'pprzEnv'
DOCS_URL = 'https://paparazzi-uav.readthedocs.io/en/latest/getting_started/install.html'

# --os: the Ubuntu releases that can be simulated, with their codename and the version of their system Python
UBUNTU = {
    '20.04': ('focal', '3.8'),
    '22.04': ('jammy', '3.10'),
    '24.04': ('noble', '3.12'),
    '26.04': ('resolute', '3.14'),
}
CODENAMES = {codename: version for version, (codename, _) in UBUNTU.items()}
# Debian: the package names of the Ubuntu release of the same time (newer Debian: 24.04)
DEBIAN = {'11': '20.04', '12': '22.04'}

PY_MODES = {
    'system': 'apt packages for the system Python',
    'venv': 'the venv pprzEnv with the system packages, plus pip for the rest',
    'conda': 'pip in the active conda environment',
}

# A small program that prints the modules (its arguments) that the Python running it can't import
FIND_MISSING = '''
import importlib.util, sys
for m in sys.argv[1:]:
    try:
        ok = importlib.util.find_spec(m) is not None
    except Exception:
        ok = False
    if not ok:
        print(m)
'''


def quote(arg):
    """An argument as it must be typed in a shell: in quotes only when the shell would split or redirect it
    (e.g. 'empy<=3.3.4'). ~, $USER and * stay as they are, so that a copied command works."""
    if "'" in arg:
        return '"%s"' % arg
    if any(c in arg for c in ' <>|&;()'):
        return "'%s'" % arg
    return arg


# ── The recipe ────────────────────────────────────────────────────────────────

def version_tuple(text):
    """A version that compares: '24.04' -> (24, 4), '3.13' -> (3, 13), '12' -> (12, 0), unknown -> (0, 0)."""
    numbers = []
    for part in text.split('.')[:2]:
        if part.isdigit():
            numbers.append(int(part))
    while len(numbers) < 2:
        numbers.append(0)
    return tuple(numbers)


def in_range(version, text):
    """True when `version` is in the range `text`: 24.04-25.10, 26.04- (and newer) or -22.04 (and older)."""
    low, _, high = text.partition('-')
    if low and version_tuple(version) < version_tuple(low):
        return False
    if high and version_tuple(version) > version_tuple(high):
        return False
    return True


def read_recipe(path=RECIPE):
    """The recipe: {component id: [(key, value, condition), ...]}, in the order of the file. The spaces that align
    the file are made single. `condition` is the list of words after the last ' if ' of the line (or [])."""
    recipe = {}
    lines = None
    with open(path) as f:
        for text in f:
            fields = text.split()
            if not fields or fields[0].startswith('#'):
                continue
            if fields[0].startswith('['):
                lines = recipe.setdefault(fields[0].strip('[]'), [])
                continue
            if lines is None:
                continue
            key = fields[0]
            value = ' '.join(fields[1:])
            condition = []
            if ' if ' in value:
                value, _, after = value.rpartition(' if ')
                condition = after.split()
            lines.append((key, value, condition))
    return recipe


def values(lines, key):
    """The values of `key` in the lines (key, value) of a component."""
    return [value for k, value in lines if k == key]


def words(lines, key):
    """All the words of the values of `key`, e.g. every apt package of a component."""
    result = []
    for value in values(lines, key):
        result += value.split()
    return result


class Component:
    """What the menus, --list and the Paparazzi Center show of a component."""

    def __init__(self, cid, state, default, python, title, reason, about, provides):
        self.id = cid
        self.state = state                # 'installed', '-' or 'n/a'
        self.default = default            # selected by default when not installed yet
        self.python = python              # it has Python modules
        self.title = title
        self.reason = reason              # why it is not available here ('' when it is)
        self.about = about
        self.provides = provides          # files or commands a build may ask for (--for)
        self.installed = state == 'installed'


# ── The installer ─────────────────────────────────────────────────────────────

class Installer:
    """This computer (or a simulated fresh Ubuntu with --os), the recipe as it applies here, and how to install."""

    def __init__(self, os_arg=None, py_mode=None, dry_run=False, askpass=False, no_recommends=False):
        self.recipe = read_recipe()
        self.dry_run = dry_run
        self.no_recommends = no_recommends
        self.sim = bool(os_arg)           # --os: a fresh Ubuntu, nothing installed
        self.doc = False                  # the commands as written in the documentation (~/paparazzi, $USER)
        self.for_build = False            # installing for a build (--for): ends with "The build continues."
        self.sinks = [sys.stdout]         # where the output goes (also var/install.log, or a window)
        self.notes = []
        self.failed = False
        self.apt_need_update = True
        self.detect_system(os_arg)
        self.py_mode = py_mode or self.default_py_mode()
        self.sudo = self.sudo_command(askpass)

    # ── This computer ─────────────────────────────────────────────────────────

    def detect_system(self, os_arg):
        """Set os_id (ubuntu, debian, macos...), os_ver (the Ubuntu version whose packages apply), codename,
        os_name, warning, py_ver (the system Python) and wsl."""
        self.os_id, self.os_ver, self.codename = 'unknown', '0', ''
        self.os_name, self.warning = 'this system', ''
        self.py_ver = '%d.%d' % sys.version_info[:2]
        self.wsl = False
        if os_arg:
            self.os_id, self.os_ver = 'ubuntu', os_arg
            self.codename, self.py_ver = UBUNTU[os_arg]
            self.os_name = 'Ubuntu %s (simulated, nothing installed)' % os_arg
            return
        if sys.platform == 'darwin':
            self.os_id = 'macos'
            self.os_name = 'macOS ' + subprocess.run(['sw_vers', '-productVersion'], stdout=subprocess.PIPE,
                                                     stderr=subprocess.DEVNULL, text=True).stdout.strip()
        elif os.path.exists('/etc/os-release'):
            release = os_release()
            self.os_id = release.get('ID', 'unknown')
            self.os_ver = release.get('VERSION_ID', '0')
            self.codename = release.get('VERSION_CODENAME', '')
            self.os_name = release.get('PRETTY_NAME', self.os_id + ' ' + self.os_ver)
            # Ubuntu derivatives (Linux Mint, Pop!_OS, ...): the Ubuntu release they are based on
            if self.os_id != 'ubuntu' and release.get('UBUNTU_CODENAME'):
                self.os_id = 'ubuntu'
                self.codename = release['UBUNTU_CODENAME']
                self.os_ver = CODENAMES.get(self.codename, self.os_ver)
            if self.os_id == 'debian':
                self.os_ver = DEBIAN.get(self.os_ver, '24.04')
                self.warning = ('%s: not tested. The components that need the Paparazzi PPA (Ubuntu) are not '
                                'available, the others install the packages of Ubuntu %s (some may be missing or '
                                'have another name).' % (self.os_name, self.os_ver))
        if self.os_id == 'ubuntu' and not in_range(self.os_ver, '22.04-26.04'):
            self.warning = ('%s: not tested (tested: Ubuntu 22.04, 24.04 and 26.04). Some components may fail.'
                            % self.os_name)
        if not shutil.which('apt-get'):
            self.warning = ('%s: this installer needs apt (Ubuntu or Debian). Install by hand: %s, or use a Docker '
                            'image: docker/README.md' % (self.os_name, DOCS_URL))
        # Windows Subsystem for Linux: like Ubuntu, but USB devices must be shared from Windows
        self.wsl = bool(os.environ.get('WSL_DISTRO_NAME')) or 'microsoft' in read_file('/proc/version').lower()

    def default_py_mode(self):
        """The existing venv, else the active conda environment (not base), else system. Ubuntu 26.04: venv (pip is
        not allowed in the system Python and the build needs empy 3 from pip)."""
        if not self.sim and os.access(os.path.join(PPRZ_DIR, VENV_NAME, 'bin', 'python3'), os.X_OK):
            return 'venv'
        if not self.sim and os.environ.get('CONDA_PREFIX') and os.environ.get('CONDA_DEFAULT_ENV', 'base') != 'base':
            return 'conda'
        if in_range(self.os_ver, '26.04-'):
            return 'venv'
        return 'system'

    def sudo_command(self, askpass):
        """sudo: nothing as root; with --askpass the password is asked in a window; without terminal (e.g. a build
        from the Paparazzi Center) only without password, else sudo would wait for a password nobody sees."""
        if os.geteuid() == 0 and not self.sim:
            return []
        if askpass:
            return ['sudo', '-A']
        if not sys.stdin.isatty() and not self.dry_run:
            return ['sudo', '-n']
        return ['sudo']

    def has_apt(self):
        """True when this computer has apt (always with --os)."""
        return self.sim or shutil.which('apt-get') is not None

    def packages_installed(self, packages):
        """True when all the apt packages are installed (one dpkg-query for all of them)."""
        if self.sim or not shutil.which('dpkg-query'):
            return False
        if not packages:
            return True
        out = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Status}\\n'] + packages, stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, text=True).stdout
        installed = [line.split()[0] for line in out.splitlines() if line.endswith('ok installed')]
        return all(p in installed for p in packages)

    def apt_has(self, package):
        """True when apt can install the package."""
        out = subprocess.run(['apt-cache', 'policy', package], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True).stdout
        return 'Candidate:' in out and 'Candidate: (none)' not in out

    def apt_sources_have(self, text):
        """True when `text` is in the apt sources already."""
        if self.sim:
            return False
        files = ['/etc/apt/sources.list']
        for folder, _, names in os.walk('/etc/apt/sources.list.d'):
            files += [os.path.join(folder, name) for name in names]
        return any(text in read_file(path) for path in files)

    def missing_modules(self, python, modules):
        """The modules that `python` can't import (all of them when there is no such Python, or with --os)."""
        if self.sim or not os.access(python, os.X_OK):
            return list(modules)
        if not modules:
            return []
        out = subprocess.run([python, '-c', FIND_MISSING] + list(modules), stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, text=True).stdout
        return out.split()

    def py_externally_managed(self):
        """True when the system Python refuses pip (Ubuntu 23.04 and newer)."""
        if self.sim:
            return in_range(self.os_ver, '23.04-')
        return bool(glob.glob('/usr/lib/python3*/EXTERNALLY-MANAGED'))

    def py_bin(self):
        """The Python of the Python mode."""
        if self.py_mode == 'venv':
            return os.path.join(PPRZ_DIR, VENV_NAME, 'bin', 'python3')
        if self.py_mode == 'conda':
            return os.path.join(os.environ.get('CONDA_PREFIX', '/nonexistent'), 'bin', 'python')
        return '/usr/bin/python3'

    def needs_group(self, group):
        """True when the user is not in `group` yet (root needs no group; --os: a new user)."""
        if self.sim:
            return True
        if os.geteuid() == 0:
            return False
        names = []
        for gid in set(os.getgroups() + [os.getegid()]):
            try:
                names.append(grp.getgrgid(gid).gr_name)
            except KeyError:
                pass
        return group not in names

    # ── The recipe here ───────────────────────────────────────────────────────

    def condition_holds(self, condition):
        """True when the condition after 'if' holds here: 24.04-25.10, python 3.13-, missing docker,
        not-in-group docker, wsl."""
        kind = condition[0]
        if kind == 'python':
            return in_range(self.py_ver, condition[1])
        if kind == 'missing':
            return self.sim or shutil.which(condition[1]) is None
        if kind == 'not-in-group':
            return self.needs_group(condition[1])
        if kind == 'wsl':
            return self.wsl
        return in_range(self.os_ver, kind)

    def components(self):
        """The component ids, in the order of the recipe."""
        return list(self.recipe)

    def lines(self, cid):
        """The lines (key, value) of component `cid` whose condition holds here."""
        result = []
        for key, value, condition in self.recipe.get(cid, []):
            if not condition or self.condition_holds(condition):
                result.append((key, value))
        return result

    def value(self, cid, key):
        """The first value of `key` of component `cid` ('' when there is none), e.g. value('core', 'title')."""
        found = values(self.lines(cid), key)
        return found[0] if found else ''

    # ── Available, installed, needs ───────────────────────────────────────────

    def unavailable_reason(self, cid):
        """Why component `cid` can't be installed here ('' when it can). The components it needs must be
        available, and all the words of its 'on' lines must hold."""
        lines = self.lines(cid)
        why = self.value(cid, 'why')
        for dep in words(lines, 'needs'):
            reason = self.unavailable_reason(dep)
            if reason:
                return 'needs %s: %s' % (dep, reason)
        for word in words(lines, 'on'):
            if word == 'apt' and not self.has_apt():
                return 'needs apt (Ubuntu or Debian), see the manual installation'
            if word == 'ppa' and self.os_id != 'ubuntu':
                return 'needs the Paparazzi PPA (Ubuntu), see the manual installation'
            if word == 'ubuntu' and self.os_id != 'ubuntu':
                return why or 'Ubuntu only'
            if word == 'amd64' and not self.sim and dpkg_architecture() != 'amd64':
                return why or 'amd64 only'
            if word not in ('apt', 'ppa', 'ubuntu', 'amd64') and not in_range(self.os_ver, word):
                return why or 'Ubuntu ' + word
        return ''

    def installed(self, cid):
        """True when component `cid` is installed: its 'check' lines (all must hold), or else all its apt
        packages are installed and all its Python modules can be imported. With --os nothing is installed."""
        if self.sim:
            return False
        lines = self.lines(cid)
        checks = [value.split() for value in values(lines, 'check')] or [['packages'], ['python']]
        for check in checks:
            kind, args = check[0], check[1:]
            if kind == 'packages' and not self.packages_installed(words(lines, 'apt')):
                return False
            if kind == 'python':
                modules = [value.split()[0] for value in values(lines, 'py')]
                if self.missing_modules(self.py_bin(), modules):
                    return False
            if kind == 'file':
                path = args[0] if args[0].startswith('/') else os.path.join(PPRZ_DIR, args[0])
                if not os.path.exists(path):
                    return False
            if kind == 'runs' and not runs(args):
                return False
            if kind == 'command' and shutil.which(args[0]) is None:
                return False
            if kind == 'group' and self.needs_group(args[0]):
                return False
            if kind == 'udev-rules' and not udev_rules_installed():
                return False
        return True

    def with_needs(self, ids):
        """The components `ids` and the ones they need that are not installed yet, in the order of the recipe."""
        want = list(ids)
        changed = True
        while changed:
            changed = False
            for cid in list(want):
                for dep in self.value(cid, 'needs').split():
                    if dep not in want and not self.installed(dep):
                        want.append(dep)
                        changed = True
        return [cid for cid in self.components() if cid in want]

    def info(self, cid):
        """The Component (state, title, ...) of `cid` for the menus, --list and the Paparazzi Center."""
        lines = self.lines(cid)
        reason = self.unavailable_reason(cid)
        if reason:
            state = 'n/a'
        elif self.installed(cid):
            state = 'installed'
        else:
            state = '-'
        return Component(cid, state, self.value(cid, 'default') == 'yes', bool(values(lines, 'py')),
                         self.value(cid, 'title'), reason, self.value(cid, 'about'), words(lines, 'provides'))

    # ── Output and commands ───────────────────────────────────────────────────

    def echo(self, text=''):
        """Show a line of output (terminal, log file, window)."""
        for sink in self.sinks:
            try:
                sink.write(text + '\n')
                sink.flush()
            except OSError:          # e.g. the window was closed: keep installing
                pass

    def note(self, text):
        """A message for the user, repeated at the end."""
        self.echo('  Note: ' + text)
        self.notes.append('  Note: ' + text)

    def stream(self, cmd, env=None):
        """Run a command with its output shown by echo. Returns its exit code."""
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                    env=env, text=True, errors='replace')
        except OSError as error:
            self.echo('  %s: %s' % (cmd[0], error))
            return 127
        for line in proc.stdout:
            self.echo(line.rstrip('\n'))
        return proc.wait()

    def run(self, *cmd):
        """Show a command (to copy), then run it, except in a dry run. False (and self.failed) when it failed."""
        self.echo('  $ ' + ' '.join(quote(arg) for arg in cmd))
        if self.dry_run:
            return True
        if self.stream(list(cmd)) != 0:
            self.echo('  FAILED: ' + ' '.join(cmd))
            self.failed = True
            return False
        return True

    def step_env(self):
        """The settings the step scripts read (steps/common.sh)."""
        env = dict(os.environ)
        env.update({
            'PPRZ_INSTALL': '1',
            'PPRZ_DIR': '~/paparazzi' if self.doc else PPRZ_DIR,
            'SUDO': ' '.join(self.sudo),
            'DRY_RUN': '1' if self.dry_run else '0',
            'PY_MODE': self.py_mode,
            'VENV_NAME': VENV_NAME,
            'SIM': '1' if self.sim else '0',
            'DOC': '1' if self.doc else '0',
        })
        return env

    def step(self, args):
        """Run the step script steps/<args[0]>.sh with its arguments (the script shows its own commands)."""
        if self.stream([os.path.join(STEPS_DIR, args[0] + '.sh')] + args[1:], self.step_env()) != 0:
            self.failed = True

    # ── apt ───────────────────────────────────────────────────────────────────

    def apt_update(self):
        """apt-get update, once after each new repository."""
        if self.apt_need_update and self.run(*self.sudo, 'apt-get', 'update'):
            self.apt_need_update = False

    def apt_install(self, packages):
        """Install the apt packages (nothing without packages)."""
        if not packages:
            return
        self.apt_update()
        options = ['-y']
        if self.no_recommends:
            options.append('--no-install-recommends')
        self.run(*self.sudo, 'env', 'DEBIAN_FRONTEND=noninteractive', 'apt-get', 'install', *options, *packages)

    def add_repo(self, repo):
        """Add the apt repository `repo`: ppa:<name>, or osrf (packages.osrfoundation.org, for Gazebo)."""
        if repo.startswith('ppa:'):
            if self.apt_sources_have(repo[4:]):
                return
            if self.sim or not shutil.which('add-apt-repository'):
                self.apt_install(['software-properties-common'])
            self.run(*self.sudo, 'add-apt-repository', '-y', repo)
        elif repo == 'osrf':
            if self.apt_sources_have('packages.osrfoundation.org/gazebo'):
                return
            if self.sim or not shutil.which('curl'):
                self.apt_install(['curl', 'ca-certificates'])
            key = '/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg'
            arch = 'amd64' if self.sim else dpkg_architecture()
            source = ('deb [arch=%s signed-by=%s] http://packages.osrfoundation.org/gazebo/ubuntu-stable %s main'
                      % (arch, key, self.codename))
            self.run(*self.sudo, 'curl', '-sSL', 'https://packages.osrfoundation.org/gazebo.gpg', '-o', key)
            self.run(*self.sudo, 'sh', '-c', "echo '%s' > /etc/apt/sources.list.d/gazebo-stable.list" % source)
        self.apt_need_update = True

    # ── Python modules ────────────────────────────────────────────────────────
    #
    # system  apt python3-* packages for the system Python. Modules without apt package: pip --user where the
    #         system allows it (before Ubuntu 23.04), otherwise a note.
    # venv    the same apt packages, plus the venv pprzEnv (with the system packages) for the modules without apt
    #         package. ./paparazzi uses pprzEnv when it exists.
    # conda   pip in the active conda environment.

    def setup_venv(self):
        """Create the venv pprzEnv (with the system packages) when it is not there."""
        venv = os.path.join(PPRZ_DIR, VENV_NAME)
        if not self.sim and os.access(os.path.join(venv, 'bin', 'python3'), os.X_OK):
            return
        if self.sim or importlib.util.find_spec('ensurepip') is None:
            self.apt_install(['python3-venv'])
        self.run('/usr/bin/python3', '-m', 'venv', '--system-site-packages', venv)
        link = os.path.join(venv, 'bin', 'calibrate.py')
        if self.sim or not os.path.lexists(link):
            self.run('ln', '-s', '../../sw/tools/calibration/calibrate.py', link)

    def py_install(self, lines):
        """Install the Python modules of a component (its lines "py <module> <apt package|-> <pip|->")."""
        modules = [value.split() for value in values(lines, 'py')]
        if not modules:
            return
        # the apt packages (not for conda)
        if self.py_mode != 'conda':
            apts = [apt for _, apt, _ in modules if apt != '-' and (self.dry_run or self.apt_has(apt))]
            self.apt_install(apts)
        if self.py_mode == 'venv':
            self.setup_venv()
        python = self.py_bin()
        if self.py_mode == 'conda' and not os.access(python, os.X_OK):
            self.echo("  FAILED: no conda environment active: run 'conda activate <env>' first")
            self.failed = True
            return

        # the modules still missing. In a dry run the apt packages are not installed yet: only the modules without
        # apt package, and the venv may not exist yet: ask the system Python.
        if self.dry_run and self.py_mode != 'conda':
            check = python if os.access(python, os.X_OK) else '/usr/bin/python3'
            missing = self.missing_modules(check, [module for module, apt, _ in modules if apt == '-'])
        else:
            missing = self.missing_modules(python, [module for module, _, _ in modules])
        pips = [pip for module, _, pip in modules if module in missing and pip != '-']
        if not pips:
            return

        if self.py_mode != 'system':
            self.run(python, '-m', 'pip', 'install', *pips)
        elif self.py_externally_managed():
            self.note('Not installed (no apt package, and pip is not allowed in the system Python): %s. Choose the '
                      'venv (--python=venv) if you need them' % ' '.join(pips))
        else:
            if self.sim or not runs(['/usr/bin/python3', '-m', 'pip', '--version']):
                self.apt_install(['python3-pip'])
            # as root (Docker images): for all users
            if os.geteuid() == 0:
                self.run('/usr/bin/python3', '-m', 'pip', 'install', *pips)
            else:
                self.run('/usr/bin/python3', '-m', 'pip', 'install', '--user', *pips)

    # ── Install ───────────────────────────────────────────────────────────────

    def install_component(self, cid):
        """Install component `cid`: repositories, apt packages, Python modules, then its steps and notes in the
        order of the recipe. True when everything worked."""
        lines = self.lines(cid)
        title = self.value(cid, 'title')
        self.failed = False
        self.echo()
        self.echo('== ' + title)
        for repo in values(lines, 'repo'):
            self.add_repo(repo)
        self.apt_install(words(lines, 'apt'))
        self.py_install(lines)
        for key, value in lines:
            if key == 'step':
                self.step(value.split())
            elif key == 'note':
                self.note(value)
        self.echo(('  FAILED: ' if self.failed else '  ok: ') + title)
        return not self.failed

    def install_all(self, ids):
        """Install the components `ids`, then show a summary. True when all worked."""
        self.echo('Paparazzi installer - %s - %s' % (self.os_name, datetime.datetime.now().strftime('%Y-%m-%d %H:%M')))
        self.echo('Python packages: ' + self.py_mode)
        done, failed = [], []
        for cid in ids:
            if self.install_component(cid):
                done.append(cid)
            else:
                failed.append(cid)
        self.echo()
        self.echo('== Summary')
        for cid in done:
            self.echo('  ok: ' + self.value(cid, 'title'))
        for cid in failed:
            self.echo('  FAILED: ' + self.value(cid, 'title'))
        for note in self.notes:
            self.echo(note)
        self.echo()
        if failed:
            self.echo('  Check the errors above and the documentation: ' + DOCS_URL)
            return False
        if self.for_build:
            self.echo('  Done. The build continues.')
        else:
            self.echo('  Done. Start the Paparazzi Center with: ./paparazzi')
        return True

    def install_logged(self, ids):
        """install_all, with the output also in var/install.log (not in a dry run). True when all worked."""
        if self.dry_run:
            return self.install_all(ids)
        os.makedirs(os.path.join(PPRZ_DIR, 'var'), exist_ok=True)
        with open(os.path.join(PPRZ_DIR, 'var', 'install.log'), 'a') as log:
            self.sinks.append(log)
            ok = self.install_all(ids)
            self.sinks.remove(log)
        self.echo('  Log: var/install.log')
        return ok

    def sudo_once(self, ids):
        """Ask for the sudo password once, before installing. False when sudo is refused or can't ask."""
        if self.dry_run or not self.sudo:
            return True
        if self.sudo == ['sudo', '-n'] and not runs(['sudo', '-n', '-v']):
            self.echo('Installing needs sudo: run in a terminal: ./install.sh ' + ' '.join(ids))
            return False
        return subprocess.call(self.sudo + ['-v']) == 0

    # ── List and documentation ────────────────────────────────────────────────

    def print_list(self):
        """The components with their state and title (--list)."""
        if self.warning:
            print(self.warning)
            print()
        print('%-16s %-10s %s' % ('component', 'state', 'description'))
        for cid in self.components():
            comp = self.info(cid)
            print('%-16s %-10s %s' % (cid, comp.state, comp.title))
            if comp.reason:
                print('%-16s %-10s (%s)' % ('', '', comp.reason))

    def print_json(self):
        """The same for programs (--list --json, the Paparazzi Center)."""
        comps = [vars(self.info(cid)) for cid in self.components()]
        print(json.dumps({'system': self.os_name, 'python': self.py_mode, 'warning': self.warning,
                          'components': comps}, indent=1))

    def describe(self):
        """For docs.py (with --os): {component id: [(key, value), ...]}: 'unavailable' and the reason, the recipe
        lines that apply, and 'command' lines with the commands of its steps, as written in the documentation."""
        self.dry_run, self.doc, self.sudo, self.py_mode = True, True, ['sudo'], 'system'
        result = {}
        for cid in self.components():
            lines = self.lines(cid)
            reason = self.unavailable_reason(cid)
            if reason:
                lines = [('unavailable', reason)] + lines
            for value in values(self.lines(cid), 'step'):
                args = value.split()
                out = subprocess.run([os.path.join(STEPS_DIR, args[0] + '.sh')] + args[1:], env=self.step_env(),
                                     stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, text=True).stdout
                lines += [('command', line[4:]) for line in out.splitlines() if line.startswith('  $ ')]
            result[cid] = lines
        return result


# ── Small helpers ─────────────────────────────────────────────────────────────

def read_file(path):
    """The text of a file ('' when it can't be read)."""
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def os_release():
    """The fields of /etc/os-release (ID, VERSION_ID, ...)."""
    fields = {}
    for line in read_file('/etc/os-release').splitlines():
        key, _, value = line.partition('=')
        fields[key.strip()] = value.strip().strip('"')
    return fields


def runs(cmd):
    """True when the command runs and succeeds (output hidden)."""
    try:
        return subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              stdin=subprocess.DEVNULL).returncode == 0
    except OSError:
        return False


def dpkg_architecture():
    """amd64, arm64, ... ('' without dpkg)."""
    try:
        return subprocess.run(['dpkg', '--print-architecture'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              text=True).stdout.strip()
    except OSError:
        return ''


def udev_rules_installed():
    """True when the udev rules of conf/system/udev/rules are in /etc/udev/rules.d."""
    for rule in glob.glob(os.path.join(PPRZ_DIR, 'conf', 'system', 'udev', 'rules', '*.rules')):
        installed = os.path.join('/etc/udev/rules.d', os.path.basename(rule))
        if not os.path.exists(installed) or not filecmp.cmp(rule, installed, shallow=False):
            return False
    return True


def has_display():
    """True on a desktop with zenity (the installer windows)."""
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) and shutil.which('zenity') is not None


# ── On demand: the build needs a file or command ──────────────────────────────

def on_demand(inst, need, yes):
    """./install.sh --for=<need>, e.g. from conf/Makefile.on_demand while compiling an airframe: find the component
    that provides it, ask in the terminal, else in a window, else say how to install it. All output goes to
    stderr (make reads stdout). Exit code 0: there or installed, 1: not installed, 2: no component provides it."""
    if os.path.exists(need) or shutil.which(need):
        return 0
    comps = [inst.info(cid) for cid in inst.components()]
    comp = next((c for c in comps if need in c.provides), None)
    if comp is None:
        return 2
    if comp.installed:
        print('%s: %s is installed, but %s is not there' % (need, comp.title, need))
        return 1
    if comp.reason:
        print('%s: %s is not available here: %s' % (need, comp.title, comp.reason))
        return 1
    question = "This build needs %s, from the installer component '%s': %s.\n%s" % (need, comp.id, comp.title,
                                                                                   comp.about)
    inst.for_build = True
    if yes:
        return install_selected(inst, [comp.id], ask=False)
    import ui
    if sys.stdin.isatty():
        # make started in a terminal. Not /dev/tty: the Paparazzi Center may have been started from a terminal,
        # its make has a pipe as stdin and the question would wait, unseen, in that terminal.
        if not ui.ask_in_terminal(question, comp.id):
            print('Not installed. Later: ./install.sh ' + comp.id)
            return 1
        return install_selected(inst, [comp.id], ask=False)
    if has_display():
        # no terminal, a desktop (e.g. a build from the Paparazzi Center): ask in a window
        if not ui.ask_in_window(question, comp.id):
            print('Not installed. Later: ./install.sh ' + comp.id)
            return 1
        return install_in_window(inst, [comp.id])
    print(question)
    print('Install it with: ./install.sh ' + comp.id)
    return 1


# ── Installing what was chosen ────────────────────────────────────────────────

def install_selected(inst, ids, ask):
    """Install the components `ids` and the ones they need. With `ask` (and a terminal): show them and ask first.
    Returns the exit code."""
    todo = inst.with_needs(ids)
    if not todo:
        inst.echo('Nothing selected.')
        return 0
    if inst.warning:
        inst.echo(inst.warning)
    if inst.py_mode == 'conda' and not os.environ.get('CONDA_PREFIX'):
        if any(values(inst.lines(cid), 'py') for cid in todo):
            print('Python packages in conda: activate the conda environment first (conda activate <env>)',
                  file=sys.stderr)
            return 2
    if ask and sys.stdin.isatty():
        print()
        print('  Install:')
        for cid in todo:
            print('    ' + inst.value(cid, 'title'))
        answer = input('  Python packages: %s\n\n  Continue? [Y/n] ' % inst.py_mode)
        if answer.strip() in ('n', 'N', 'no', 'No'):
            return 0
    if not inst.sudo_once(todo):
        return 1
    return 0 if inst.install_logged(todo) else 1


def install_in_window(inst, ids):
    """Install with the sudo password and the output in windows (no terminal). Returns the exit code."""
    import ui
    if inst.sudo:
        os.environ['SUDO_ASKPASS'] = ASKPASS
        inst.sudo = ['sudo', '-A']
    window = ui.LogWindow('Paparazzi installer (log in var/install.log)')
    inst.sinks = [window]
    code = install_selected(inst, ids, ask=False)
    window.close()
    return code


def choose_and_install(inst, ui_choice):
    """No components given: choose them in the window (zenity) or the terminal menu, then install. Without both,
    say how to continue. Returns the exit code."""
    comps = [inst.info(cid) for cid in inst.components()]
    if all(c.reason for c in comps):
        inst.print_list()
        print()
        print('Nothing can be installed here with this installer: ' + DOCS_URL)
        return 1
    if ui_choice != 'text' and has_display():
        import ui
        chosen = ui.choose_in_window(inst, comps)
        if chosen is None:
            return 0
        ids, inst.py_mode = chosen
        if sys.stdin.isatty():
            # started from a terminal: the password and the output there
            return install_selected(inst, ids, ask=False)
        return install_in_window(inst, ids)
    if ui_choice != 'gui' and sys.stdin.isatty() and sys.stdout.isatty() and importlib.util.find_spec('curses'):
        import ui
        try:
            chosen = ui.choose_in_terminal(inst, comps)
        except ui.MenuError as error:
            print(error, file=sys.stderr)
            return 1
        if chosen is None:
            return 0
        ids, inst.py_mode = chosen
        if not ids:
            print('Nothing selected.')
            return 0
        return install_selected(inst, ids, ask=True)
    if inst.warning:
        print(inst.warning)
    print('No menu here (it needs a terminal, or zenity on a desktop). Choose the components:')
    print('  ./install.sh --yes core python arm    (./install.sh --help, the components below)')
    print()
    inst.warning = ''
    inst.print_list()
    return 2


# ── Command line ──────────────────────────────────────────────────────────────

def parse_args(argv):
    """The options of ./install.sh (./install.sh --help)."""
    components = ' '.join(read_recipe())
    parser = argparse.ArgumentParser(
        prog='./install.sh', usage='./install.sh [options] [component ...]',
        description='Without components: choose them in a window (zenity) or a menu in the terminal. With '
                    'components: install those (and the ones they need), e.g. ./install.sh --yes core arm jsbsim',
        epilog='Components: %s. Also: default (the components selected by default), available (all the components '
               'that can be installed here). Documentation: %s' % (components, DOCS_URL))
    parser.add_argument('components', nargs='*', metavar='component', help=argparse.SUPPRESS)
    parser.add_argument('-y', '--yes', action='store_true', help="don't ask (for scripts and Docker)")
    parser.add_argument('-n', '--dry-run', action='store_true', help='only show the commands')
    parser.add_argument('--python', choices=list(PY_MODES), metavar='MODE',
                        help='where the Python modules go: system (apt), venv (%s) or conda' % VENV_NAME)
    parser.add_argument('--text', dest='ui', action='store_const', const='text', default='auto',
                        help='menu in the terminal, also with a desktop')
    parser.add_argument('--gui', dest='ui', action='store_const', const='gui', help='window (zenity)')
    parser.add_argument('--list', action='store_true', help='list the components and whether they are installed')
    parser.add_argument('--json', action='store_true', help='the same list for programs (the Paparazzi Center)')
    parser.add_argument('--for', dest='for_need', metavar='FILE|COMMAND',
                        help='install the component that provides it, after asking (used by the build: '
                             'conf/Makefile.on_demand). Exit 0: there, 1: not installed, 2: no component has it')
    parser.add_argument('--no-recommends', action='store_true', help='apt without recommended packages (Docker)')
    parser.add_argument('--os', choices=list(UBUNTU), metavar='VERSION',
                        help='a fresh Ubuntu VERSION (%s), with --dry-run or --list' % ', '.join(UBUNTU))
    parser.add_argument('--askpass', action='store_true', help='ask the sudo password in a window (sudo -A)')
    parser.add_argument('--doc', action='store_true', help='the documentation of the components (reStructuredText)')
    parser.add_argument('--wiki', action='store_true', help='the same as MediaWiki text')
    parser.add_argument('--doxygen', nargs='?', const='', metavar='DIR', help='the Doxygen page (DIR/install.dox)')
    parser.add_argument('--requirements', action='store_true', help='requirements.txt (pip) of the python component')
    return parser.parse_args(argv)


def selected_ids(inst, names):
    """The component ids of the command line, with 'default' and 'available' replaced. Exits on an unknown or
    unavailable component."""
    ids = []
    for name in names:
        if name in ('default', 'available'):
            for cid in inst.components():
                if not inst.unavailable_reason(cid) and (name == 'available' or inst.value(cid, 'default') == 'yes'):
                    ids.append(cid)
            continue
        if name not in inst.recipe:
            print('Unknown component %s (see --list)' % name, file=sys.stderr)
            sys.exit(2)
        reason = inst.unavailable_reason(name)
        if reason:
            print('%s: %s' % (name, reason), file=sys.stderr)
            sys.exit(2)
        ids.append(name)
    return ids


def main(argv):
    args = parse_args(argv)
    if args.doc or args.wiki or args.requirements or args.doxygen is not None:
        import docs
        if args.doxygen is not None:
            return docs.main(['--doxygen=' + args.doxygen])
        return docs.main(['--doc' if args.doc else '--wiki' if args.wiki else '--requirements'])
    if args.os and not (args.dry_run or args.list or args.json):
        print('--os only simulates: add --dry-run (or --list)', file=sys.stderr)
        return 2
    if args.for_need:
        # make reads stdout: everything to stderr
        sys.stdout = sys.stderr
    inst = Installer(args.os, args.python, args.dry_run, args.askpass, args.no_recommends)
    if args.for_need:
        return on_demand(inst, args.for_need, args.yes)
    if args.json:
        inst.print_json()
        return 0
    if args.list:
        inst.print_list()
        return 0
    if not args.components:
        return choose_and_install(inst, args.ui)
    return install_selected(inst, selected_ids(inst, args.components), ask=not args.yes)


if __name__ == '__main__':
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        sys.exit(1)
