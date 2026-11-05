#!/usr/bin/env python3
"""
Summarize the compile log of the aircraft tests (var/compile.log by default).

The raw log of a full test run is huge: the same warning in a shared file shows up once per aircraft and target.
This groups the output into unique errors and warnings, each with the builds it occurs in, and gives every failed
build one root cause. Problems of the build machine (a missing toolchain or simulator) are listed separately from
problems in the code or the conf files.

Usage: parse_compile_logs.py [log_file] [--markdown] [--max N]
"""

import argparse
import os
import re
import sys
from collections import OrderedDict, defaultdict
from os import path

sys.path.append(path.normpath(path.join(path.dirname(path.abspath(__file__)), 'modules')))
from TAP import Builder

PPRZ_HOME = os.getenv('PAPARAZZI_HOME', path.normpath(path.join(path.dirname(path.abspath(__file__)), '..')))

RE_CONF = re.compile(r'Testing all aircrafts in conf: (\S+)')
RE_COMPILING = re.compile(r'compiling AIRCRAFT: \[(\S+)\] TARGET: \[(\S+)\]')
RE_FAILED = re.compile(r'Error: The command "make AIRCRAFT=(\S+) clean_ac (\S+)\.compile" failed')
RE_WARNED = re.compile(r'Warning: AIRCRAFT=(\S+) target=(\S+) compiled sucessfully but had warnings')
RE_MODULE_TEST = re.compile(r'^(not )?ok \d+ - (.*)$')
RE_GCC = re.compile(r'^(?P<file>[^\s:][^:]*):(?P<line>\d+):(?:\d+:)? (?P<kind>fatal error|error|warning): (?P<msg>.*)$')
RE_LD = re.compile(r'^(?P<file>[^\s:][^:]*):(?:(?P<line>\d+):)?\(\.text[^)]*\): (?P<msg>undefined reference to .*)$')
RE_MAKE_ERR = re.compile(r'\*\*\* (?P<msg>(?!\[).*?)\.?\s+Stop\.$')
RE_LD_MSG = re.compile(r'/ld: (?P<msg>.*)$')
RE_OCAML = re.compile(r'Fatal error: exception (?P<msg>.*)$')
RE_MISSING_AF = re.compile(r'Parsing airframe (\S+): File does not exist')
RE_GEN_WARN = re.compile(r'^Warning: (?!AIRCRAFT=)(?P<msg>.*)$')

# problems of the build machine, not of the code
SETUP_PATTERNS = [
    (re.compile(r'gazebo/gazebo\.hh: No such file'), 'Gazebo Classic not installed (NPS with Gazebo)'),
    (re.compile(r'gz/sim\d*/.*No such file|gz-sim.*not found'), 'Gazebo Harmonic not installed'),
    (re.compile(r'cross-compiler not found'), 'cross-compiler not found (check PREFIX in the airframe)'),
    (re.compile(r'arm-none-eabi-gcc.*not found'), 'arm-none-eabi-gcc not installed'),
]

TAG_RE = re.compile(r'var/aircrafts/[^/]+/[^/]+/')


def norm_path(p):
    p = p.replace(PPRZ_HOME + '/', '')
    p = TAG_RE.sub('var/aircrafts/<AC>/<target>/', p)
    if not path.isabs(p) and not p.startswith(('var/', 'sw/', 'conf/', '<')) and \
            path.exists(path.join(PPRZ_HOME, 'sw/airborne', p)):
        p = 'sw/airborne/' + path.normpath(p)
    return p


def norm_msg(m):
    m = m.replace('‘', "'").replace('’', "'").replace(PPRZ_HOME + '/', '')
    return TAG_RE.sub('var/aircrafts/<AC>/<target>/', m).strip()


def parse_line(line):
    """ Return (kind, location, message) for a diagnostic line, or None """
    line = line.rstrip()
    m = RE_GCC.match(line)
    if m:
        kind = 'error' if 'error' in m.group('kind') else 'warning'
        return kind, f"{norm_path(m.group('file'))}:{m.group('line')}", norm_msg(m.group('msg'))
    m = RE_LD.match(line)
    if m:
        loc = norm_path(m.group('file')) + (f":{m.group('line')}" if m.group('line') else '')
        return 'error', loc, norm_msg(m.group('msg'))
    m = RE_LD_MSG.search(line)
    if m:
        if RE_LD.match(m.group('msg')):
            return parse_line(m.group('msg'))
        if re.search(r"in function `[^']*':$", m.group('msg')):
            return None
        msg = norm_msg(m.group('msg'))
        if msg.startswith('warning:'):
            return 'warning', 'linker', msg[len('warning:'):].strip()
        return 'error', 'linker', msg
    m = RE_MAKE_ERR.search(line)
    if m:
        return 'error', 'make', norm_msg(m.group('msg'))
    m = RE_OCAML.search(line)
    if m:
        return 'error', 'generator', norm_msg(m.group('msg'))
    m = RE_GEN_WARN.match(line)
    if m:
        return 'gen_warning', 'generator', norm_msg(m.group('msg'))
    return None


def setup_issue(msg):
    for regex, name in SETUP_PATTERNS:
        if regex.search(msg):
            return name
    return None


def parse_log(log_file):
    """ Return the list of builds and a dict (kind, loc, msg) -> list of builds """
    builds = OrderedDict()   # (conf, aircraft, target) -> status
    issues = defaultdict(list)
    root_causes = {}
    missing_airframes = []
    conf = 'unknown'
    current = None

    with open(log_file, 'r', errors='replace') as f:
        for line in f:
            m = RE_CONF.search(line)
            if m:
                conf, current = m.group(1), None
                continue
            if 'tests/modules/test_modules.py' in line:
                conf, current = 'modules', None
                continue
            m = RE_COMPILING.search(line)
            if m:
                builds.setdefault((conf, m.group(1), m.group(2)), 'ok')
                continue
            if conf == 'modules':
                m = RE_MODULE_TEST.match(line)
                if m:
                    builds[(conf, m.group(2), '')] = 'failed' if m.group(1) else 'ok'
                    current = (conf, m.group(2), '')
                    continue
            m = RE_FAILED.search(line)
            if m:
                current = (conf, m.group(1), m.group(2))
                builds[current] = 'failed'
                continue
            m = RE_WARNED.search(line)
            if m:
                current = (conf, m.group(1), m.group(2))
                if builds.get(current) != 'failed':
                    builds[current] = 'warnings'
                continue
            m = RE_MISSING_AF.search(line)
            if m:
                if (conf, m.group(1)) not in missing_airframes:
                    missing_airframes.append((conf, m.group(1)))
                continue
            if current is None:
                continue
            diag = parse_line(line)
            if diag is None:
                continue
            kind, loc, msg = diag
            if kind == 'error' and msg.startswith('ld returned'):
                continue
            if current not in issues[diag]:
                issues[diag].append(current)
            if kind == 'error' and builds.get(current) == 'failed' and current not in root_causes:
                root_causes[current] = diag

    return builds, issues, root_causes, missing_airframes


def build_name(b):
    conf, ac, target = b
    return f"{ac} {target}".strip()


def print_report(builds, issues, root_causes, missing_airframes, markdown=False, max_items=0):
    failed = [b for b, s in builds.items() if s == 'failed']
    warned = [b for b, s in builds.items() if s == 'warnings']
    errors = {k: v for k, v in issues.items() if k[0] == 'error'}
    warnings = {k: v for k, v in issues.items() if k[0] == 'warning'}
    gen_warnings = {k: v for k, v in issues.items() if k[0] == 'gen_warning'}
    bullet = '- [ ] ' if markdown else '  - '
    code = (lambda s: f'`{s}`') if markdown else (lambda s: s)

    def limit(items):
        return items[:max_items] if max_items else items

    def where(blds):
        confs = sorted(set(b[0] for b in blds))
        uniq = list(OrderedDict.fromkeys(build_name(b) for b in blds))
        names = ', '.join(uniq[:4]) + (', ...' if len(uniq) > 4 else '')
        return f"{len(blds)} build{'s' if len(blds) > 1 else ''} in {len(confs)} conf{'s' if len(confs) > 1 else ''}: {names}"

    print(f"\nBuilds: {len(builds)}, failed: {len(failed)}, with warnings: {len(warned)}")
    print(f"Unique errors: {len(errors)}, unique compiler warnings: {len(warnings)}, "
          f"unique generator warnings: {len(gen_warnings)}")

    # failed builds grouped by root cause, machine setup first
    by_cause = defaultdict(list)
    for b in failed:
        by_cause[root_causes.get(b, ('error', '?', 'no error message found in the log'))].append(b)
    setup = defaultdict(list)
    code_causes = []
    for cause, blds in by_cause.items():
        s = setup_issue(cause[2])
        if s:
            setup[s] += blds
        else:
            code_causes.append((cause, blds))

    if setup:
        print('\n## Build machine setup (not code bugs)\n')
        for name, blds in sorted(setup.items(), key=lambda x: -len(x[1])):
            print(f"{bullet}{name}: {where(blds)}")

    if code_causes:
        print('\n## Failed builds, by root cause\n')
        for (kind, loc, msg), blds in limit(sorted(code_causes, key=lambda x: -len(x[1]))):
            print(f"{bullet}{code(loc)}: {code(msg)}")
            print(f"      {where(blds)}")

    if missing_airframes:
        print('\n## Missing or invalid airframe files\n')
        for conf, af in missing_airframes:
            print(f"{bullet}{code(af)} in {code(conf)}")

    if warnings:
        print('\n## Compiler warnings per file, most builds first\n')
        per_file = defaultdict(lambda: ([], []))
        for (kind, loc, msg), blds in warnings.items():
            items, all_blds = per_file[loc.rsplit(':', 1)[0]]
            items.append((loc.rsplit(':', 1)[-1], msg))
            all_blds += [b for b in blds if b not in all_blds]
        for fname, (items, blds) in limit(sorted(per_file.items(), key=lambda x: -len(x[1][1]))):
            print(f"{bullet}{code(fname)}: {len(items)} warning{'s' if len(items) > 1 else ''}, {where(blds)}")
            for line, msg in sorted(items, key=lambda x: int(x[0]) if x[0].isdigit() else 0):
                print(f"      - line {line}: {code(msg)}")

    if gen_warnings:
        print('\n## Generator warnings (flight plans, airframes), most builds first\n')
        per_pattern = defaultdict(lambda: ([], []))
        for (kind, loc, msg), blds in gen_warnings.items():
            pattern = re.sub(r"'[^']*'", "'...'", re.sub(r'\d+(\.\d+)?', 'N', msg))
            msgs, all_blds = per_pattern[pattern]
            msgs.append(msg)
            all_blds += [b for b in blds if b not in all_blds]
        for pattern, (msgs, blds) in limit(sorted(per_pattern.items(), key=lambda x: -len(x[1][1]))):
            example = f" (e.g. {code(msgs[0])})" if len(msgs) > 1 else ''
            print(f"{bullet}{code(pattern if len(msgs) > 1 else msgs[0])}: {len(msgs)}x{example}")
            print(f"      {where(blds)}")

    return failed, errors, warnings


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('log_file', nargs='?', default='./var/compile.log')
    parser.add_argument('--markdown', action='store_true', help='print checklists for a GitHub issue')
    parser.add_argument('--max', type=int, default=0, help='max items per section (0: all)')
    args = parser.parse_args()

    print('=========================')
    print('||      SUMMARY        ||')
    print('=========================')
    builds, issues, root_causes, missing = parse_log(args.log_file)
    failed, errors, warnings = print_report(builds, issues, root_causes, missing, args.markdown, args.max)
    print('')
    ok = Builder.create(1).ok
    if not failed and not errors and not warnings and not missing:
        ok(True, ' - everything looks OK!')
    else:
        ok(False, f'- {len(failed)} failed builds, {len(errors)} unique errors, {len(warnings)} unique warnings')
