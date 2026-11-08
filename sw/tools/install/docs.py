#!/usr/bin/env python3
"""The documentation of the installer components, made from the recipe (components.txt) by the installer itself.

For each Ubuntu version, install.py (as with --os) gives the recipe lines that apply there, why a component is
not available, and the commands of its steps. This file only formats them:

  docs.py --doc               reStructuredText for Sphinx (doc/sphinx/source/conf.py includes it in install.rst)
  docs.py --doxygen[=DIR]     the Doxygen page install_components (DIR/install.dox, default doc/manual/generated)
  docs.py --wiki              MediaWiki text, to paste in the wiki
  docs.py --requirements      requirements.txt (pip) of the python component

Also as ./install.sh --doc, --doxygen, --wiki, --requirements. Python standard library only.
"""

import os
import sys

import install

VERSIONS = ['22.04', '24.04', '26.04']
PPRZLINK = 'pip install -e sw/ext/pprzlink/lib/v2.0/python'


def describe(version):
    """{component id: [(key, value), ...]} on a fresh Ubuntu `version`, in the order of the recipe."""
    return install.Installer(os_arg=version).describe()


def first(lines, key):
    return next((v for k, v in lines if k == key), '')


def items(lines):
    """What a component installs: (label, [words]) and ('command', [one command]) items, or a 'not available'."""
    if first(lines, 'unavailable'):
        return [('not available', [first(lines, 'unavailable')])]
    result = []
    repos = [v.replace('osrf', 'packages.osrfoundation.org (Gazebo)') for k, v in lines if k == 'repo']
    apt = [w for k, v in lines if k == 'apt' for w in v.split()]
    py = [v.split() for k, v in lines if k == 'py']
    for label, words in (('repository', repos), ('apt', apt), ('Python, apt', [p[1] for p in py if p[1] != '-']),
                         ('Python, pip only', [p[2] for p in py if p[1] == '-'])):
        if words:
            result.append((label, words))
    if ('step', 'pprzlink') in lines:
        result.append(('venv or conda, the pprzlink library', [PPRZLINK]))
    result += [('command', [v]) for k, v in lines if k == 'command']
    return result


def grouped(per_version, cid):
    """[(versions, items)] with the Ubuntu versions that install the same together."""
    groups = []
    for version, comps in per_version:
        its = items(comps[cid])
        for vers, other in groups:
            if other == its:
                vers.append(version)
                break
        else:
            groups.append(([version], its))
    return groups


def all_versions():
    per_version = [(v, describe(v)) for v in VERSIONS]
    return per_version, list(per_version[-1][1])


def rst():
    per_version, ids = all_versions()
    out = []
    for cid in ids:
        lines = per_version[-1][1][cid]
        head = '``%s``: %s' % (cid, first(lines, 'title'))
        out += [head, '~' * len(head), '', first(lines, 'about')]
        needs = first(lines, 'needs').split()
        if needs:
            out.append('Needs: %s.' % ' '.join('``%s``' % n for n in needs))
        out.append('')
        for vers, its in grouped(per_version, cid):
            out += ['- **Ubuntu %s**' % ', '.join(vers), '']
            for label, words in its or [('nothing to install', [])]:
                if label in ('not available', 'nothing to install'):
                    out.append('  - %s%s' % (label, ': ' + words[0] if words else ''))
                else:
                    out.append('  - %s: %s' % (label, ' '.join('``%s``' % w for w in words)))
            out.append('')
    return '\n'.join(out) + '\n'


def html(words):
    """Words for Doxygen and the wiki, with < > & escaped (empy<=3.3.4)."""
    return ' '.join(words).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def doxygen():
    per_version, ids = all_versions()
    out = ['/** @page install_components Installation: what each component installs', '',
           'Made from sw/tools/install/components.txt (the recipe of ./install.sh) by sw/tools/install/docs.py.', '']
    for cid in ids:
        lines = per_version[-1][1][cid]
        out += ['@section install_%s %s: %s' % (cid, cid, first(lines, 'title')), '', first(lines, 'about'), '']
        if first(lines, 'needs'):
            out += ['Needs: %s.' % first(lines, 'needs'), '']
        for vers, its in grouped(per_version, cid):
            out.append('- <b>Ubuntu %s</b>' % ', '.join(vers))
            for label, words in its or [('nothing to install', [])]:
                out.append('  - %s%s' % (label, ': <tt>%s</tt>' % html(words) if words else ''))
        out.append('')
    return '\n'.join(out) + '\n*/\n'


def wiki():
    per_version, ids = all_versions()
    out = ['Made from sw/tools/install/components.txt by <code>./install.sh --wiki</code>: change the recipe, '
           'not this page.', '']
    for cid in ids:
        lines = per_version[-1][1][cid]
        out += ['== %s: %s ==' % (cid, first(lines, 'title')), first(lines, 'about'), '']
        for vers, its in grouped(per_version, cid):
            out.append("* '''Ubuntu %s'''" % ', '.join(vers))
            for label, words in its or [('nothing to install', [])]:
                out.append('** %s%s' % (label, ': <code>%s</code>' % html(words) if words else ''))
        out.append('')
    return '\n'.join(out)


def requirements():
    """The pip requirements of the python component on every Ubuntu version, in the order of the recipe."""
    reqs = []
    for version in VERSIONS:
        for k, v in describe(version)['python']:
            if k == 'py' and v.split()[2] != '-' and v.split()[2] not in reqs:
                reqs.append(v.split()[2])
    return ''.join(r + '\n' for r in reqs)


def main(args):
    """Write the documentation asked by args[0] (--doc by default). Returns the exit code."""
    arg = args[0] if args else '--doc'
    if arg == '--doc':
        sys.stdout.write(rst())
    elif arg.startswith('--doxygen'):
        folder = arg.partition('=')[2] or os.path.join(install.PPRZ_DIR, 'doc', 'manual', 'generated')
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, 'install.dox'), 'w') as f:
            f.write(doxygen())
        print('wrote', os.path.join(folder, 'install.dox'))
    elif arg == '--wiki':
        sys.stdout.write(wiki())
    elif arg == '--requirements':
        sys.stdout.write(requirements())
    else:
        print(__doc__, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
