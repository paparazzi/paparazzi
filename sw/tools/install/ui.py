"""The installer window (zenity) and the menu in the terminal (curses), used by install.py.

  choose_in_window(inst, comps)      choose the components and where the Python packages go, in windows
  choose_in_terminal(inst, comps)    the same in a menu in the terminal
  LogWindow                          a window that shows the output while installing (no terminal)

They only show and ask: install.py knows the components (inst.info) and installs. Python standard library
only; zenity is on every Ubuntu desktop.
"""

import os
import subprocess

from install import PY_MODES

TITLE = 'Paparazzi installer'


class MenuError(Exception):
    """The menu does not work in this terminal."""


# ── Menu in the terminal ──────────────────────────────────────────────────────

HELP = 'up/down move · space select · a all · n none · left/right Python packages · enter install · q quit'


def menu(scr, inst, comps, sel, mode):
    """The curses loop. Returns the Python mode to install with, or None to quit. `sel` is changed in place."""
    import curses
    curses.curs_set(0)
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    modes = list(PY_MODES)
    n = len(comps)
    cur = 0
    while True:
        scr.erase()
        width = scr.getmaxyx()[1] - 1

        def line(y, text, attr=0):
            try:
                scr.addnstr(y, 0, text, width, attr)
            except curses.error:     # terminal too small
                pass

        line(0, '  Paparazzi installer — %s' % inst.os_name, curses.color_pair(1) | curses.A_BOLD)
        line(1, '  ' + HELP, curses.A_DIM)
        for i, c in enumerate(comps):
            box = '[-]' if c.reason else '[x]' if sel[c.id] else '[ ]'
            mark = c.reason or ('✔ installed' if c.installed else '')
            attr = curses.A_REVERSE if i == cur else curses.A_DIM if c.reason else 0
            line(3 + i, '  %s %s %-52s %s' % ('>' if i == cur else ' ', box, c.title, mark), attr)
        choice = '  '.join(('(•) ' if m == mode else '( ) ') + m for m in modes)
        line(4 + n, '  %s Python packages: %s' % ('>' if cur == n else ' ', choice), curses.A_REVERSE if cur == n else 0)
        line(5 + n, '      ' + PY_MODES[mode], curses.A_DIM)
        if cur < n:
            line(7 + n, '  ' + comps[cur].about, curses.color_pair(2))
        if inst.warning:
            line(9 + n, '  ' + inst.warning, curses.color_pair(2))
        scr.refresh()

        key = scr.getch()
        step = 0
        if key in (curses.KEY_UP, ord('k')):
            cur = (cur - 1) % (n + 1)
        elif key in (curses.KEY_DOWN, ord('j')):
            cur = (cur + 1) % (n + 1)
        elif key in (curses.KEY_RIGHT, ord('l'), ord('p')):
            step = 1
        elif key in (curses.KEY_LEFT, ord('h')):
            step = -1
        elif key == ord(' '):
            if cur == n:
                step = 1
            elif not comps[cur].reason:
                sel[comps[cur].id] = not sel[comps[cur].id]
        elif key == ord('a'):
            for c in comps:
                sel[c.id] = not c.reason
        elif key == ord('n'):
            for c in comps:
                sel[c.id] = False
        elif key in (ord('q'), ord('Q'), 27):
            return None
        elif key in (curses.KEY_ENTER, 10, 13):
            return mode
        if step:
            mode = modes[(modes.index(mode) + step) % len(modes)]


def choose_in_terminal(inst, comps):
    """The menu in the terminal. Returns (component ids, Python mode), or None to quit."""
    import curses    # here, not at the top: the windows don't need it
    sel = {c.id: c.default and c.state == '-' for c in comps}
    os.environ.setdefault('ESCDELAY', '25')    # quit with Esc without waiting a second
    try:
        mode = curses.wrapper(menu, inst, comps, sel, inst.py_mode)
    except curses.error as error:
        raise MenuError('The menu does not work in this terminal (%s): ./install.sh --yes <components>, see '
                        './install.sh --list' % error)
    if mode is None:
        return None
    return [cid for cid, on in sel.items() if on], mode


# ── Windows (zenity) ──────────────────────────────────────────────────────────

def zenity(*args):
    """Run zenity, return its output, or None when the window was closed or Cancel pressed."""
    p = subprocess.run(['zenity', '--title=' + TITLE] + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.DEVNULL, text=True)
    return p.stdout.strip() if p.returncode == 0 else None


def choose_in_window(inst, comps):
    """Choose the components and the Python mode in windows. Returns (component ids, Python mode), or None."""
    rows, skipped = [], ''
    by_id = {c.id: c for c in comps}
    for c in comps:
        if c.reason:
            skipped += '\n  - %s: %s' % (c.title, c.reason)
            continue
        selected = c.default and c.state == '-'
        rows += ['TRUE' if selected else 'FALSE', c.id, c.title, 'installed' if c.installed else '', c.about]
    text = 'Choose what to install on %s. Installed parts can be selected again to update them.' % inst.os_name
    if inst.warning:
        text += '\n' + inst.warning
    if skipped:
        text += '\nNot available here:' + skipped
    out = zenity('--list', '--checklist', '--width=1100', '--height=620', '--text=' + text, '--column=', '--column=id',
                 '--column=Component', '--column=State', '--column=Description', '--hide-column=2',
                 '--print-column=2', '--separator= ', *rows)
    if not out:
        return None
    ids = out.split()
    mode = inst.py_mode
    if any(by_id[cid].python for cid in ids):
        rows = []
        for m, description in PY_MODES.items():
            rows += ['TRUE' if m == mode else 'FALSE', m, description]
        mode = zenity('--list', '--radiolist', '--width=700', '--height=300', '--text=Where should the Python '
                      'packages go?', '--column=', '--column=Mode', '--column=Description', *rows)
        if mode is None:
            return None
    titles = ''.join('\n  - ' + by_id[cid].title for cid in ids)
    if zenity('--question', '--width=600', '--text=Install:%s\n\nPython packages: %s. Every command is shown while '
              'it runs.' % (titles, mode or inst.py_mode)) is None:
        return None
    return ids, mode or inst.py_mode


class LogWindow:
    """A zenity window that shows the output while installing (a sink for install.py's echo)."""

    def __init__(self, title):
        self.proc = subprocess.Popen(['zenity', '--text-info', '--auto-scroll', '--width=1000', '--height=650',
                                      '--title=' + title], stdin=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def write(self, text):
        self.proc.stdin.write(text)

    def flush(self):
        self.proc.stdin.flush()

    def close(self):
        """Leave the window open until the user closes it."""
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        self.proc.wait()
