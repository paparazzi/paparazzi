#!/usr/bin/env python3
"""
Live link statistics per aircraft: where frames get lost between the autopilot and the GCS.

Combines, from the Ivy bus:
  DATALINK_REPORT  the autopilot (frames it sent, uplink frames it got)
  LINK_STATS       every node on the path, by its hop from the autopilot (e.g. pprzlink_router: Herelink air
                   unit 1, handheld 2); a LINK_STATS with hop 0 replaces DATALINK_REPORT
  LINK_REPORT      Paparazzi's link on this laptop
and shows per leg between two neighbours (AP-1, 1-2, 2-laptop) and direction the frames sent and received per
second, the loss, the frames rejected (errors) or not sent, and the longest time without a frame. Without
nodes in between (e.g. a Bebop on Wi-Fi) the autopilot and link are compared directly. The signal of this
laptop's Wi-Fi is shown when the uplink goes over it (Linux).

Columns: sent/s and got/s at both ends of the leg, lost, reject (frames with a bad checksum or length at the
receiving end), unsent (frames the sending end could not send) and the longest time without a frame.

Usage: link_stats.py [-b IVY_BUS] [-w WINDOW_S] [--duration S] [--ac AC_ID]
"""
import argparse
import os
import subprocess
import sys
import threading
import time
from collections import defaultdict, deque

from ivy.std_api import IvyInit, IvyStart, IvyBindMsg, IvyStop

DEFAULT_BUS = "224.255.255.255:2010" if sys.platform == "darwin" else "127.255.255.255:2010"

DATALINK_REPORT = ["uplink_lost_time", "uplink_nb_msgs", "downlink_nb_msgs", "downlink_rate", "uplink_rate",
                   "downlink_ovrn", "uplink_err", "uplink_ovrn"]
LINK_STATS = ["hop", "uptime", "down_rx", "down_rx_err", "down_tx", "down_tx_err", "up_rx", "up_rx_err", "up_tx",
              "up_tx_err", "down_max_gap", "up_max_gap", "nb_peers", "dropped", "rssi"]
LINK_REPORT = ["ac_id", "link_id", "run_time", "rx_lost_time", "rx_bytes", "rx_msgs", "rx_err", "rx_bytes_rate",
               "rx_msgs_rate", "tx_msgs", "ping_time", "tx_err", "peer", "rx_max_gap"]
# counters that wrap (number of bits)
WRAP_DATALINK = {"uplink_nb_msgs": 16, "downlink_nb_msgs": 16, "downlink_ovrn": 8, "uplink_err": 8, "uplink_ovrn": 8}
WRAP_STATS = {k: 16 for k in LINK_STATS[2:10] + ["dropped"]}
# generic counter names of a path end -> field of its report
KEYS_DATALINK = {"down_tx": "downlink_nb_msgs", "down_tx_err": "downlink_ovrn", "up_rx": "uplink_nb_msgs",
                 "up_rx_err": "uplink_err"}
KEYS_LINK = {"down_rx": "rx_msgs", "down_rx_err": "rx_err", "down_max_gap": "rx_max_gap", "up_tx": "tx_msgs",
             "up_tx_err": "tx_err"}
KEYS_STATS = {k: k for k in LINK_STATS}
UNKNOWN = 127
NODE_TIMEOUT = 60.0  # s: a node not heard for longer leaves the table (until then its legs show "-")


def to_num(v):
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


class Source:
    """History of the counters of one report source (autopilot, a node on the path, link)"""

    def __init__(self, window, wrap=None):
        self.window = window
        self.wrap = wrap or {}
        self.hist = deque()
        self.last = None
        self.count = 0  # reports received

    def add(self, values):
        now = time.time()
        if self.last is not None and "uptime" in values and values["uptime"] < self.last.get("uptime", 0):
            self.hist.clear()  # the node restarted (or its uptime wrapped after 18 h)
        self.hist.append((now, values))
        self.last = values
        self.count += 1
        # keep one report older than the window: slow reports (DATALINK_REPORT every 5 s) still give a rate
        while len(self.hist) > 2 and now - self.hist[1][0] > self.window:
            self.hist.popleft()

    def period(self):
        """Time between two reports (None if unknown)"""
        if len(self.hist) < 2:
            return None
        return (self.hist[-1][0] - self.hist[0][0]) / (len(self.hist) - 1)

    def age(self):
        return time.time() - self.hist[-1][0] if self.hist else None

    def fresh(self):
        """A report came in the last 3 s, or in the last 2.5 report periods for the slow ones"""
        if not self.hist:
            return False
        return self.age() < max(3.0, 2.5 * (self.period() or 5.0))  # one report only: period unknown

    def delta(self, key):
        """Increase of a counter over the window (None if unknown)"""
        if len(self.hist) < 2 or key not in self.hist[0][1] or key not in self.hist[-1][1]:
            return None
        d = self.hist[-1][1][key] - self.hist[0][1][key]
        if key in self.wrap:
            d %= (1 << self.wrap[key])
        return d if d >= 0 else None

    def rate(self, key):
        d = self.delta(key)
        if d is None:
            return None
        return d / (self.hist[-1][0] - self.hist[0][0])

    def report_rate(self):
        if len(self.hist) < 2:
            return None
        return (len(self.hist) - 1) / (self.hist[-1][0] - self.hist[0][0])

    def max(self, key):
        """Largest value over the window (the report kept from before the window not included)"""
        now = time.time()
        vals = [v[key] for t, v in self.hist if key in v and now - t <= self.window]
        return max(vals) if vals else None


class Stats:
    def __init__(self, window):
        self.window = window
        self.lock = threading.Lock()
        self.ac = defaultdict(lambda: {"ap": Source(window, WRAP_DATALINK), "nodes": {}, "link": Source(window)})

    def on_datalink_report(self, _agent, ac_id, fields):
        vals = dict(zip(DATALINK_REPORT, [to_num(v) for v in fields.split()]))
        with self.lock:
            self.ac[int(ac_id)]["ap"].add(vals)

    def on_link_stats(self, _agent, ac_id, fields):
        vals = dict(zip(LINK_STATS, [to_num(v) for v in fields.split()]))
        with self.lock:
            nodes = self.ac[int(ac_id)]["nodes"]
            nodes.setdefault(vals["hop"], Source(self.window, WRAP_STATS)).add(vals)

    def on_link_report(self, _agent, fields):
        vals = dict(zip(LINK_REPORT, [to_num(v) for v in fields.split()]))
        with self.lock:
            self.ac[int(vals["ac_id"])]["link"].add(vals)


class End:
    """One end of a leg: the counters of a report source under the generic names of KEYS_STATS"""

    def __init__(self, name, src, keys):
        self.name, self.src, self.keys = name, src, keys
        self.fresh = src.fresh()

    def _key(self, k):
        return self.keys.get(k) if self.fresh else None

    def rate(self, k):
        key = self._key(k)
        r = self.src.rate(key) if key else None
        if r == 0 and self.keys is KEYS_DATALINK and k == "down_tx":
            return None  # firmware that does not count its frames (pprzlink before link-stats)
        return r

    def delta(self, k):
        key = self._key(k)
        return self.src.delta(key) if key else None

    def max(self, k):
        key = self._key(k)
        return self.src.max(key) if key else None

    def last(self, k):
        key = self._key(k)
        return (self.src.last or {}).get(key) if key else None


def chain(s):
    """The ends along the path: autopilot, the nodes by hop, this laptop"""
    nodes = s["nodes"]
    for hop in [h for h, src in nodes.items() if src.age() > NODE_TIMEOUT]:
        del nodes[hop]
    if 0 in nodes:
        ends = [End("AP", nodes[0], KEYS_STATS)]
    else:
        ends = [End("AP", s["ap"], KEYS_DATALINK)]
    ends += [End(str(h), nodes[h], KEYS_STATS) for h in sorted(nodes) if h > 0]
    ends.append(End("laptop", s["link"], KEYS_LINK))
    return ends


def fmt_rate(r):
    return "      -" if r is None else "%7.1f" % r


def fmt_loss(sent, got):
    if sent is None or got is None or sent <= 0:
        return "     -"
    return "%5.1f%%" % max(0.0, 100.0 * (1.0 - got / sent))


def fmt_count(c):
    return "     -" if c is None else "%6d" % c


def fmt_gap(g):
    if g is None:
        return "      -"
    if g >= 65535 and g < 65536:
        return "  >65 s"  # the routers' counter is full
    return "%4d ms" % g if g < 10000 else "%5.0f s" % (g / 1000.0)


_wifi_cache = {}


def wifi_signal(peer):
    """Signal (dBm) of this laptop's Wi-Fi when the uplink to peer ("ip:port") goes over Wi-Fi (Linux), else None"""
    ip = peer.rsplit(":", 1)[0]
    now = time.time()
    if ip in _wifi_cache and now - _wifi_cache[ip][0] < 1.0:
        return _wifi_cache[ip][1]
    level = None
    try:
        route = subprocess.run(["ip", "route", "get", ip], capture_output=True, text=True, timeout=1).stdout.split()
        dev = route[route.index("dev") + 1] if "dev" in route else None
        with open("/proc/net/wireless") as f:
            for line in f.readlines()[2:]:
                name, vals = line.split(":", 1)
                if name.strip() == dev:
                    level = int(float(vals.split()[2]))
                    level = level - 256 if level > 0 else level  # old drivers: unsigned dBm
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        pass
    _wifi_cache[ip] = (now, level)
    return level


def legs(ends):
    """Rows: (direction, leg, sent/s, got/s, rejected, not sent, max gap)"""
    rows = []
    for a, b in zip(ends, ends[1:]):
        sent = a.rate("down_tx")
        if b.name == "laptop" and a.keys is KEYS_STATS and a.fresh:
            # each laptop gets every frame the node got, plus the node's own report
            r, rep = a.rate("down_rx"), a.src.report_rate()
            sent = None if r is None else r + (rep or 0)
        rows.append(("down", "%s-%s" % (a.name, b.name), sent, b.rate("down_rx"), b.delta("down_rx_err"),
                     a.delta("down_tx_err"), b.max("down_max_gap")))
    for a, b in reversed(list(zip(ends, ends[1:]))):
        sent = b.rate("up_tx")
        if b.name == "laptop" and (a.last("nb_peers") or 0) > 1:
            sent = None  # only this laptop's uplink is known
        rows.append(("up", "%s-%s" % (b.name, a.name), sent, a.rate("up_rx"), a.delta("up_rx_err"),
                     b.delta("up_tx_err"), a.max("up_max_gap")))
    return rows


def render(stats, only_ac=None):
    out = ["Link statistics %s (over the last %.0f s)" % (time.strftime("%H:%M:%S"), stats.window)]
    with stats.lock:
        for ac_id in sorted(stats.ac):
            if only_ac is not None and ac_id != only_ac:
                continue
            s = stats.ac[ac_id]
            ends = chain(s)
            link = s["link"].last or {}
            info = ["A/C %d" % ac_id]
            if link:
                peer = str(link.get("peer", "?"))
                info.append("uplink to %s" % peer)
                ping = link.get("ping_time", -1)
                info.append(("ping %.0f ms" % ping) if 0 <= ping < 60000 else "no ping reply yet")
                level = wifi_signal(peer) if ":" in peer else None
                if level is not None:
                    info.append("laptop Wi-Fi %d dBm" % level)
            for e in ends[:-1]:
                if e.keys is not KEYS_STATS or not e.fresh:
                    continue
                node = []
                if e.last("rssi") not in (None, UNKNOWN):
                    node.append("%d dBm" % e.last("rssi"))
                if e.last("nb_peers"):
                    node.append("%d laptops" % e.last("nb_peers"))
                if e.delta("dropped"):
                    node.append("%d dropped" % e.delta("dropped"))
                if node:
                    info.append("hop %s: %s" % (e.name, ", ".join(node)))
            out.append("")
            out.append(", ".join(info))
            out.append("%-17s %7s %7s %6s  %6s   %6s  %7s" % ("", "sent/s", "got/s", "lost", "reject", "unsent", "max gap"))
            for d, leg, sent, got, rej, nots, gap in legs(ends):
                out.append("%-4s %-12s %s %s %s  %s   %s  %s" % (d, leg, fmt_rate(sent), fmt_rate(got),
                                                               fmt_loss(sent, got), fmt_count(rej), fmt_count(nots),
                                                               fmt_gap(gap)))
            # always one note line per A/C, so the screen does not jump
            note = ""
            lost = link.get("rx_lost_time", 0) if link else 0
            if isinstance(lost, (int, float)) and lost >= 3:
                note = "(nothing heard from the A/C for %d s)" % lost
            elif 0 in s["nodes"]:
                pass  # the autopilot sends LINK_STATS
            elif s["ap"].count == 0:
                if s["link"].count >= 15:  # the link reports every second: DATALINK_REPORT is every 5 s or so
                    note = "(no DATALINK_REPORT from the autopilot: add it to the telemetry)"
            elif not s["ap"].fresh():
                note = "(last DATALINK_REPORT %.0f s ago)" % s["ap"].age()
            out.append(note)
    if len(out) == 1:
        out.append("no aircraft yet")
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-b", "--bus", default=os.getenv("IVYBUS", DEFAULT_BUS), help="Ivy bus [%(default)s]")
    parser.add_argument("-w", "--window", type=float, default=10.0, help="averaging window in s [%(default)s]")
    parser.add_argument("--duration", type=float, default=None, help="print once after this time and stop")
    parser.add_argument("--ac", type=int, default=None, help="only this aircraft")
    args = parser.parse_args()

    stats = Stats(args.window)
    IvyInit("link_stats", "link_stats READY")
    IvyStart(args.bus)
    IvyBindMsg(stats.on_datalink_report, r"^(\d+) DATALINK_REPORT (.*)$")
    IvyBindMsg(stats.on_link_stats, r"^(\d+) LINK_STATS (.*)$")
    IvyBindMsg(stats.on_link_report, r"^link LINK_REPORT (.*)$")

    try:
        if args.duration is not None:
            time.sleep(args.duration)
            print(render(stats, args.ac))
        elif not sys.stdout.isatty():
            while True:  # piped or logged: one block per second
                time.sleep(1)
                print(render(stats, args.ac) + "\n", flush=True)
        else:
            # alternate screen (like top): redrawn in place, nothing added to the terminal history
            sys.stdout.write("\033[?1049h\033[?25l")
            try:
                while True:
                    text = render(stats, args.ac)
                    # home, each line cleared to its end, then the rest of the screen cleared
                    sys.stdout.write("\033[H" + "\033[K\n".join(text.split("\n")) + "\033[K\n\033[J")
                    sys.stdout.flush()
                    time.sleep(1)
            finally:
                sys.stdout.write("\033[?25h\033[?1049l")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        IvyStop()


if __name__ == "__main__":
    main()
