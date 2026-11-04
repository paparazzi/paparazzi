#!/usr/bin/env python3
"""
Live link statistics per aircraft: where frames get lost between the autopilot and the GCS.

Combines, from the Ivy bus:
  DATALINK_REPORT     the autopilot (frames it sent, uplink frames it got)
  LINK_ROUTER_REPORT  pprzlink_router on the Herelink air unit (AIR) and handheld (GROUND)
  LINK_REPORT         Paparazzi's link on this laptop
and shows per leg (UART, Herelink radio, Wi-Fi) and direction the frames sent and received per second, the
loss, the frames rejected (errors) or not sent, and the longest time without a frame. Without the routers
(e.g. a Bebop on Wi-Fi) the autopilot and link are compared directly, and the signal of this laptop's Wi-Fi
is shown when the uplink goes over it (Linux).

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
ROUTER_REPORT = ["node", "uptime", "down_rx_msgs", "down_rx_err", "down_tx_msgs", "down_tx_err", "up_rx_msgs",
                 "up_rx_err", "up_tx_msgs", "up_tx_err", "up_dropped", "down_max_gap", "up_max_gap", "nb_gcs",
                 "rssi", "snr"]
LINK_REPORT = ["ac_id", "link_id", "run_time", "rx_lost_time", "rx_bytes", "rx_msgs", "rx_err", "rx_bytes_rate",
               "rx_msgs_rate", "tx_msgs", "ping_time", "tx_err", "peer", "rx_max_gap"]
# counters that wrap (number of bits)
WRAP = {"uplink_nb_msgs": 16, "downlink_nb_msgs": 16, "downlink_ovrn": 8, "uplink_err": 8, "uplink_ovrn": 8}
UNKNOWN = 127


def to_num(v):
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


class Source:
    """History of the counters of one report source (autopilot, air router, ground router, link)"""

    def __init__(self, window):
        self.window = window
        self.hist = deque()
        self.last = None
        self.count = 0  # reports received

    def add(self, values):
        now = time.time()
        if self.last is not None and "uptime" in values and values["uptime"] < self.last.get("uptime", 0):
            self.hist.clear()  # the router restarted
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
        if key in WRAP:
            d %= (1 << WRAP[key])
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
        self.ac = defaultdict(lambda: {"ap": Source(window), "air": Source(window), "ground": Source(window),
                                        "link": Source(window)})

    def on_datalink_report(self, _agent, ac_id, fields):
        vals = dict(zip(DATALINK_REPORT, [to_num(v) for v in fields.split()]))
        with self.lock:
            self.ac[int(ac_id)]["ap"].add(vals)

    def on_router_report(self, _agent, ac_id, fields):
        vals = dict(zip(ROUTER_REPORT, [to_num(v) for v in fields.split()]))
        node = {0: "air", 1: "ground", "AIR": "air", "GROUND": "ground"}.get(vals["node"])
        if node:
            with self.lock:
                self.ac[int(ac_id)][node].add(vals)

    def on_link_report(self, _agent, fields):
        vals = dict(zip(LINK_REPORT, [to_num(v) for v in fields.split()]))
        with self.lock:
            self.ac[int(vals["ac_id"])]["link"].add(vals)


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


def legs(s):
    """Rows: (direction, leg, sent/s, got/s, rejected, not sent, max gap)"""
    ap, air, ground, link = s["ap"], s["air"], s["ground"], s["link"]
    has_air, has_ground = air.fresh(), ground.fresh()
    rows = []
    # Downlink
    down_sent = ap.rate("downlink_nb_msgs") if ap.fresh() else None
    if not down_sent:
        down_sent = None  # 0: firmware that does not count its frames (pprzlink before link-stats)
    down_src = (down_sent, ap.delta("downlink_ovrn") if ap.fresh() else None)
    if has_air:
        rows.append(("down", "UART", down_src[0], air.rate("down_rx_msgs"), air.delta("down_rx_err"), down_src[1],
                     air.max("down_max_gap")))
        down_src = (air.rate("down_tx_msgs"), air.delta("down_tx_err"))
    if has_ground:
        rows.append(("down", "radio" if has_air else "to handheld", down_src[0], ground.rate("down_rx_msgs"),
                     ground.delta("down_rx_err"), down_src[1], ground.max("down_max_gap")))
        # each laptop gets every frame the handheld got, plus the handheld's report (beacon)
        r, b = ground.rate("down_rx_msgs"), ground.report_rate()
        down_src = (None if r is None else r + (b or 0), ground.delta("down_tx_err"))
    rows.append(("down", "Wi-Fi" if has_ground else "to laptop", down_src[0], link.rate("rx_msgs"),
                 link.delta("rx_err"), down_src[1], link.max("rx_max_gap")))
    # Uplink (only this laptop's link is seen: with several laptops the Wi-Fi uplink loss is not known)
    nb_gcs = ground.last.get("nb_gcs") if has_ground else None
    up_sent = (link.rate("tx_msgs"), link.delta("tx_err"))
    if has_ground:
        got = ground.rate("up_rx_msgs")
        # nb_gcs 0: the handheld sends to a fixed address (one laptop)
        rows.append(("up", "Wi-Fi", up_sent[0] if nb_gcs in (0, 1) else None, got, ground.delta("up_rx_err"), up_sent[1],
                     ground.max("up_max_gap")))
        up_sent = (ground.rate("up_tx_msgs"), ground.delta("up_tx_err"))
    if has_air:
        rows.append(("up", "radio" if has_ground else "to air unit", up_sent[0], air.rate("up_rx_msgs"),
                     air.delta("up_rx_err"), up_sent[1], air.max("up_max_gap")))
        up_sent = (air.rate("up_tx_msgs"), air.delta("up_tx_err"))
    ap_err = ap.delta("uplink_err") if ap.fresh() else None
    rows.append(("up", "UART" if has_air else "to autopilot", up_sent[0],
                 ap.rate("uplink_nb_msgs") if ap.fresh() else None, ap_err,
                 up_sent[1], None))
    return rows


def render(stats, only_ac=None):
    out = ["Link statistics %s (over the last %.0f s)" % (time.strftime("%H:%M:%S"), stats.window)]
    with stats.lock:
        for ac_id in sorted(stats.ac):
            if only_ac is not None and ac_id != only_ac:
                continue
            s = stats.ac[ac_id]
            link, ground, air = s["link"].last or {}, s["ground"].last or {}, s["air"].last or {}
            info = ["A/C %d" % ac_id]
            if link:
                peer = str(link.get("peer", "?"))
                info.append("uplink to %s" % peer)
                ping = link.get("ping_time", -1)
                info.append(("ping %.0f ms" % ping) if 0 <= ping < 60000 else "no ping reply yet")
                if not s["air"].fresh() and not s["ground"].fresh() and ":" in peer:
                    level = wifi_signal(peer)
                    if level is not None:
                        info.append("laptop Wi-Fi %d dBm" % level)
            if s["ground"].fresh():
                info.append("laptops found by the handheld: %d" % ground.get("nb_gcs", 0))
                if ground.get("rssi", UNKNOWN) != UNKNOWN:
                    info.append("handheld Wi-Fi %d dBm" % ground["rssi"])
            if s["air"].fresh() and air.get("rssi", UNKNOWN) != UNKNOWN:
                info.append("radio %d dBm" % air["rssi"])
            if s["ground"].fresh() and ground.get("up_dropped"):
                info.append("uplink for other A/C dropped: %d" % ground["up_dropped"])
            out.append("")
            out.append(", ".join(info))
            out.append("%-17s %7s %7s %6s  %6s   %6s  %7s" % ("", "sent/s", "got/s", "lost", "reject", "unsent", "max gap"))
            for d, leg, sent, got, rej, nots, gap in legs(s):
                out.append("%-4s %-12s %s %s %s  %s   %s  %s" % (d, leg, fmt_rate(sent), fmt_rate(got),
                                                               fmt_loss(sent, got), fmt_count(rej), fmt_count(nots),
                                                               fmt_gap(gap)))
            # always one note line per A/C, so the screen does not jump
            note = ""
            lost = link.get("rx_lost_time", 0) if link else 0
            if isinstance(lost, (int, float)) and lost >= 3:
                note = "(nothing heard from the A/C for %d s)" % lost
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
    IvyBindMsg(stats.on_router_report, r"^(\d+) LINK_ROUTER_REPORT (.*)$")
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
