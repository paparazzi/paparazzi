#!/usr/bin/env bash
# Check that the Herelink link to Paparazzi is healthy. Only checks: it does not start or restart anything.
# Usage: ./herelink_check.sh [usb|wifi|net] [--wait SECONDS]
#   mode    usb: USB tethering (192.168.42.x), wifi: the handheld's hotspot (192.168.43.x),
#           net: the handheld and the laptop joined the same Wi-Fi network.
#           Default: detected from the laptop addresses (net if neither 42.x nor 43.x)
#   --wait  how long to wait for Paparazzi's link agent and telemetry (default 15 s, useful when the
#           check starts together with a Paparazzi session)
# Exit code 0 when everything needed for telemetry is OK.
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"

MODE=""
WAIT=15
while [ $# -gt 0 ]; do
  case "$1" in
    usb|wifi|net) MODE=$1; shift ;;
    --wait) WAIT=${2:?--wait needs seconds}; shift 2 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "Unknown argument $1 (usage: $0 [usb|wifi|net] [--wait SECONDS])"; exit 2 ;;
  esac
done

FAIL=0
ok()   { echo "[ OK ] $*"; }
bad()  { echo "[FAIL] $*"; FAIL=1; }
warn() { echo "[WARN] $*"; }
info() { echo "       $*"; }

echo "Herelink link check $(date +%H:%M:%S)"

# 1. Laptop address on a network shared with the handheld
USB_IF=$(ip -4 -br a | awk 'index($3,"192.168.42.")==1 {print $1; exit}')
WIFI_IF=$(ip -4 -br a | awk 'index($3,"192.168.43.")==1 {print $1; exit}')
NET_IF=$(ip -4 -br a | awk '$1 != "lo" && $2 == "UP" && $3 != "" {print $1; exit}')
if [ -z "$MODE" ]; then
  if [ -n "$USB_IF" ]; then MODE=usb; elif [ -n "$WIFI_IF" ]; then MODE=wifi; elif [ -n "$NET_IF" ]; then MODE=net; fi
fi
case "$MODE" in
  usb)  IFACE=$USB_IF;  HL_IP=192.168.42.129 ;;
  wifi) IFACE=$WIFI_IF; HL_IP=192.168.43.1 ;;
  net)  IFACE=$NET_IF;  HL_IP="" ;;   # known from link once the drone is found
  *)    IFACE="" ;;
esac
if [ -n "$IFACE" ]; then
  ok "laptop on the $MODE network: $IFACE $(ip -4 -br a show "$IFACE" | awk '{print $3}')"
else
  bad "laptop has no network address for mode '$MODE'"
  info "USB: turn on USB tethering on the handheld and connect the new USB device to the VM (VMware Fusion)"
  info "Wi-Fi: turn on the hotspot on the handheld and join it, or join the same Wi-Fi network as the handheld"
fi
if [ -n "$USB_IF" ] && [ -n "$WIFI_IF" ]; then
  warn "laptop is on both USB tethering and the handheld's hotspot: it may get every frame twice"
  info "use one of them"
fi

# 2. Handheld reachable
ping_handheld() {
  if ping -c 2 -W 1 "$1" >/dev/null 2>&1; then ok "handheld answers ping ($1)"
  else bad "no ping reply from the handheld ($1)"; fi
}
[ -n "$IFACE" ] && [ -n "$HL_IP" ] && ping_handheld "$HL_IP"

# 3. Router on the handheld (needs adb, i.e. the USB cable)
SER=$(adb devices -l 2>/dev/null | awk '/optimus/ {print $1; exit}')
if [ -z "$SER" ]; then
  warn "handheld not on adb: router state not checked (fine on Wi-Fi without cable)"
else
  PS=$(adb -s "$SER" shell ps 2>/dev/null | grep -E 'pprzlink_router' | tr -d '\r')
  APP=$(echo "$PS" | grep -c 'files/pprzlink_router$')
  OTHER=$(echo "$PS" | grep -E 'pprzlink_router' | grep -vc 'files/pprzlink_router$')
  if [ "$APP" -ge 1 ] && [ "$OTHER" = 0 ]; then
    ok "router running, started by the pprzlink router app"
  elif [ "$APP" -ge 1 ]; then
    bad "two routers on the handheld: the app's and an old one started over adb"
    info "stop the old one: adb shell 'kill \$(pidof pprzlink_router_cc pprzlink_router_test)'"
  elif [ "$OTHER" -ge 1 ]; then
    warn "router running, but not the app's one (old adb start: no keep-awake, no discovery)"
  elif adb -s "$SER" shell pm list packages 2>/dev/null | grep -q org.paparazziuav.pprzlinkrouter; then
    bad "app installed but no router running: start it once with"
    info "adb shell am broadcast -n org.paparazziuav.pprzlinkrouter/.BootReceiver -f 32"
  else
    bad "no router and no app on the handheld: install the app (see pprzlink_router.md, Herelink ground unit)"
  fi

  if [ "$APP" -ge 1 ]; then
    LAST=$(adb -s "$SER" shell logcat -d -s pprzlink_router 2>/dev/null | grep 'Started the router to' | tail -1 | tr -d '\r')
    TO=${LAST##* }
    if [ "$TO" = "auto" ]; then ok "router finds the laptops itself (app 1.1 or newer)"
    elif [ -n "$TO" ]; then warn "old app: the router broadcasts to $TO only. Update the app (pprzlink_router.md)"
    else warn "could not read how the router was started"; fi

    PID=$(adb -s "$SER" shell pidof pprzlink_router 2>/dev/null | tr -d '\r' | awk '{print $1}')
    NICE19=$(adb -s "$SER" shell "for t in /proc/$PID/task/*; do cut -d' ' -f19 \$t/stat; done" 2>/dev/null | tr -d '\r' | grep -c '^19$')
    if [ "$NICE19" -ge 1 ]; then ok "keep-awake active (uplink holds fix)"
    else warn "keep-awake not active (router started with -n?)"; fi

    if adb -s "$SER" shell dumpsys package org.paparazziuav.pprzlinkrouter 2>/dev/null | grep -q "READ_LOGS: granted=true"; then
      ok "handheld radio signal reported (READ_LOGS granted)"
    else
      warn "handheld radio signal not reported: update with herelink_install.sh (app 1.2, grants READ_LOGS)"
    fi
  fi

  # Battery and versions of the handheld, and of the air unit when it is on adb too
  BAT=$(adb -s "$SER" shell dumpsys battery 2>/dev/null | awk '/  level:/ {print $2}' | tr -d '\r')
  if [ -n "$BAT" ] && [ "$BAT" -lt 20 ]; then warn "handheld battery $BAT%"; elif [ -n "$BAT" ]; then ok "handheld battery $BAT%"; fi
  APPV=$(adb -s "$SER" shell dumpsys package org.paparazziuav.pprzlinkrouter 2>/dev/null | awk -F= '/versionName/ {print $2; exit}' | tr -d '\r')
  GBB=$(adb -s "$SER" shell getprop gsm.version.baseband 2>/dev/null | tr -d '\r')
  info "handheld: app ${APPV:-none}, radio firmware ${GBB:-?}"
  AIR=$(adb devices -l 2>/dev/null | awk '/telepathy-air/ && $2 == "device" {print $1; exit}')
  if [ -n "$AIR" ]; then
    ABB=$(adb -s "$AIR" shell getprop gsm.version.baseband 2>/dev/null | tr -d '\r')
    AFW=$(adb -s "$AIR" shell getprop ro.build.display.id 2>/dev/null | tr -d '\r' | awk '{print $4}')
    info "air unit: firmware ${AFW:-?}, radio firmware ${ABB:-?}"
    # the radio firmware build (e.g. R03.T34) of both units should be the same release
    if [ -n "$GBB" ] && [ -n "$ABB" ] && [ "${GBB%%-*}" != "${ABB%%-*}" ]; then
      warn "the air unit and the handheld have different radio firmware (${ABB%%-*} / ${GBB%%-*}): update both to the same Herelink release"
    fi
  fi
fi

# 4. Paparazzi link agent and telemetry
LINKCMD=""
for i in $(seq 1 "$WAIT"); do
  LINKCMD=$(pgrep -af 'ground_segment/tmtc/link( |$)' | head -1)
  [ -n "$LINKCMD" ] && break
  sleep 1
done
if [ -z "$LINKCMD" ]; then
  bad "Paparazzi link agent not running: start 'Data Link' with -udp in Paparazzi Center"
else
  case "$LINKCMD" in
    *-udp*) ok "link agent running with -udp" ;;
    *)      bad "link agent runs without -udp (it must listen to the Herelink on UDP): $LINKCMD" ;;
  esac
  REP=$(python3 - "$WAIT" <<'EOF' 2>/dev/null
import sys, time
from ivy.std_api import IvyInit, IvyStart, IvyBindMsg, IvyStop
rep, ground = [], []
IvyInit("herelink_check", "herelink_check READY")
IvyStart("127.255.255.255:2010")
# ac_id link_id run_time rx_lost_time rx_bytes rx_msgs rx_err rx_bytes_rate rx_msgs_rate tx_msgs ping_time [tx_err peer ...]
IvyBindMsg(lambda a, *g: rep.append(g), r"^link LINK_REPORT (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)(?: (\S+) (\S+))?(?: .*)?$")
# LINK_STATS of the handheld router (hop 2): down_rx ... nb_peers
IvyBindMsg(lambda a, *g: ground.append((time.time(), g)), r"^\d+ LINK_STATS (2) \S+ (\S+)(?: \S+){9} (\S+)")
t0 = time.time()
while time.time() - t0 < max(float(sys.argv[1]), 4.) and len(rep) < 4:
    time.sleep(0.2)
IvyStop()
if len(rep) >= 2:
    a, b = rep[0], rep[-1]
    dt = max(float(b[2]) - float(a[2]), 1.)
    peer = b[12] if b[12] else "?"
    # telemetry the handheld got per second, to find laptops that get every frame twice
    grx = (((int(ground[-1][1][1]) - int(ground[0][1][1])) % 65536) / (ground[-1][0] - ground[0][0])) if len(ground) >= 2 else -1
    gcs = ground[-1][1][2] if ground else "-1"
    print(b[0], float(b[8]), int(b[6]) - int(a[6]), (int(b[9]) - int(a[9])) / dt, float(b[10]), int(b[3]), peer, grx, gcs)
EOF
)
  if [ -z "$REP" ]; then
    bad "no telemetry: link reports no aircraft (is the drone on? is the router sending to the laptop?)"
  else
    read -r AC RXR ERR TXR PING LOST PEER GRX GCS <<< "$REP"
    if awk "BEGIN{exit !($RXR > 1)}" && [ "$LOST" -lt 3 ]; then
      ok "telemetry from aircraft $AC: $(printf '%.0f' "$RXR") msgs/s received, $ERR errors, $(printf '%.0f' "$TXR") msgs/s sent up to $PEER"
    else
      bad "telemetry from aircraft $AC stalled: $RXR msgs/s, nothing received for $LOST s"
    fi
    [ "$ERR" -gt 0 ] && warn "$ERR receive errors in link during the check"
    [ "$GCS" -ge 1 ] 2>/dev/null && info "laptops served by the handheld: $GCS"
    if awk "BEGIN{exit !($GRX > 0 && $RXR > 1.6 * $GRX)}"; then
      warn "this laptop gets every frame about twice ($RXR msgs/s, the handheld got $GRX/s): it reaches the handheld two ways"
    fi
    # Wi-Fi network: ping the handheld now its address is known
    if [ "$MODE" = net ] && [ "$PEER" != "?" ] && [ "$PEER" != "none" ]; then
      ping_handheld "${PEER%%:*}"
    fi
  fi
fi

echo
if [ "$FAIL" = 0 ]; then echo "HEALTHY ($MODE)"; else echo "NOT HEALTHY: see [FAIL] lines above"; fi
echo "Live statistics per leg (UART, radio, Wi-Fi): $DIR/link_stats.py"
exit $FAIL
