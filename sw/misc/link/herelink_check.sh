#!/usr/bin/env bash
# Check that the Herelink link to Paparazzi is healthy. Only checks: it does not start or restart anything.
# Usage: ./herelink_check.sh [usb|wifi] [--wait SECONDS]
#   mode    default: detected from the laptop address (192.168.42.x = usb, 192.168.43.x = wifi)
#   --wait  how long to wait for Paparazzi's link agent and telemetry (default 15 s, useful when the
#           check starts together with a Paparazzi session)
# Exit code 0 when everything needed for telemetry is OK.
set -u

MODE=""
WAIT=15
while [ $# -gt 0 ]; do
  case "$1" in
    usb|wifi) MODE=$1; shift ;;
    --wait) WAIT=${2:?--wait needs seconds}; shift 2 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "Unknown argument $1 (usage: $0 [usb|wifi] [--wait SECONDS])"; exit 2 ;;
  esac
done

FAIL=0
ok()   { echo "[ OK ] $*"; }
bad()  { echo "[FAIL] $*"; FAIL=1; }
warn() { echo "[WARN] $*"; }
info() { echo "       $*"; }

echo "Herelink link check $(date +%H:%M:%S)"

# 1. Laptop address on the Herelink network
USB_IF=$(ip -4 -br a | awk 'index($3,"192.168.42.")==1 {print $1; exit}')
WIFI_IF=$(ip -4 -br a | awk 'index($3,"192.168.43.")==1 {print $1; exit}')
if [ -z "$MODE" ]; then
  if [ -n "$USB_IF" ]; then MODE=usb; elif [ -n "$WIFI_IF" ]; then MODE=wifi; fi
fi
case "$MODE" in
  usb)  IFACE=$USB_IF;  HL_IP=192.168.42.129; BCAST=192.168.42.255 ;;
  wifi) IFACE=$WIFI_IF; HL_IP=192.168.43.1;   BCAST=192.168.43.255 ;;
  *)    IFACE="" ;;
esac
if [ -n "$IFACE" ]; then
  ok "laptop on the Herelink $MODE network: $IFACE $(ip -4 -br a show "$IFACE" | awk '{print $3}')"
else
  bad "laptop has no 192.168.42.x (USB tethering) or 192.168.43.x (Herelink hotspot) address"
  info "USB: turn on USB tethering on the handheld and connect the new USB device to the VM (VMware Fusion)"
  info "Wi-Fi: turn on the hotspot on the handheld and join it from the laptop"
fi

# 2. Handheld reachable
if [ -n "$IFACE" ]; then
  if ping -c 2 -W 1 "$HL_IP" >/dev/null 2>&1; then ok "handheld answers ping ($HL_IP)"
  else bad "no ping reply from the handheld ($HL_IP)"; fi
fi

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
    bad "two routers on the handheld: the app's and an old one started over adb (herelink_connect.sh?)"
    info "stop the old one: adb shell 'kill \$(pidof pprzlink_router_cc pprzlink_router_test)'"
  elif [ "$OTHER" -ge 1 ]; then
    warn "router running, but not the app's one (old adb start: no keep-awake fix)"
  elif adb -s "$SER" shell pm list packages 2>/dev/null | grep -q org.paparazziuav.pprzlinkrouter; then
    bad "app installed but no router running: start it once with"
    info "adb shell am broadcast -n org.paparazziuav.pprzlinkrouter/.BootReceiver -f 32"
  else
    bad "no router and no app on the handheld: install the app (see pprzlink_router.md, Herelink ground unit)"
  fi

  if [ "$APP" -ge 1 ]; then
    LAST=$(adb -s "$SER" shell logcat -d -s pprzlink_router 2>/dev/null | grep 'Started the router to' | tail -1 | tr -d '\r')
    TO=${LAST##* }
    if [ -n "$MODE" ] && [ "$TO" = "$BCAST" ]; then ok "router sends to the $MODE network ($TO)"
    elif [ -n "$TO" ]; then bad "router sends to $TO, not to the $MODE network ($BCAST): it switches within ~2 s after the tethering changes"
    else warn "could not read which network the router sends to"; fi

    PID=$(adb -s "$SER" shell pidof pprzlink_router 2>/dev/null | tr -d '\r' | awk '{print $1}')
    NICE19=$(adb -s "$SER" shell "for t in /proc/$PID/task/*; do cut -d' ' -f19 \$t/stat; done" 2>/dev/null | tr -d '\r' | grep -c '^19$')
    if [ "$NICE19" -ge 1 ]; then ok "keep-awake active (uplink holds fix)"
    else warn "keep-awake not active (router started with -n?)"; fi
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
rep = []
IvyInit("herelink_check", "herelink_check READY")
IvyStart("127.255.255.255:2010")
# ac_id link_id run_time rx_lost_time rx_bytes rx_msgs rx_err rx_bytes_rate rx_msgs_rate tx_msgs ping_time
IvyBindMsg(lambda a, *g: rep.append(g), r"^link LINK_REPORT (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
t0 = time.time()
while time.time() - t0 < max(float(sys.argv[1]), 4.) and len(rep) < 4:
    time.sleep(0.2)
IvyStop()
if len(rep) >= 2:
    a, b = rep[0], rep[-1]
    dt = max(float(b[2]) - float(a[2]), 1.)
    print(b[0], float(b[8]), int(b[6]) - int(a[6]), (int(b[9]) - int(a[9])) / dt, float(b[10]), int(b[3]))
EOF
)
  if [ -z "$REP" ]; then
    bad "no telemetry: link reports no aircraft (is the drone on? is the router sending to the laptop?)"
  else
    read -r AC RXR ERR TXR PING LOST <<< "$REP"
    if awk "BEGIN{exit !($RXR > 1)}" && [ "$LOST" -lt 3 ]; then
      ok "telemetry from aircraft $AC: $(printf '%.0f' "$RXR") msgs/s received, $ERR errors, $(printf '%.0f' "$TXR") msgs/s sent up"
    else
      bad "telemetry from aircraft $AC stalled: $RXR msgs/s, nothing received for $LOST s"
    fi
    [ "$ERR" -gt 0 ] && warn "$ERR receive errors in link during the check"
  fi
fi

echo
if [ "$FAIL" = 0 ]; then echo "HEALTHY ($MODE)"; else echo "NOT HEALTHY: see [FAIL] lines above"; fi
exit $FAIL
