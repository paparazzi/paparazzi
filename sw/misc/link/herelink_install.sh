#!/usr/bin/env bash
# Install or update the pprzlink router on the Herelink units connected over USB (adb).
#  - Air unit (Telepathy Air, root): /oem/bin/pprzlink_router_cc and /oem/bin/init.sh, restarted at once.
#    The UART and radio endpoints of an existing init.sh are kept.
#  - Ground unit (handheld): the pprzlink router app (herelink_app), started at once.
# The router and the app are (re)built when their sources changed. Both routers log all frames
# (air: /data/pprzlink_log, 500 MB; ground: the app's folder, 1 GB): --logs copies them to the laptop.
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
APK="$DIR/herelink_app/pprzlink_router.apk"
PKG=org.paparazziuav.pprzlinkrouter
AIR_LOG=/data/pprzlink_log
AIR_LOG_MB=500
GROUND_LOG=/sdcard/Android/data/$PKG/files/log
UART= BAUD= ONLY= SERIAL= BUILD=1 LOGS=

usage() {
  cat <<EOF
Install or update the pprzlink router on the Herelink units connected over USB.

usage: $(basename "$0") [options]
  --air, --ground   only update this unit (default: every Herelink unit found)
  -s SERIAL         only this adb device
  -u DEVICE         air unit UART (default: from the installed init.sh, else /dev/ttyS2)
  -b BAUD           air unit UART baud rate (default: from the installed init.sh, else 460800)
  --no-build        use the router and app as they are built
  --logs [DIR]      do not update: copy the logs of the routers to DIR
                    (default: var/logs/herelink/DATE), convert them with sw/logalizer/sd2log
  -h, --help        show this help
EOF
}

while [ $# -gt 0 ]; do
  case $1 in
    --air) ONLY=air ;;
    --ground) ONLY=ground ;;
    -s) SERIAL=$2; shift ;;
    -u) UART=$2; shift ;;
    -b) BAUD=$2; shift ;;
    --no-build) BUILD=0 ;;
    --logs) LOGS="$(cd "$DIR/../../.." && pwd)/var/logs/herelink/$(date +%Y%m%d_%H%M%S)"
            if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then LOGS=$2; shift; fi ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

ok()   { echo "  [ OK ] $*"; }
fail() { echo "  [FAIL] $*"; }

# Find the Herelink units: "serial device ... product:telepathy-air" (air) or "product:optimus" (ground)
command -v adb >/dev/null || { echo "adb not found: sudo apt install adb" >&2; exit 1; }
AIR=() GROUND=()
while read -r serial state rest; do
  [ -n "$SERIAL" ] && [ "$serial" != "$SERIAL" ] && continue
  case $rest in
    *product:telepathy-air*) kind=air ;;
    *product:optimus*) kind=ground ;;
    *) continue ;;
  esac
  if [ "$state" != device ]; then
    echo "Herelink $kind unit $serial is '$state' (accept the USB debugging prompt, or replug)" >&2
    continue
  fi
  [ -n "$ONLY" ] && [ "$ONLY" != "$kind" ] && continue
  if [ $kind = air ]; then AIR+=("$serial"); else GROUND+=("$serial"); fi
done < <(adb devices -l | tail -n +2)

if [ ${#AIR[@]} -eq 0 ] && [ ${#GROUND[@]} -eq 0 ]; then
  echo "No Herelink unit found over adb (adb devices -l)" >&2
  exit 1
fi
echo "Found: ${#AIR[@]} air unit(s) ${AIR[*]}, ${#GROUND[@]} ground unit(s) ${GROUND[*]}"

# Copy the logs of the routers
if [ -n "$LOGS" ]; then
  for s in "${AIR[@]}"; do
    adb -s "$s" root >/dev/null && timeout 30 adb -s "$s" wait-for-device || { echo "air unit $s: no adb" >&2; continue; }
    mkdir -p "$LOGS/air_$s"
    adb -s "$s" pull "$AIR_LOG/." "$LOGS/air_$s" | tail -1
  done
  for s in "${GROUND[@]}"; do
    mkdir -p "$LOGS/ground_$s"
    adb -s "$s" pull "$GROUND_LOG/." "$LOGS/ground_$s" | tail -1
  done
  echo "Logs in $LOGS"
  exit 0
fi

# Build what is needed
if [ $BUILD -eq 1 ]; then
  make -s -C "$DIR" pprzlink_router_cc
  if [ ${#GROUND[@]} -gt 0 ]; then
    newer=$(find "$DIR/pprzlink_router_cc" "$DIR/herelink_app/src" "$DIR/herelink_app/AndroidManifest.xml" \
      -newer "$APK" 2>/dev/null | head -1)
    if [ ! -f "$APK" ] || [ -n "$newer" ]; then
      for t in javac dalvik-exchange aapt apksigner; do
        command -v $t >/dev/null || { echo "Cannot build the ground unit app, $t is missing:" >&2
          echo "  sudo apt install default-jdk-headless aapt dalvik-exchange apksigner android-sdk-platform-23" >&2; exit 1; }
      done
      "$DIR/herelink_app/build.sh" >/dev/null
    fi
  fi
fi
[ -f "$DIR/pprzlink_router_cc" ] || { echo "$DIR/pprzlink_router_cc is not built (make)" >&2; exit 1; }

update_air() {
  local s=$1 a="adb -s $1" init cur
  echo "Air unit $s"
  # adb restarts as root, the unit reconnects
  if ! { $a root >/dev/null && timeout 30 $a wait-for-device; }; then
    fail "the air unit did not come back after adb root (reconnect its USB cable)"
    return
  fi
  init=$($a shell cat /oem/bin/init.sh 2>/dev/null | tr -d '\r')
  # keep the endpoints of the installed init.sh
  cur=$(grep -o 'uart://[^ ]*' <<<"$init" | head -1)
  local uart=${UART:-$(sed -E 's#uart://([^:]*):.*#\1#' <<<"$cur")}
  local baud=${BAUD:-$(sed -E 's#uart://[^:]*:([0-9]+).*#\1#' <<<"$cur")}
  uart=${uart:-/dev/ttyS2} baud=${baud:-460800}
  local radio=$(grep -o 'udp://[^ ]*' <<<"$init" | head -1)
  radio=${radio:-udp://192.168.0.10:5243:192.168.0.11:5242}

  $a push "$DIR/pprzlink_router_cc" /data/local/tmp/pprzlink_router_cc >/dev/null
  local tmp=$(mktemp)
  cat > "$tmp" <<'EOF'
#!/system/bin/sh
# pprzlink router, installed by herelink_install.sh
# The router gets values for its LINK_STATS on stdin: the radio RSRP measured by the air unit ("Air M" in the
# Herelink app, logged 5 times per second by d2d_config in the radio log, averaged per second), and the temperature.
rsrp() {
  sleep 2
  while pidof pprzlink_router_cc >/dev/null; do
    logcat -b radio -v raw -T 1 -s d2d_config:D
    sleep 1
  done | /bin/busybox awk '/message "RSRP -?[0-9]+"/ {
    match($0, /RSRP -?[0-9]+/); s += substr($0, RSTART + 5, RLENGTH - 5)
    if (++n == 5) { printf "rssi %d\n", s / n; fflush(); s = 0; n = 0 } }'
}
# the hottest temperature sensor, every 10 s
temp() {
  sleep 2
  while pidof pprzlink_router_cc >/dev/null; do
    t=0
    for z in /sys/class/thermal/thermal_zone*/temp; do read v < $z; [ $v -gt $t ] && t=$v; done
    echo "temp $((t / 1000))"
    sleep 10
  done
}
{ rsrp & temp; } | /oem/bin/pprzlink_router_cc -n -p -e uart://@UART@:@BAUD@ -e @RADIO@ -l @LOG@ -L @LOG_MB@ &
EOF
  sed -i "s#@UART@#$uart#; s#@BAUD@#$baud#; s#@RADIO@#$radio#; s#@LOG@#$AIR_LOG#; s#@LOG_MB@#$AIR_LOG_MB#" "$tmp"
  $a push "$tmp" /data/local/tmp/init.sh >/dev/null
  rm -f "$tmp"
  # after a boot the /oem block device is read-only: clear that first. The toybox mount fails on /oem
  # ("busy"), the busybox one works
  if ! $a shell 'set -e
    for p in $(pgrep -f /oem/bin/init.sh) $(pgrep -f d2d_config:D); do [ $p = $$ ] || kill $p 2>/dev/null || true; done
    for p in $(pidof pprzlink_router_cc); do kill $p; done
    blockdev --setrw $(readlink -f /dev/block/platform/soc/by-name/oem)
    /bin/busybox mount -o remount,rw /oem
    touch /oem/bin/.rw && rm /oem/bin/.rw
    cp /data/local/tmp/pprzlink_router_cc /data/local/tmp/init.sh /oem/bin/
    chmod 755 /oem/bin/pprzlink_router_cc /oem/bin/init.sh
    rm /data/local/tmp/pprzlink_router_cc /data/local/tmp/init.sh
    sync; /bin/busybox mount -o remount,ro /oem || true
    setprop persist.uart.1.disable 1'; then
    fail "could not write /oem/bin (the old router is stopped: reboot the air unit to start it again)"
    return
  fi
  ok "router and init.sh in /oem/bin: uart://$uart:$baud, $radio, log $AIR_LOG ($AIR_LOG_MB MB)"
  $a shell grep -q /oem/bin/init.sh /system/bin/mass_storage.sh \
    && ok "started at boot (/system/bin/mass_storage.sh)" \
    || fail "not started at boot: /system/bin/mass_storage.sh does not call /oem/bin/init.sh (run /oem/bin/install.sh of the OEM image)"
  $a shell 'nohup sh /oem/bin/init.sh >/dev/null 2>&1 &' ; sleep 1
  if $a shell pidof pprzlink_router_cc >/dev/null; then ok "router running"; else fail "router not running"; fi
}

update_ground() {
  local s=$1 a="adb -s $1" out
  echo "Ground unit $s"
  out=$($a install -r "$APK" 2>&1) || true
  if grep -qE "UPDATE_INCOMPATIBLE|signatures do not match" <<<"$out"; then
    echo "  app installed with another key: reinstalling"
    $a uninstall $PKG >/dev/null
    out=$($a install "$APK" 2>&1) || true
  fi
  grep -q Success <<<"$out" && ok "app installed" || { fail "app not installed: $out"; return; }
  # the radio signal of the handheld is read from the radio log (restart the app so the permission applies)
  if $a shell pm grant $PKG android.permission.READ_LOGS; then ok "radio signal: READ_LOGS granted"
  else fail "radio signal: could not grant READ_LOGS, the handheld's RSSI is not reported"; fi
  $a shell am force-stop $PKG
  $a shell am broadcast -n $PKG/.BootReceiver -f 32 >/dev/null
  sleep 2
  if $a shell ps | grep -q "/oem/bin/pprzlink_router_cc"; then
    fail "an old router from /oem/bin is also running (started by hand): reboot the handheld"
  fi
  if $a shell ps | grep -q "$PKG/files/pprzlink_router"; then ok "router running, log $GROUND_LOG (adb logcat -s pprzlink_router)"
  else fail "router not running: adb -s $s logcat -s pprzlink_router"; fi
}

for s in "${AIR[@]}"; do update_air "$s"; done
for s in "${GROUND[@]}"; do update_ground "$s"; done
echo "Done. Check the link with herelink_check.sh or link_stats.py"
