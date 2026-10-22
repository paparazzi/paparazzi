#!/usr/bin/env bash
# Connect the laptop to the Herelink handheld and check the link.
# Switches USB tethering (usb) or the Wi-Fi hotspot (wifi) on over adb if it is off, waits for the laptop's
# address, then runs herelink_check.sh. It does not start or restart the router (the pprzlink router app
# does that at boot).
# Usage: ./herelink_tether.sh usb|wifi [--wait SECONDS]   (or herelink_usb.sh / herelink_wifi.sh)
set -u

DIR="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-}"
shift || true
WAIT=20
while [ $# -gt 0 ]; do
  case "$1" in
    --wait) WAIT=${2:?--wait needs seconds}; shift 2 ;;
    *) echo "Unknown argument $1"; exit 2 ;;
  esac
done

case "$MODE" in
  usb)  HL_IP=192.168.42.129; NET=192.168.42.; LABEL="USB tethering" ;;
  wifi) HL_IP=192.168.43.1;   NET=192.168.43.; LABEL="Portable Wi" ;;
  *) echo "Usage: $0 usb|wifi [--wait SECONDS]"; exit 2 ;;
esac

ok()   { echo "[ OK ] $*"; }
warn() { echo "[WARN] $*"; }
info() { echo "       $*"; }
serial() { adb devices -l 2>/dev/null | awk '/optimus/ {print $1; exit}'; }

# Open the tethering screen on the handheld and tap the switch of the row whose title starts with $1.
# Prints: on (already on), tapped, or notfound. (Same method as herelink_connect.sh.)
tether_switch_on() {
  adb -s "$SER" shell input keyevent KEYCODE_WAKEUP
  adb -s "$SER" shell service call statusbar 2 >/dev/null 2>&1   # close the notification shade
  # -S: restart Settings, otherwise an open Settings window comes back on its last page
  adb -s "$SER" shell am start -S -n 'com.android.settings/.Settings\$TetherSettingsActivity' >/dev/null 2>&1
  sleep 2
  adb -s "$SER" shell uiautomator dump /sdcard/ui.xml >/dev/null 2>&1
  adb -s "$SER" shell cat /sdcard/ui.xml | grep -oE '<node [^>]*>' | awk -v label="$1" '
    /resource-id="android:id\/title"/ && $0 ~ "text=\"" label { found=1; next }
    found && /android:id\/switch_widget/ {
      match($0, /checked="[a-z]+"/); chk = substr($0, RSTART+9, RLENGTH-10)
      match($0, /bounds="[^"]+"/);   b = substr($0, RSTART+8, RLENGTH-9)
      gsub(/[\[\]]/, " ", b); gsub(/,/, " ", b); split(b, c, " ")
      print chk, int((c[1]+c[3])/2), int((c[2]+c[4])/2); exit }' | {
    read -r chk x y || { echo notfound; return; }
    if [ "$chk" = true ]; then echo on
    else adb -s "$SER" shell input tap "$x" "$y"; echo tapped; fi
  }
  # leave the settings screen (switching USB tethering reconnects the handheld's USB: wait for adb)
  sleep 2
  timeout 15 adb -s "$SER" wait-for-device >/dev/null 2>&1
  adb -s "$SER" shell input keyevent KEYCODE_BACK >/dev/null 2>&1
}

handheld_has_ip() { adb -s "$SER" shell ip -4 -br a 2>/dev/null | grep -q "$HL_IP/"; }

echo "Herelink $MODE connection $(date +%H:%M:%S)"

# 1. Handheld on adb (needed to switch the tethering; with Wi-Fi only if the cable is connected)
SER=$(serial)
if [ -z "$SER" ] && [ "$MODE" = usb ]; then
  echo "       waiting for the handheld on USB (switch it on and plug it in)..."
  for i in $(seq 1 60); do
    SER=$(serial); [ -n "$SER" ] && break
    if [ $((i % 15)) = 0 ]; then
      info "still not seen: in VMware Fusion connect it to the VM"
      info "(Virtual Machine > USB & Bluetooth > 'Google Optimus' / Android device)"
    fi
    sleep 1
  done
fi

# 2. Tethering / hotspot on the handheld
if [ -z "$SER" ]; then
  if [ "$MODE" = usb ]; then
    echo "[FAIL] handheld not seen on adb after 60 s"; exit 1
  fi
  warn "handheld not on adb (no cable): cannot switch the hotspot from here, make sure it is on"
elif handheld_has_ip; then
  ok "$LABEL already on (handheld $HL_IP)"
else
  case "$(tether_switch_on "$LABEL")" in
    tapped)   echo "       switched $LABEL on" ;;
    on)       echo "       $LABEL switch was already on" ;;
    notfound) echo "[FAIL] could not find the '$LABEL' switch: turn it on by hand on the handheld"; exit 1 ;;
  esac
  for i in $(seq 1 15); do handheld_has_ip && break; sleep 1; done
  if handheld_has_ip; then ok "$LABEL on (handheld $HL_IP)"
  else echo "[FAIL] handheld has no $HL_IP after switching $LABEL on"; exit 1; fi
fi

# 3. Laptop address on that network
for i in $(seq 1 30); do
  ip -4 -br a | awk -v p="$NET" 'index($3,p)==1 {f=1} END {exit !f}' && break
  if [ "$i" = 10 ]; then
    if [ "$MODE" = usb ]; then info "no ${NET}x address yet: VMware may show a new USB device, connect it to the VM"
    else info "no ${NET}x address yet: join the Herelink hotspot from the laptop"; fi
  fi
  sleep 1
done

# 4. Full check
echo
exec "$DIR/herelink_check.sh" "$MODE" --wait "$WAIT"
