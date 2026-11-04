# pprzlink_router

Forwards Paparazzi telemetry between endpoints (UART and UDP). It is used on both Herelink units:

- **Air unit:** UART to the autopilot ↔ Herelink radio.
- **Ground unit:** Herelink radio ↔ the laptop, over USB tethering or the Herelink Wi-Fi hotspot.

```
pprzlink_router -e ENDPOINT -e ENDPOINT [...] [options]
  -e --endpoint uart://DEVICE:BAUD | udp://BIND_ADDR:BIND_PORT:SEND_ADDR:SEND_PORT
  -n --no-keep-awake   do not keep the CPU awake
  -p --pprz            only forward complete pprzlink v2 frames, one at a time
  -s --stats MS        period of LINK_ROUTER_REPORT (default 1000, 0 = off)
  -N --node air|ground node in LINK_ROUTER_REPORT (default: air if the first endpoint is an UART)
  -v --verbose         also prints the header of every frame (sender, receiver, class, message id)
```

Everything received on one endpoint is sent out of all the others. The first endpoint faces the aircraft:
what it receives is counted as downlink, what the other endpoints receive as uplink.

## Options

**Keep awake (on by default).** On the Herelink ground unit, when the CPU is idle the forwarding stalls for
~35 ms once per second. Several uplink messages then arrive late together. For external poses (OptiTrack)
on a fast-spinning drone this corrupted the attitude estimate.

- A thread at the lowest priority keeps one low-power core (CPU 0) busy, which removes the stalls.
- Measured at the air unit: 6.7–9.5 holds/min without it, 0 with it.
- Letting the thread sleep part of the time does not help.
- Use `-n` where it is not needed. The air unit's UART polling already keeps its CPU awake.

**Frame mode (`-p`).** Every endpoint parses its input as pprzlink v2 frames (STX 0x99, LEN, payload,
CK_A, CK_B). Only complete frames with a valid checksum are forwarded, one per send. This means:

- partial UART reads are never sent as fragments;
- frames from different endpoints (for example several drones) are never mixed;
- corrupted bytes are dropped at the router.

**Statistics.** Every second (`-s`) the router sends a `LINK_ROUTER_REPORT` towards the ground, with the
aircraft id learned from the downlink as sender, as soon as it has seen a downlink frame. It counts per
direction the frames received, rejected (bad checksum or length), sent (one per destination) and not sent
(socket full or error), and the longest time without a frame. Compared with `DATALINK_REPORT` (autopilot) and
`LINK_REPORT` (link) this tells on which leg frames get lost. Frames are counted exactly with `-p`; without it
the sent counters count the forwarded pieces. The air router's report goes over the radio (~56 bytes/s),
never over the UART. RSSI and SNR can be given on stdin, one line per value: `rssi <dBm>` or `snr <dB>`.

The UDP send buffer is kept small (32 kB): on a slow link frames are dropped (and counted) instead of
being delayed.

## Build

```
make                    # pprzlink_router (host) and pprzlink_router_cc (static ARM, for the Herelink)
herelink_app/build.sh   # Android app for the ground unit (see below)
```

`herelink_app/build.sh` needs `default-jdk-headless aapt dalvik-exchange apksigner android-sdk-platform-23`.

## Herelink air unit

The air unit runs a `userdebug` build, so `adb root` / `su` work. The router lives in `/oem/bin` and is
started at boot by `/system/bin/mass_storage.sh`, which calls `/oem/bin/init.sh`. Example `init.sh`
(use the baud rate of your airframe's telemetry):

```
#!/system/bin/sh
/oem/bin/pprzlink_router_cc -n -e uart:///dev/ttyS2:460800 -e udp://192.168.0.10:5243:192.168.0.11:5242 &
```

Disable the UART that Herelink uses for MAVLink: `setprop persist.uart.1.disable 1`.

## Herelink ground unit

The ground unit runs a `user` build (no root). Its boot scripts cannot be changed, so the router is started
by a small app (`herelink_app/`):

- At boot it starts the router.
- It restarts the router if the router stops.
- It sends to the laptop on USB tethering (192.168.42.255) when that is active, otherwise on the Wi-Fi
  hotspot (192.168.43.255).

Install it with:

```
adb install -r herelink_app/pprzlink_router.apk
adb shell am broadcast -n org.paparazziuav.pprzlinkrouter/.BootReceiver -f 32   # once, after installing
```

The laptop link itself (USB tethering or the hotspot) is enabled in the Herelink settings. Android switches
USB tethering off at every boot, so it has to be switched on again each session. These scripts do it over adb,
then check the whole link. They never start a router (the app does that):

| Script | What it does |
|---|---|
| `herelink_usb.sh` | Switches USB tethering on if needed, waits for the laptop's address, then runs the check |
| `herelink_wifi.sh` | Same for the Wi-Fi hotspot (switching it on needs the USB cable for adb) |
| `herelink_check.sh [usb\|wifi]` | Only checks: laptop address, ping, router and keep-awake on the handheld, link agent, telemetry |

They end with `HEALTHY (usb)` or with `[FAIL]` lines that say what to do. Use `--wait SECONDS` when they
start together with a Paparazzi session (default 20/15 s for the link agent and telemetry). They can be added
as tools to a Paparazzi Center control panel, for example:

```
<program name="Herelink USB" command="sw/misc/link/herelink_usb.sh"/>
```

Paparazzi side: `link -udp` (it receives on port 4242 and answers to port 4243).

## Herelink OEM images

To ship the router with the Herelink firmware, build an OEM image as described in the
[CubePilot OEM image guide](https://docs.cubepilot.org/herelink/herelink-user-guides/oem-image-setup):

- Start from the base image of the right unit (`oem_ru_base.img` for the ground unit, `oem_au_base.img`
  for the air unit).
- Add `bin/pprzlink_router_cc`. For the ground unit, also add the app in `app/`.
- Flash it with `fastboot flash oem <image>`.
