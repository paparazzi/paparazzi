#!/usr/bin/env python3
"""Forward RTCM3 corrections from NTRIP or mixed UBX/RTCM serial input."""

import argparse
import base64
import os
import sys
import time
import urllib.error
import urllib.request

PPRZ_HOME = os.getenv(
    "PAPARAZZI_HOME",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../..")),
)
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from pyrtcm import RTCMMessage
from pyubx2 import ERR_LOG, RTCM3_PROTOCOL, UBX_PROTOCOL, UBXReader, VALCKSUM, ecef2llh

DEFAULT_IVY_BUS = "127.255.255.255:2010"
DEFAULT_URL = "https://crtk.net:443"
DEFAULT_MOUNTPOINT = "ENFA"
DEFAULT_USER = "centipede"
DEFAULT_PASSWORD = "centipede"
DEFAULT_SERIAL_BAUDRATE = 9600
RTCM3_BLACKLIST = {1004, 1006, 1008, 1012, 1033, 1042, 1046, 1127}

RTCM3_TO_PPRZ_ID = {
    1005: 0x69,
    1077: 0xB1,
    1087: 0xBB,
    1097: 0xC5,
    4072: 0x72,
    1230: 0xE6,
}

MSM_MESSAGE_TYPES = {
    *(range(1071, 1078)),
    *(range(1081, 1088)),
    *(range(1091, 1098)),
    *(range(1101, 1108)),
    *(range(1111, 1118)),
    *(range(1121, 1128)),
    *(range(1131, 1138)),
}

DF393_BIT_OFFSET = 3 * 8 + 12 + 12 + 30
DF393_BYTE_OFFSET = DF393_BIT_OFFSET // 8
DF393_MASK = 1 << (7 - (DF393_BIT_OFFSET % 8))


def rtcm_message_type(parsed):
    return int(str(parsed.identity).split("_", 1)[0])


def rtcm_set_df393_zero(frame, msg_type, msg_types):
    if msg_type not in msg_types or msg_type not in MSM_MESSAGE_TYPES:
        return frame

    frame = bytearray(frame)
    frame[DF393_BYTE_OFFSET] &= ~DF393_MASK
    return RTCMMessage(payload=bytes(frame[3:-3])).serialize()


def gnss_messages(stream, buffer_size):
    """Yield validated RTCM frames while consuming interleaved UBX messages."""
    reader = UBXReader(
        stream,
        protfilter=RTCM3_PROTOCOL | UBX_PROTOCOL,
        quitonerror=ERR_LOG,
        validate=VALCKSUM,
        bufsize=buffer_size,
    )
    for raw, parsed in reader:
        if raw and parsed and raw[0] == 0xD3:
            yield raw, parsed


def open_ntrip_stream(url, username, password):
    token = base64.b64encode(f"{username}:{password}".encode("ascii")).decode("ascii")
    request = urllib.request.Request(
        url,
        headers={
            "Authorization": f"Basic {token}",
            "Ntrip-Version": "Ntrip/2.0",
            "User-Agent": "NTRIP ntrip2ivy/1.0",
            "Connection": "close",
        },
    )
    return urllib.request.urlopen(request, timeout=30)


def ivy_send_rtcm(ivy, packet_id, frame, packet_size, delay):
    max_payload = packet_size - 6
    if max_payload <= 0:
        raise ValueError("packet_size must be greater than 6")

    for offset in range(0, len(frame), max_payload):
        fragment = frame[offset:offset + max_payload]
        message = PprzMessage("datalink", "RTCM_INJECT")
        message["packet_id"] = packet_id
        message["data"] = list(fragment)
        time.sleep(delay)
        ivy.send(message)


def send_ground_station_position(ivy, rtcm):
    lat, lon, alt = ecef2llh(rtcm.DF025, rtcm.DF026, rtcm.DF027)
    message = PprzMessage("ground", "FLIGHT_PARAM")
    message["ac_id"] = "GCS"
    message["roll"] = 0.0
    message["pitch"] = 0.0
    message["heading"] = 0.0
    message["lat"] = lat
    message["long"] = lon
    message["speed"] = 0.0
    message["course"] = 0.0
    message["alt"] = alt
    message["climb"] = 0.0
    message["agl"] = 0.0
    message["unix_time"] = 0.0
    message["itow"] = 0
    message["airspeed"] = 0.0
    ivy.send(message)


def init_ivy(ivy_bus):
    return IvyMessagesInterface("ntrip2ivy", ivy_bus=ivy_bus)


def open_serial(port, baudrate):
    import serial

    return serial.Serial(port, baudrate)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-m", "--mountpoint", default=DEFAULT_MOUNTPOINT, help=f"NTRIP mountpoint (default: {DEFAULT_MOUNTPOINT})")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"NTRIP caster URL (default: {DEFAULT_URL})")
    parser.add_argument("--username", default=DEFAULT_USER, help=f"NTRIP username (default: {DEFAULT_USER})")
    parser.add_argument("--password", default=DEFAULT_PASSWORD, help="NTRIP password")

    parser.add_argument( "-d", dest="rtk_base", help="Read mixed UBX/RTCM input from this RTK base serial port instead of NTRIP")
    parser.add_argument( "-b", dest="rtk_base_baudrate", type=int, default=DEFAULT_SERIAL_BAUDRATE, help=f"RTK base baudrate (default: {DEFAULT_SERIAL_BAUDRATE})")

    parser.add_argument("--no-ivy", action="store_true", help="Do not forward RTCM messages to Ivy")
    parser.add_argument("--bus", default=os.getenv("IVY_BUS", DEFAULT_IVY_BUS), help="Ivy bus address")

    parser.add_argument( "--serial-output", dest="serial_output", help="Optional serial output receiving raw RTCM frames")
    parser.add_argument( "--serial-output-baudrate", dest="serial_output_baudrate", type=int, default=DEFAULT_SERIAL_BAUDRATE, help=f"Serial output baudrate (default: {DEFAULT_SERIAL_BAUDRATE})")

    parser.add_argument("-p", "--packet-size", type=int, default=100, help="Paparazzi payload size")
    parser.add_argument("--chunk-size", type=int, default=1024, help="GNSS parser read buffer size")
    parser.add_argument("--reconnect-delay", type=float, default=5.0, help="Delay before reconnecting")
    parser.add_argument("--fragment-delay", type=float, default=0.05, help="Delay between Ivy fragments")
    parser.add_argument(
        "--zero-df393",
        type=int,
        nargs="*",
        default=[1097],
        metavar="MSG_ID",
        help=(
            "Set the MSM DF393 multiple-message bit to 0 for these message IDs and recompute "
            "the RTCM CRC before forwarding (default: 1097; use --zero-df393 with no IDs to disable)"
        ),
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Print forwarded/skipped RTCM messages")
    return parser.parse_args()


def forward_rtcm(args, ivy, serial_output, frame, parsed):
    msg_type = rtcm_message_type(parsed)
    if msg_type in RTCM3_BLACKLIST:
        if args.verbose:
            print(f"Skipping blacklisted RTCM type {msg_type}", file=sys.stderr)
        return False
    if args.zero_df393:
        frame = rtcm_set_df393_zero(frame, msg_type, args.zero_df393)

    forwarded = []
    if serial_output:
        serial_output.write(frame)
        serial_output.flush()
        forwarded.append("serial")

    if ivy:
        packet_id = RTCM3_TO_PPRZ_ID.get(msg_type)
        if packet_id is None:
            if args.verbose:
                print(f"Skipping unsupported Ivy RTCM type {msg_type}", file=sys.stderr)
        else:
            ivy_send_rtcm(ivy, packet_id, frame, args.packet_size, args.fragment_delay)
            forwarded.append(f"Ivy packet {packet_id}")
            if msg_type == 1005:
                send_ground_station_position(ivy, parsed)

    if args.verbose:
        if forwarded:
            print(f"Forwarded RTCM {msg_type} to {', '.join(forwarded)} ({len(frame)} bytes)", file=sys.stderr)
        else:
            print(f"No transport accepted RTCM {msg_type} ({len(frame)} bytes)", file=sys.stderr)
    return bool(forwarded)


def main():
    args = parse_args()
    use_ivy = not args.no_ivy
    if not use_ivy and not args.serial_output:
        print("No transport enabled; use Ivy or set --serial-output", file=sys.stderr)
        return 2
    if args.chunk_size <= 0:
        print("chunk_size must be positive", file=sys.stderr)
        return 2

    ivy = init_ivy(args.bus) if use_ivy else None
    serial_output = (
        open_serial(args.serial_output, args.serial_output_baudrate) if args.serial_output else None
    )
    serial_input = None
    stream_url = f"{args.url.rstrip('/')}/{args.mountpoint.lstrip('/')}"
    count = 0

    try:
        while True:
            try:
                if args.rtk_base:
                    serial_input = open_serial(args.rtk_base, args.rtk_base_baudrate)
                    for frame, parsed in gnss_messages(serial_input, args.chunk_size):
                        count += forward_rtcm(args, ivy, serial_output, frame, parsed)
                    raise OSError("serial input stream closed")

                with open_ntrip_stream(stream_url, args.username, args.password) as stream:
                    for frame, parsed in gnss_messages(stream, args.chunk_size):
                        count += forward_rtcm(args, ivy, serial_output, frame, parsed)
                raise OSError("NTRIP stream closed")
            except (OSError, urllib.error.URLError, TimeoutError) as exc:
                if serial_input:
                    serial_input.close()
                    serial_input = None
                source = "RTK base" if args.rtk_base else "NTRIP connection"
                print(f"{source} failed: {exc}; reconnecting in {args.reconnect_delay:g}s", file=sys.stderr)
                time.sleep(args.reconnect_delay)
    except KeyboardInterrupt:
        if args.verbose:
            print(f"Stopped after forwarding {count} RTCM frames", file=sys.stderr)
    finally:
        if ivy:
            ivy.shutdown()
        if serial_input:
            serial_input.close()
        if serial_output:
            serial_output.close()


if __name__ == "__main__":
    sys.exit(main())
