#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from os import path, getenv

import serial

# Setup Paparazzi paths
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

try:
    from pprzlink.ivy import IvyMessagesInterface
    from pprzlink.message import PprzMessage
    HAS_PPRZLINK = True
except ImportError:
    HAS_PPRZLINK = False
    IvyMessagesInterface = None
    PprzMessage = None


FT742_QUERY = b"$//,WV?*//\r\n"


def build_query_bytes(query_text: str) -> bytes:
    query_text = query_text.strip()
    if not query_text:
        raise ValueError("query must not be empty")
    if not query_text.endswith("\r\n"):
        query_text += "\r\n"
    return query_text.encode("ascii")


def normalize_query_text(query_text: str) -> str:
    query_text = query_text.strip()
    if not query_text:
        raise ValueError("query must not be empty")
    # Allow shorthand like "WV?" and expand to FT742 query envelope.
    if not query_text.startswith("$"):
        query_text = f"$//,{query_text}*//"
    return query_text

def init_pprzlink(app_name: str = "ft742_reader"):
    """Initialize pprzlink interface if available."""
    if not HAS_PPRZLINK:
        return None
    try:
        interface = IvyMessagesInterface(app_name)
        return interface
    except Exception as e:
        print(f"Warning: Could not initialize pprzlink: {e}", file=sys.stderr)
        return None



def send_wind_info(pprzlink_interface, ac_id: int, wind_speed_mps: float, direction_rad: float, temp_celsius: float | None = None) -> None:
    """Send WIND_INFO uplink message via pprzlink."""
    if pprzlink_interface is None:
        return
    try:
        msg = PprzMessage("datalink", "WIND_INFO")
        msg.set_value_by_name("ac_id", ac_id)

        flags = 0x01  # bit 0: horizontal wind valid
        msg.set_value_by_name("east", wind_speed_mps * math.sin(direction_rad))
        msg.set_value_by_name("north", wind_speed_mps * math.cos(direction_rad))
        msg.set_value_by_name("up", 0.0)
        msg.set_value_by_name("airspeed", 0.0)

        if temp_celsius is not None:
            flags |= 0x08  # bit 3: temperature valid
            msg.set_value_by_name("temp", temp_celsius)
        else:
            msg.set_value_by_name("temp", -1000.0)  # matches air_data's "unknown" sentinel

        msg.set_value_by_name("flags", flags)
        pprzlink_interface.send_raw_datalink(msg)
    except Exception as e:
        print(f"Warning: Could not send WIND_INFO: {e}", file=sys.stderr)




def probe_queries(port: str, baud: int, query_candidates: list[str]):
    with serial.Serial(port=port, baudrate=baud, timeout=0.35) as ser:
        print(f"Connected to {port} at {baud} baud. Probe mode")
        for candidate in query_candidates:
            query_text = normalize_query_text(candidate)
            ser.write(build_query_bytes(query_text))
            raw_line = ser.readline().decode("ascii", errors="ignore").strip()
            if raw_line:
                print(f"probe query={query_text} raw={raw_line}")
            else:
                print(f"probe query={query_text} raw=<no response>")


def compute_checksum(payload: str) -> int:
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return value


def parse_ft742_line(line: str):
    line = line.strip()

    if not line.startswith("$") or "*" not in line:
        return None

    body, checksum_text = line[1:].split("*", 1)
    checksum_text = checksum_text.strip()

    if not checksum_text:
        return None

    try:
        rx_checksum = int(checksum_text, 16)
    except ValueError:
        return None

    if compute_checksum(body) != rx_checksum:
        return None

    marker = None
    for candidate in (",WVP=", ",WVC=", ",HT="):
        if candidate in body:
            marker = candidate
            break

    if marker is None:
        return None

    talker, data = body.split(marker, 1)
    sentence = marker.strip(",=")
    fields = data.split(",")

    sample = {
        "talker": talker,
        "sentence": sentence,
    }

    if sentence in {"WVP", "WVC"}:
        if len(fields) < 3:
            return None

        speed_text, angle_text, status_text = fields[0], fields[1], fields[2]

        try:
            # FT742 commonly reports speed as tenths when no decimal point is present.
            speed = float(speed_text) if "." in speed_text else int(speed_text) / 10.0
            angle_deg = float(angle_text)
            status = int(status_text)
        except ValueError:
            return None

        sample.update(
            {
                "speed": speed,
                "angle_deg": angle_deg,
                "direction_rad": math.radians(angle_deg - 180.0),
                "status": status,
            }
        )

    elif sentence == "HT":
        # Observed frame example: $WI,HT=99,00,+25*3F
        # Interpreted as: humidity, status, temperature.
        if len(fields) < 1:
            return None

        try:
            sample["humidity"] = float(fields[0])
            sample["humidity_unit"] = "%"
        except ValueError:
            pass

        if len(fields) >= 2 and fields[1]:
            try:
                sample["status"] = int(fields[1])
            except ValueError:
                sample["status_text"] = fields[1]

        if len(fields) >= 3 and fields[2]:
            try:
                sample["temperature"] = float(fields[2])
                sample["temperature_unit"] = "C"
            except ValueError:
                pass

        if "humidity" not in sample and "temperature" not in sample:
            return None

    # Extended fields can include triplets such as:
    #   <value>,<unit>,<validity>
    # e.g. +029.1,C,A or 45.2,%,A
    extras = fields[3:] if sentence in {"WVP", "WVC"} else fields[3:]
    if extras:
        # Parse as repeating value/unit pairs; treat trailing single letter as validity.
        i = 0
        while i < len(extras):
            token = extras[i].strip()

            # A/V style validity flags are often sent as standalone trailing token.
            if len(token) == 1 and token.isalpha():
                sample["validity"] = token
                i += 1
                continue

            if i + 1 >= len(extras):
                break

            unit = extras[i + 1].strip()

            try:
                value = float(token)
            except ValueError:
                i += 1
                continue

            unit_upper = unit.upper()

            if unit_upper in {"C", "F", "K"}:
                sample["temperature"] = value
                sample["temperature_unit"] = unit
            elif "%" in unit_upper or unit_upper in {"RH", "%RH"}:
                # Relative humidity in percent (0-100).
                sample["humidity"] = value
                sample["humidity_unit"] = "%"
            else:
                # Keep unknown numeric/unit pair for troubleshooting sentence variants.
                sample.setdefault("extra_values", []).append({"value": value, "unit": unit})

            i += 2

    return sample


def run(port: str, baud: int, rate_hz: float, query_text: str | None, ac_id: int, multi_query: bool = False, use_pprzlink: bool = True, verbose: bool = False) -> None:
    period_s = 1.0 / rate_hz

    if multi_query:
        # Alternate between WV? (wind) and HT? (humidity) queries.
        queries = ["WV?", "HT?"]
        query_list = [build_query_bytes(normalize_query_text(q)) for q in queries]
        query_idx = 0
    else:
        query_bytes = build_query_bytes(normalize_query_text(query_text)) if query_text else None
        query_list = None

    latest_data = {}
    pprzlink_interface = init_pprzlink() if use_pprzlink else None

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        if multi_query:
            mode = "multi-query (alternating WV? and HT?)"
        elif query_text:
            mode = f"query={query_text!r}"
        else:
            mode = "listen-only"

        pprzlink_status = "pprzlink enabled" if pprzlink_interface else "pprzlink disabled"
        print(f"Connected to {port} at {baud} baud. Polling at {rate_hz:.1f} Hz ({mode}) [{pprzlink_status}]")

        while True:
            cycle_start = time.time()
            if multi_query:
                ser.write(query_list[query_idx])
                query_idx = (query_idx + 1) % len(query_list)
            elif query_bytes is not None:
                ser.write(query_bytes)
            raw_line = ser.readline().decode("ascii", errors="ignore")
            raw = raw_line.strip()

            if not raw_line:
                print("No response")
            else:
                sample = parse_ft742_line(raw_line)
                if verbose:
                    print(f"raw={raw}")
                if sample is None:
                    if verbose:
                        print("parsed=None (invalid frame)")
                else:
                    if multi_query:
                        latest_data.update(sample)
                        if verbose:
                            print(f"parsed={json.dumps(latest_data, sort_keys=True)}")
                        data_to_send = latest_data
                    else:
                        if verbose:
                            print(f"parsed={json.dumps(sample, sort_keys=True)}")
                        data_to_send = sample

                    # Extract wind data and send via pprzlink if available.

                    if "speed" in data_to_send and "angle_deg" in data_to_send:
                        wind_speed_mps = float(data_to_send["speed"])
                        direction_rad = float(data_to_send.get("direction_rad", math.radians(data_to_send["angle_deg"] - 180.0)))
                        temp_c = data_to_send.get("temperature")  # None if not yet received this cycle

                        send_wind_info(pprzlink_interface, ac_id, wind_speed_mps, direction_rad, temp_c)

                        if not verbose:
                            temp_str = f"{temp_c:.1f}C" if temp_c is not None else "n/a"
                            print(f"wind_speed={wind_speed_mps:.2f}m/s dir={data_to_send['angle_deg']:.1f}deg temp={temp_str}")



            elapsed = time.time() - cycle_start
            sleep_time = period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


def main():
    parser = argparse.ArgumentParser(description="Simple FT742 serial reader")
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    parser.add_argument("--rate", type=float, default=10.0, help="Polling rate in Hz (max 10)")
    parser.add_argument(
        "--query",
        default="WV?",
        help="Command to send each cycle; can be full frame or shorthand like 'WV?' (default: WV?)",
    )
    parser.add_argument(
        "--no-pprzlink",
        action="store_true",
        help="Do not send messages via pprzlink",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print raw and parsed frames (otherwise just prints compact output)",
    )
    parser.add_argument(
        "--no-query",
        action="store_true",
        help="Do not send any command, only listen for streamed frames",
    )
    parser.add_argument(
        "--multi-query",
        action="store_true",
        help="Alternate WV? (wind) and HT? (humidity) queries, merge latest values",
    )
    parser.add_argument(
        "--probe",
        action="store_true",
        help="Probe candidate queries once each and print raw replies, then exit",
    )
    parser.add_argument(
        "--probe-list",
        default="WV?,WVC?,WH?,RH?,HU?,WT?",
        help="Comma-separated query candidates used by --probe",
    )
    parser.add_argument(
    "--ac-id",
    type=int,
    default=2,
    help="Aircraft ID for Paparazzi logging"
    )


    args = parser.parse_args()

    if args.rate <= 0 or args.rate > 10.0:
        print("--rate must be in range (0, 10]", file=sys.stderr)
        raise SystemExit(2)

    probe_candidates = [q.strip() for q in args.probe_list.split(",") if q.strip()]
    if args.probe and not probe_candidates:
        print("--probe-list must contain at least one candidate", file=sys.stderr)
        raise SystemExit(2)

    try:
        if args.probe:
            probe_queries(args.port, args.baud, probe_candidates)
        else:
            run(
                args.port,
                args.baud,
                args.rate,
                None if args.no_query else args.query,
                args.ac_id,
                args.multi_query,
                use_pprzlink=not args.no_pprzlink,
                verbose=args.verbose,
            )
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
