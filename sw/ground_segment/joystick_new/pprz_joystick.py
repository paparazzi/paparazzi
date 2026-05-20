#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from threading import Event

from config import parse_config
from env import ensure_pprz_paths, ivy_bus, paparazzi_home
from runtime import JoystickRuntime
from sdl_input import JoystickDevice, list_joysticks


ensure_pprz_paths()
from flight_plan import FlightPlan
from pprz_connect import PprzConnect
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings import PprzSettingsParser


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Forward SDL joystick input to the Paparazzi Ivy bus.")
    parser.add_argument("xml", nargs="?", help="joystick XML file in conf/joystick, or an absolute/relative path")
    parser.add_argument("-b", "--bus", default=None, help="Ivy bus")
    parser.add_argument("-ac", "--aircraft", default=None, help="aircraft name, defaults to the first detected aircraft")
    parser.add_argument("-d", "--device", type=int, default=0, help="SDL joystick device index")
    parser.add_argument("-v", "--verbose", action="store_true", help="print joystick values")
    parser.add_argument("-id", "--joystick-id", type=int, default=random.randint(0, 254), help="JoystickID() value")
    parser.add_argument("-c", "--check-axis", action="store_true", help="wait until every axis has moved")
    parser.add_argument("--list", action="store_true", help="list SDL joysticks and exit")
    args = parser.parse_args(argv)

    home = paparazzi_home()

    if args.list:
        for info in list_joysticks():
            print(f"{info.index}: {info.name} ({info.axes} axes, {info.buttons} buttons, {info.hats} hats)")
        return 0

    if not args.xml:
        parser.error("missing joystick XML file")

    xml_path = Path(args.xml)
    if not xml_path.is_absolute() and not xml_path.exists():
        xml_path = home / "conf" / "joystick" / args.xml
    xml_path = xml_path.resolve()

    bus = args.bus if args.bus is not None else ivy_bus()
    ivy = IvyMessagesInterface("pprz_joystick", start_ivy=True, verbose=False, ivy_bus=bus)
    device = None
    connect = None
    try:
        selected_config = {}
        config_ready = Event()

        def on_aircraft_config(config) -> None:
            if args.aircraft is None or config.name == args.aircraft:
                selected_config["config"] = config
                config_ready.set()

        connect = PprzConnect(notify=on_aircraft_config, ivy=ivy)
        connect._ivy = None
        if args.aircraft is None:
            print("Waiting for first aircraft on Ivy...")
        else:
            print(f"Waiting for aircraft {args.aircraft!r} on Ivy...")
        config_ready.wait()

        config = selected_config["config"]
        print(f"Using aircraft {config.name!r} with id {config.id}")
        ac_id = int(config.id)
        if ac_id > 0:
            settings_parser = PprzSettingsParser.parse(config.settings)
            settings = {setting.var: setting.index for setting in settings_parser.get_all_settings()}
            flight_plan = FlightPlan.parse(config.flight_plan)
            blocks = {block.name: block.no for block in flight_plan.blocks}
        else:
            settings = {}
            blocks = {}

        actions = parse_config(xml_path, settings, blocks)

        print(f"Joystick ID (option -id): {args.joystick_id}")
        print(f"Joystick SDL device index (option -d): {args.device}")

        device = JoystickDevice(args.device)
        print(f'Input device name: "{device.name}" on SDL device "{args.device}"')
        if args.check_axis:
            device.wait_until_all_axes_moved()

        def send_message(msg_class: str, name: str, values: dict[str, object]) -> None:
            message = PprzMessage(msg_class, name)
            for field, value in values.items():
                message[field] = value
            ivy.send(message, sender_id="input2ivy")

        runtime = JoystickRuntime(actions, ac_id, send_message, args.joystick_id, verbose=args.verbose)
        runtime.run(device)
    except KeyboardInterrupt:
        return 130
    finally:
        if connect is not None:
            connect._ivy = None
        ivy.shutdown()
        if device is not None:
            device.close()


if __name__ == "__main__":
    sys.exit(main())
