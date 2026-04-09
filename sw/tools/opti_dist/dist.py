#!/usr/bin/env python3
import time
from datetime import datetime
import sys
import os
import math
import socket
import struct
import argparse
import matplotlib.pyplot as plt

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/sw/ground_segment/python/natnet3.x")

from NatNetClient import NatNetClient


def discover_motive(multicast="239.255.42.99", data_port=1511, timeout=3):
    """Discover a Motive server by listening for NatNet multicast data."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind(('', data_port))

    # Join the NatNet multicast group
    mreq = struct.pack('4sL', socket.inet_aton(multicast), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        _, addr = sock.recvfrom(4096)
        sock.close()
        return addr[0]
    except socket.timeout:
        sock.close()
        return None

# Global state
recording = False
pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
track_id = None
frame_count = 0
seen_ids = set()


def on_press(event):
    global recording
    if event.key == 'r':
        recording = not recording
    if event.key == 'q':
        plt.close()


def receive_rigid_body_frame(rigid_body_data, stamp):
    global pos_x, pos_y, pos_z, frame_count, seen_ids
    frame_count += 1
    for rb in rigid_body_data.rigid_body_list:
        seen_ids.add(rb.id_num)
        if track_id is not None and rb.id_num != track_id:
            continue
        pos_x = rb.pos[0]
        pos_y = rb.pos[1]
        pos_z = rb.pos[2]


def main(args):
    global track_id

    # Discover or use provided server IP
    if args.server:
        server_ip = args.server
        print(f"Using provided server IP: {server_ip}")
    else:
        print("Discovering Motive server on the network...")
        server_ip = discover_motive()
        if server_ip is None:
            print("ERROR: No Motive server found. Check that Motive is running and streaming.")
            print("You can also specify the IP manually with --server <ip>")
            return
        print(f"Found Motive server at {server_ip}")

    client = NatNetClient()
    client.server_ip_address = server_ip
    client.local_ip_address = "0.0.0.0"
    client.set_print_level(0)
    client.rigid_body_list_listener = receive_rigid_body_frame
    client.run()

    print("Waiting for rigid body data...")
    time.sleep(3)

    if frame_count == 0:
        print("ERROR: No data received. Check that Motive is streaming.")
        client.shutdown()
        return

    print(f"Receiving data: {frame_count} frames, rigid body IDs: {sorted(seen_ids)}")

    # Pick which rigid body to track
    if len(seen_ids) == 1:
        track_id = list(seen_ids)[0]
        print(f"Auto-selected ID {track_id}")
    else:
        while True:
            choice = input(f"Enter rigid body ID to track {sorted(seen_ids)}: ").strip()
            try:
                choice = int(choice)
                if choice in seen_ids:
                    track_id = choice
                    break
                print(f"ID {choice} not available.")
            except ValueError:
                print("Please enter a valid integer.")

    print(f"Tracking ID {track_id}. Press r to record, q to quit.")

    # Output file with timestamp and rigid body ID
    if args.outputfile:
        output_path = args.outputfile
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"dist_rb{track_id}_{timestamp}.csv"
    file = open(output_path, 'w')
    file.write('time, distance, x, y, z, recording\n')
    print(f"Writing data to {output_path}")

    # Setup plot
    fig = plt.figure()
    plt.axis([-6, 6, -6, 6])
    fig.canvas.mpl_connect('key_press_event', on_press)

    old_x = pos_x
    old_z = pos_z
    distance = 0
    glitch_count = 0
    start_time = time.time()
    pre_time = time.time()
    freq_count = 0
    freq_time = time.time()
    data_freq = 0.0

    while plt.fignum_exists(fig.number):
        h = math.hypot(pos_x - old_x, pos_z - old_z)

        if h > 1.0:
            # Glitch: jump too large, ignore but update position
            glitch_count += 1
            old_x = pos_x
            old_z = pos_z
        elif h > 0.10:
            if recording:
                distance += h
            old_x = pos_x
            old_z = pos_z

        # Track data frequency
        freq_count += 1
        now = time.time()
        if now - freq_time >= 1.0:
            data_freq = freq_count / (now - freq_time)
            freq_count = 0
            freq_time = now

        if now - pre_time > 0.1:
            current_time = now - start_time
            print(f'distance: {distance:.2f} m; time: {current_time:.2f} s; freq: {data_freq:.0f} Hz; recording: {recording}')
            pre_time = now

            data = f'{current_time}, {distance}, {pos_x}, {pos_y}, {pos_z}, {recording}\n'
            file.write(data)

            rec_str = "RECORDING" if recording else "NOT RECORDING"
            plt.title(f'Distance: {distance:.2f} m | {data_freq:.0f} Hz | {rec_str}\nPress r to record, q to quit')

            if recording:
                plt.plot(pos_z, pos_x, 'ro')
            else:
                plt.plot(pos_z, pos_x, 'bo')

        plt.pause(0.001)

    client.shutdown()
    file.close()
    print(f"Total distance: {distance:.2f} m ({glitch_count} glitches ignored)")
    print(f"Data saved to {output_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Distance counter using Optitrack NatNet stream")
    parser.add_argument('--server', default=None, help="Motive server IP (auto-discovers if not set)")
    parser.add_argument('--outputfile', type=str, default=None, help="CSV output file (default: dist_rb<id>_<timestamp>.csv)")
    args = parser.parse_args()

    main(args)
