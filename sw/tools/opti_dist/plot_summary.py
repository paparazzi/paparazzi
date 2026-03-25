#!/usr/bin/env python3
"""Plot summary of a dist.py recording: distance over time, recording regions, totals."""
import csv
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def main(csvfile):
    times = []
    distances = []
    recording = []

    with open(csvfile) as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            times.append(float(row[0]))
            distances.append(float(row[1]))
            recording.append(row[5].strip() == 'True')

    # Compute total distance and total time spent recording
    total_distance = distances[-1] if distances else 0.0
    total_rec_time = 0.0
    for i in range(1, len(times)):
        if recording[i]:
            total_rec_time += times[i] - times[i - 1]

    print(f"File: {csvfile}")
    print(f"Total distance: {total_distance:.2f} m")
    print(f"Total recording time: {total_rec_time:.1f} s")
    print(f"Total session time: {times[-1]:.1f} s")

    fig, ax = plt.subplots()
    ax.plot(times, distances, 'b-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title(f'{csvfile}\nTotal distance: {total_distance:.2f} m | Recording time: {total_rec_time:.1f} s')

    # Shade recording regions
    in_region = False
    region_start = 0
    for i, rec in enumerate(recording):
        if rec and not in_region:
            region_start = times[i]
            in_region = True
        elif not rec and in_region:
            ax.axvspan(region_start, times[i], alpha=0.2, color='red')
            in_region = False
    if in_region:
        ax.axvspan(region_start, times[-1], alpha=0.2, color='red')

    rec_patch = mpatches.Patch(color='red', alpha=0.2, label='Recording active')
    ax.legend(handles=[rec_patch])
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot summary of a dist.py recording")
    parser.add_argument('csvfile', help="CSV file from dist.py")
    args = parser.parse_args()
    main(args.csvfile)
