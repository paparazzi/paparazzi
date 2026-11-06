# EKF2 replay

Replay the EKF2 of `ins_ekf2` offline on the logged inputs of a flight, to tune it.

## 1. Log the EKF inputs

Set in the airframe (in the `ins` module):

```xml
<module name="ins" type="ekf2">
  <configure name="INS_EKF2_LOG_INPUTS" value="TRUE"/>
</module>
```

Every sample given to the EKF (IMU at full rate, magnetometer, baro, GNSS, airspeed, range finder,
optical flow, external vision, flags), every update and all EKF parameters are then logged as
`EKF2_INPUT` messages in the flight recorder log on the SD card (`INS_EKF2_LOG_DEVICE`), about
25 kB/s at a 500 Hz IMU. Every 10th update the EKF output is logged too, to check the replay.

A GNSS sample also logs why it is (not) usable: in its flags (the 5th integer) bit 4 is set when the
strict GNSS checks decide, bit 5 when they passed, and bits 6-8 are the at rest, armed and in air
inputs of the checks; the 6th and 7th integers are the checks failed at the strict and relaxed
thresholds (`GNSS_CHECK_*` bits, e.g. 0x20 horizontal drift); the 8th to 10th integers are the time
given to the checks (ms) and the time since their last pass and last failure (ms, -1 when never), the
11th and 12th the enabled checks and the required pass time (ms) of the checks (constant).

The EKF only starts once the log is open, so the replay sees everything the EKF saw.
`SDLOG_START_DELAY` is therefore 2 s by default with this option (the build tells you when it is set
higher). Without SD card the EKF starts after `INS_EKF2_LOG_WAIT` (60 s) without logging.

The `flight_recorder` module is needed. On Linux (Bebop) and in NPS it logs to files with the same
format as the SD card; in NPS that is `var/logs/sdlog/<AIRCRAFT>/FLIGHT_RECORDER/fr_<n>.LOG`
(load `flight_recorder` outside the `ap` target to also log in NPS).

## 2. Replay

```bash
make -C sw/tools/ekf2_replay
sw/tools/ekf2_replay/ekf2_replay fr_0094.LOG -o orig.csv
sw/tools/ekf2_replay/ekf2_replay fr_0094.LOG -o tuned.csv --set MAG_NOISE=0.08 --set GPS_P_NOISE=0.7
sw/tools/ekf2_replay/ekf2_replay fr_0094.LOG --list   # logged parameters
```

The input is the PPRZLOG file of the SD card (or the `.tlm` copy of `sd2log --tlm`); the binary
values are used, so the replay gets exactly the values the EKF got. The parameters are the logged
ones, `--set` changes one by its PX4 name without `EKF2_` (see the
[PX4 parameter reference](https://docs.px4.io/main/en/advanced_config/parameter_reference.html)).

The replay is compared with the logged EKF output: with unchanged parameters the difference is
zero for a log of NPS, and very small for a flight (the autopilot and the PC round floats
differently).

The CSV has per update (every 10th, `--every N`): position, velocity, attitude, accuracies, control
and fault status, innovation test ratios, wind, biases, height above ground and baro bias.

## 3. Plot

```bash
sw/tools/ekf2_replay/ekf2_plot.py orig.csv tuned.csv
```

Plots the estimates on top of each other and prints the mean innovation test ratios
(well below 1 for a well tuned EKF).
