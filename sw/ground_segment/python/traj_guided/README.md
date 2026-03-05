# Exectute trajectories in guided mode from an input file

Run `./guided_traj.py` with the input trajectory file.

The input CSV file can be in NED or ENU frame, organized by line (1 state per line)
or by column (1 timestep per line, state elements in column).
Default is NED frame, organized in column format. Options `--transpose|--no-transpose` and `--frame {enu,ned}`
can be used to select the correct input format.

State vector the data is structured as either:
- `time, x, y, z, psi`
- `time, x, y, z, psi, vx, vy, vz, psi_d`
- `time, x, y, z, psi, vx, vy, vz, psi_d, ax, ay, az, psi_dd`

Input trajectories are interpolated and derivated to get world frame velocity and acceleration if needed

# Example trajectories

A few demo files are provided in the examples folder. They are designed for an indoor flight arena of at least 8x8x6 meters.

