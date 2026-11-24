# EKF2 of PX4

Copy of the EKF2 estimator of [PX4](https://github.com/PX4/PX4-Autopilot) and the PX4 libraries it needs,
used by the `ins_ekf2` module. The EKF2 is only maintained inside PX4-Autopilot, so instead of a submodule
only these parts are copied (the PX4 commit is in `PX4_VERSION`). The PX4 code is BSD licensed, see `LICENSE`.

- `EKF/`: the estimator (`src/modules/ekf2/EKF`), with the generated equations of the derivation
- `lib/`: PX4 libraries (matrix, mathlib, geo, lat_lon_alt, world_magnetic_model, atmosphere, ringbuffer, GNSS checks)
- `generated/`: the uORB message structs the EKF uses as data types, generated from the PX4 `.msg` files
- `stubs/`: minimal replacements of PX4 platform headers (not touched by the sync)

To update to another PX4 version (commit, branch or tag):

```bash
sw/ext/px4_ekf2/sync_px4_ekf2.py <PX4 commit>
```

Without an argument the version in `PX4_VERSION` is copied again. Check `ins_ekf2.cpp` and
`conf/modules/ins_ekf2.xml` (source list, `CONFIG_EKF2_*` features) against the changes in PX4.
