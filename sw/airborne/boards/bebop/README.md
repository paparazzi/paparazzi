# Parrot Bebop: cross-compilers

The Bebop 1 and 2 autopilot (`ap.elf`) is cross-compiled for ARMv7 Linux and linked statically
(`conf/boards/bebop.makefile`, `conf/boards/bebop2.makefile`). The compiler is chosen in
`conf/Makefile.arm-linux-toolchain`:

- `PREFIX` from the airframe (`<configure name="PREFIX" value="/path/to/arm-...-"/>`, ending with `-`)
  is used as it is: that exact compiler, never another one;
- otherwise `arm-linux-gnueabi-` from the `PATH` (Ubuntu package `gcc-arm-linux-gnueabi`);
- otherwise a CodeSourcery toolchain in `/usr/local/codesourcery`.
- otherwise, if docker is installed, a docker image that has this compiler (`docker/toolchains.txt`):
  only the compiler commands run in it, the first time the image is built from its Dockerfile
  (e.g. `docker/bebop-toolchain` with the Parrot 2012.03 and Ubuntu 22.04 compilers).
  `USE_DOCKER_TOOLCHAIN=0` turns this off.

The Bebop does not use `HARD_FLOAT`: the code uses the FPU (`-mfloat-abi=softfp -mfpu=vfp`) with the
soft-float calling convention, like the Parrot firmware. OpenCV for the Bebop (`sw/ext/opencv_bebop`,
`install_arm`) must be built with the same compiler and ABI.

| Compiler | GCC | ABI | Source | Result on the Bebop |
|---|---|---|---|---|
| Parrot 2012.03 (Sourcery CodeBench Lite, `arm-none-linux-gnueabi-`) | 4.6.3 | soft-float | [parrot-opensource/toolchains](https://github.com/parrot-opensource/toolchains), installs in `/opt/arm-2012.03` | original Parrot toolchain for Bebop 1/2, used in the 2017 MAV course and for OpenCV 3.2 ([tudelft/opencv_bebop#4](https://github.com/tudelft/opencv_bebop/pull/4)) |
| Parrot 2016.02-linaro (`arm-linux-gnueabihf-`) | 4.9.4 | hard-float | [parrot-opensource/toolchains](https://github.com/parrot-opensource/toolchains) | segfault with `HARD_FLOAT` on the Disco ([#2389](https://github.com/paparazzi/paparazzi/pull/2389)); GCC 4.9 has known problems ([#2439](https://github.com/paparazzi/paparazzi/issues/2439)) |
| Ubuntu `gcc-arm-linux-gnueabi` 4.7 / 4.8 | 4.7, 4.8 | soft-float | apt (2017) | OpenCV 3.3.1 does not compile with 4.7 ([tudelft/opencv_bebop#9](https://github.com/tudelft/opencv_bebop/issues/9)) |
| Ubuntu 22.04 `gcc-arm-linux-gnueabi` | 11.4 | soft-float | apt | works ([#3596](https://github.com/paparazzi/paparazzi/issues/3596)) |
| Ubuntu 24.04 `gcc-arm-linux-gnueabi` | 13 | soft-float | apt | `ap.elf` freezes after start-up ([#3596](https://github.com/paparazzi/paparazzi/issues/3596)); this compiler uses a 64-bit `time_t` by default, so the Bebop boards set `-U_TIME_BITS` (32-bit `time_t` like the Parrot firmware) |
| Arm GNU Toolchain 10.3-2021.07 (`arm-none-linux-gnueabi-`) | 10.3 | soft-float | [developer.arm.com](https://developer.arm.com/-/media/Files/downloads/gnu-a/10.3-2021.07/binrel/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabi.tar.xz) | workaround for Ubuntu 24.04 with `PREFIX` ([#3596](https://github.com/paparazzi/paparazzi/issues/3596)) |
