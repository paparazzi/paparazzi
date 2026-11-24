# Parrot Bebop: cross-compilers

The Bebop 1 and 2 (and Disco) autopilot `ap.elf` is cross-compiled for ARMv7 Linux, soft-float ABI
(`-mfloat-abi=softfp`, like the Parrot firmware), and linked statically.

The compiler is chosen in `conf/Makefile.arm-linux-toolchain`:
- `PREFIX` set by the airframe (`<configure name="PREFIX" value="/path/to/arm-...-"/>`): exactly that
  compiler;
- otherwise `arm-linux-gnueabi-` from the Ubuntu package `gcc-arm-linux-gnueabi`;
- otherwise a CodeSourcery toolchain in `/usr/local/codesourcery`.

Known to work: Ubuntu 22.04, 24.04 and 26.04 `gcc-arm-linux-gnueabi`, Arm GNU Toolchain 10.3-2021.07
and the Parrot 2012.03 toolchain (GCC 4.6.3, no OpenCV 4). Compiler history: paparazzi/paparazzi#3762.

`time_t` must stay 32-bit: the Parrot Linux 3.4 kernel only knows the 32-bit `time_t` V4L2 ioctls, and
with a 64-bit `time_t` `ap.elf` hangs at camera start. From Ubuntu 24.04 on, the cross-compiler
defaults to 64-bit, so the board makefiles add `-U_TIME_BITS`.

OpenCV for the Bebop (`sw/ext/opencv_bebop`) must be built with the same compiler, ABI and `time_t`.
