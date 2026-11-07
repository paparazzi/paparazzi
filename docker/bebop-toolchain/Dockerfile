# Cross-compilers for the Parrot Bebop, used by the Paparazzi build for the compile step
# when the compiler that the airframe asks for is not installed on the computer.
#  - Ubuntu 22.04 arm-linux-gnueabi-gcc 11.4 (the default PREFIX arm-linux-gnueabi-)
#  - Parrot toolchain 2012.03 (GCC 4.6.3) in /opt/arm-2012.03 (PREFIX /opt/arm-2012.03/bin/arm-none-linux-gnueabi-)
# See docker/toolchains.txt
FROM ubuntu:22.04

ARG PARROT_2012_03=https://firmware.parrot.com/Toolchains/parrot-tools-linuxgnutools-2012.03_1.0.0-1_i386.deb

RUN dpkg --add-architecture i386 \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      make gcc-arm-linux-gnueabi g++-arm-linux-gnueabi libc6:i386 zlib1g:i386 curl ca-certificates \
 && curl -fsSL -o /tmp/parrot.deb "$PARROT_2012_03" \
 && dpkg -i /tmp/parrot.deb \
 && rm /tmp/parrot.deb \
 && rm -rf /opt/arm-2012.03/share/doc \
 && apt-get purge -y curl \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*
