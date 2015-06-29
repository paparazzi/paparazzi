#From inside this folder
# docker build -t flixr/pprz-dep .
# docker run -t -i flixr/pprz-dep /bin/bash
# docker export pprz-dep | gzip -c > pprz-dep.tgz
# docker import pprz-dep < pprz-dep.tgz

# Use phusion/baseimage as base image.
FROM phusion/baseimage:0.9.16
MAINTAINER Felix Ruess <felix.ruess@gmail.com>

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

# Set the env variable DEBIAN_FRONTEND to noninteractive
ENV DEBIAN_FRONTEND noninteractive

# add Paparazzi PPA
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:paparazzi-uav/ppa

# install paparazzi-dev which pull in the dependencies
# also install cross compiler and some stuff for X
RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    libcanberra-gtk-module \
    paparazzi-dev \
    paparazzi-jsbsim \
    x11-apps \
    gedit

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
