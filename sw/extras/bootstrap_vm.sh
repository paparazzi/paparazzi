#!/usr/bin/env bash

export DEBIAN_FRONTEND=noninteractive
apt-get update

# install a desktop environment and login manager
apt-get install -y xfce4 gdm
dpkg-reconfigure gdm

# default precise32 box doesn't have the add-apt-repository command, get it:
apt-get install -y python-software-properties

add-apt-repository ppa:paparazzi-uav/ppa
add-apt-repository ppa:flixr/gcc-arm-embedded
apt-get update
apt-get install -y paparazzi-dev gcc-arm-none-eabi libcanberra-gtk-module


# setup some git aliases for the vagrant user
su -c /home/vagrant/paparazzi/sw/extras/setup_git_aliases.sh vagrant
