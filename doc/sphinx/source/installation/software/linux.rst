.. installation software_installation linux

======================
Linux
======================

Paparazzi is very easily installed on any laptop or workstation running Ubuntu, Debian (or any of their derivatives).

The steps required to install the software needed to be able to let your UAS fly

* Install the basic Paparazzi dependencies and the ARM cross compiling toolchain
* Download the source code from the source repository
* Allow access to your PC hardware connection by adding appropriate Udev rules
* Compile the binaries from the sources and launch the software

Users of other Linux flavors than a recent Ubuntu or Debian and anyone needing manual control of each individual package can install them independently.

For the impatient
=================

For Ubuntu add the paparazzi-uav ppa sudo add-apt-repository ppa:paparazzi-uav/ppa and install the paparazzi-dev package.

TODO: link to the video tutorials


Installation of dependencies
============================

Ubuntu
______

Binary packages for Ubuntu are available for the i386, amd64 and armhf architectures.

Add the installation sources for the Paparazzi software packages. Run from a terminal:

sudo add-apt-repository ppa:paparazzi-uav/ppa

Then update the systems package inventory and install the main Paparazzi software dependencies. This will take some time.

sudo apt-get update 
sudo apt-get install paparazzi-dev

Debian

Binary packages for Debian are available for the i386 and amd64 architectures. armhf packages seem to be currently not supported by the OpenSUSE build service.

For Debian Wheezy (7.0), Jessie (8.0) and Stretch (9.0) packages are built using the Open Build Service (OBS) on OpenSUSE Build Service project home:flixr:paparazzi-uav

Install paparazzi-dev

First add the key:

wget -q "http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_9.0/Release.key" -O- | sudo apt-key add -

Add the appropriate repo, depending on your Debian version to sources.list

echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_9.0/ ./" | tee -a /etc/apt/sources.list
echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_8.0/ ./" | tee -a /etc/apt/sources.list
echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/ ./" | tee -a /etc/apt/sources.list

Update the systems package inventory and install the main Paparazzi software dependencies.

sudo apt-get update 
sudo apt-get install paparazzi-dev

