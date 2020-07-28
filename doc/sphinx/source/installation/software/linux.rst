.. installation software_installation linux

======================
Linux
======================

.. warning::

    This guide may be outdated. See http://wiki.paparazziuav.org/wiki/Installation.

Paparazzi is very easily installed on any laptop or workstation running Ubuntu, Debian (or any of their derivatives).

The steps required to install the software needed to be able to let your UAS fly

* Install the basic Paparazzi dependencies and the ARM cross compiling toolchain
* Download the source code from the source repository
* Allow access to your PC hardware connection by adding appropriate Udev rules
* Compile the binaries from the sources and launch the software

Users of other Linux flavors than a recent Ubuntu or Debian and anyone needing manual control of each individual package can install them independently.

For the impatient
=================

For Ubuntu add the `paparazzi-uav ppa <https://launchpad.net/~paparazzi-uav/+archive/ubuntu/ppa>`_ (*sudo add-apt-repository ppa:paparazzi-uav/ppa*)
and install the paparazzi-dev package.


.. raw:: html

    <iframe width="425" height="239" src="https://www.youtube.com/embed/SshFJrBuku8" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>



Installation of dependencies
============================
______
Ubuntu
______

Binary packages for Ubuntu are available for the i386, amd64 and armhf architectures.

Add the installation sources for the Paparazzi software packages. Run from a terminal:

::

    sudo add-apt-repository ppa:paparazzi-uav/ppa

Then update the systems package inventory and install the main Paparazzi software dependencies. This will take some time.

::

    sudo apt-get update 
    sudo apt-get install paparazzi-dev

______
Debian
______

Binary packages for Debian are available for the i386 and amd64 architectures. armhf packages seem to be currently not supported by the OpenSUSE build service.

For Debian Wheezy (7.0), Jessie (8.0) and Stretch (9.0) packages are built using the Open Build Service (OBS) on OpenSUSE Build Service project home:flixr:paparazzi-uav

First add the key:

::

    wget -q "http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_9.0/Release.key" -O- | sudo apt-key add -

Add the appropriate repo, depending on your Debian version to sources.list

::

    echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_9.0/ ./" | tee -a /etc/apt/sources.list
    echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_8.0/ ./" | tee -a /etc/apt/sources.list
    echo "deb http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/ ./" | tee -a /etc/apt/sources.list

Update the systems package inventory and install the main Paparazzi software dependencies.

::

    sudo apt-get update 
    sudo apt-get install paparazzi-dev


______________________
ARM embedded toolchain
______________________

For current Paparazzi versions (v5.0 and above) the gcc-arm-embedded toolchain is recommended, which also supports the STM32F4 with FPU (hardware floating point).
gcc-arm-none-eabi as Debian/Ubuntu package

This is the recommended method

Note that there are actually two different toolchains available!

* ARM gcc-arm-embedded toolchain with Debian package name gcc-arm-embedded

  * includes libstdc++ and newlib-nano

* Debian gcc-arm-none-eabi toolchain

  * does not include libstdc++
  * does not include newlib-nano

Both toolchains should work for most use-cases (if you don't need C++ or nano specs), although the ARM gcc-arm-embedded toolchain is better tested.

gcc-arm-embedded toolchain
__________________________

This is the recommended toolchain

On most Ubuntu versions the gcc-arm-embedded toolchain can be installed as a debian package from the ppa:

::

    sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
    sudo apt-get update
    sudo apt-get install gcc-arm-embedded

Previously there was a PPA by terry.guo that contained this toolchain under the package name gcc-arm-none-eabi

gcc-arm-none-eabi Debian toolchain
__________________________________

Current Debian (jessie) and Ubuntu (14.04 trusty and later) releases have the gcc-arm-none-eabi package in the official repositories (universe), and can be installed with:

::

    sudo apt-get update
    sudo apt-get install gcc-arm-none-eabi gdb-arm-none-eabi

