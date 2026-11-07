# MAIN README

Paparazzi UAS
=============
[![Build Status](https://paparazziuav.semaphoreci.com/badges/paparazzi/branches/master.svg?style=shields&key=d3a59143-a357-434e-89b8-057f15ed8dd2)](https://paparazziuav.semaphoreci.com/projects/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)
<a href="https://scan.coverity.com/projects/paparazzi-paparazzi">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/4928/badge.svg"/>
</a>

Paparazzi is a free open source software package for Unmanned (Air) Vehicle Systems.
For many years, the system has been used successfuly by hobbyists, universities and companies all over the world, on vehicles of various sizes (11.9g to 25kg).
Paparazzi supports fixed wing, rotorcraft, hybrids, flapping vehicles and it is even possible to use it for boats and surface vehicles.

Documentation is available here https://paparazzi-uav.readthedocs.io/en/latest/

More docs is also available on the wiki http://wiki.paparazziuav.org

To get in touch, subscribe to the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi), the IRC channel (freenode, #paparazzi) and Gitter (https://gitter.im/paparazzi/discuss).

Required software
-----------------

Paparazzi runs on Ubuntu 20.04 to 26.04. Installation: https://paparazzi-uav.readthedocs.io/en/latest/getting_started/install.html

Quick start:

```
git clone https://github.com/paparazzi/paparazzi.git
cd ./paparazzi
./install.sh
./paparazzi
```

`./install.sh` opens a window (or a menu in the terminal) to choose what to install: the Paparazzi apt repository
([paparazzi-uav PPA](https://launchpad.net/~paparazzi-uav/+archive/ppa)) with **paparazzi-dev** (everything to build
Paparazzi and run the ground segment), the compilers, the simulators (JSBSim, Gazebo), the ground station and more.
`./install.sh --help` shows how to use it without menu, e.g. in scripts and Docker images.


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Bixler" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" in Operation tab and click "Start Session".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same as in simulation !
