.. installation main_installation software

======================
Software
======================

Downloading the software
________________________

The Paparazzi source code is hosted on Github. While you can download it as a tarball from https://github.com/paparazzi/paparazzi/releases, it is recommended to clone the repository with git.

From the directory of your choice type:

::

    git clone --origin upstream https://github.com/paparazzi/paparazzi.git

If you don't want to work with the latest master branch (it may contain some unstable features), check out the released stable version branch:

::

    cd paparazzi
    git checkout v5.16


OS-specific instructions
________________________

After this point, it is required to install the needed libraries. The procedure depends on the operating system you are using.
Please follow the instruction below:

.. toctree ::
	:maxdepth: 2
	
	linux
	mac_os
	windows

Buiding and running the code
____________________________

Finally, build the ground station tools:

::

    make
