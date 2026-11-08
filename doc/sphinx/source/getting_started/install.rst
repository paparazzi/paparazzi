.. quickstart install

============
Installation
============

Paparazzi runs on **Ubuntu 20.04 to 26.04**; this guide is for Ubuntu users.
On other Linux versions the installer shows what it can install there.
On Windows, see `Install on Windows`_.

If something fails, ask for help on `Github <https://github.com/paparazzi/paparazzi/discussions>`_
and add the log ``var/install.log``.


Get Paparazzi and install
-------------------------

.. code-block:: bash

    sudo apt-get install git
    cd ~
    git clone --origin upstream https://github.com/paparazzi/paparazzi.git
    cd ~/paparazzi
    ./install.sh

With a desktop, ``./install.sh`` opens a window, otherwise a menu in the terminal (or use ``./install.sh --text``).
Choose what to install; the parts needed to build and run Paparazzi are selected, installed parts are marked.
The installer is ``install.py`` (Python 3, installed first when it is missing): every command is shown before it
runs, ``sudo`` asks for your password. The components and what they install are listed under
`Manual installation`_ below.

Finally, launch Paparazzi with

.. code-block:: bash

    ./paparazzi

If all went well the Paparazzi Center should now be running. Please continue to the next page for a guided tour.

The installer can be started again at any time to add or update components.


Python packages
---------------

The Paparazzi Center and the tools are written in Python. The installer asks where their packages go:

- **system** (default up to Ubuntu 24.04): the Ubuntu ``python3-*`` packages, most of them come with ``paparazzi-dev``
  already. Since Ubuntu 23.04 pip can't install in the system Python, the few packages that Ubuntu doesn't have are then
  reported.
- On **Ubuntu 26.04** the default is **venv**: the build needs empy 3 (DroneCAN code generator), Ubuntu 26.04 has empy 4.
- **venv**: the same Ubuntu packages, plus the virtual environment ``pprzEnv`` in the paparazzi folder for the packages
  that Ubuntu doesn't have. ``./paparazzi`` and the terminals of the Paparazzi Center use ``pprzEnv`` when it exists,
  no need to activate it.
- **conda**: pip in the active conda environment (``conda activate <env>`` before starting the installer).


Without menu (scripts, Docker)
------------------------------

Give the components to install, ``default`` for the ones selected by default:

.. code-block:: bash

    ./install.sh --yes core arm jsbsim build
    ./install.sh --yes --python=venv default gazebo

``./install.sh --list`` shows the components and whether they are installed, ``./install.sh --help`` all options.
The installer also runs as root without ``sudo`` and without terminal, as in the Docker images (``docker/``).


Manual installation
-------------------

The original installation commands still work. Add the paparazzi apt-repository and install the dependencies:

.. code-block:: bash

    sudo add-apt-repository -y ppa:paparazzi-uav/ppa
    sudo apt-get update
    sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim dfu-util pprzgcs
    sudo apt-get install python-is-python3 gcc-arm-none-eabi gdb-multiarch

.. note:: If you are on Ubuntu 24.04, please also install ``liblablgtk2-ocaml-dev`` in order to use the plot and messages tools.

Clone the repository:

.. code-block:: bash

    cd ~
    git clone --origin upstream https://github.com/paparazzi/paparazzi.git
    cd ~/paparazzi
    git remote update -p
    sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules

Get the submodules and build Paparazzi. This step might take a long time the first time you launch it:

.. code-block:: bash

    make -j1

.. note::
    The ``-j1`` argument may not be necessary, but if you are not familiar with paparazzi, its safer to use it. However, it will make paparazzi build much slower.

To see what the installer would run on your computer, without running it:

.. code-block:: bash

    ./install.sh --dry-run --yes default

The lists below show what each component installs on Ubuntu 22.04, 24.04 and 26.04.
They are made from the installer itself (``sw/tools/install/components.txt``) when this documentation is built.
With the Python packages in the system (``--python=system``), the modules marked *pip only* are installed with
``pip --user`` on Ubuntu 22.04 and are not installed on newer versions (choose venv if you need them).
With conda, all the Python modules come from pip.

.. include:: install_packages.inc


Install on Windows
------------------

Paparazzi do not run natively on windows, but you can run it in the *Windows Subsystem for Linux*.

Install the Windows Subsystem for Linux and the Ubuntu distribution by following the Microsoft documentation,
then install paparazzi like on a regular Ubuntu (the installer shows its menu in the terminal).

In older versions of WLS, you may need to install an X server like `VcXsrv <https://sourceforge.net/projects/vcxsrv/>`_
or `Xming <https://sourceforge.net/projects/xming/>`_.
You will then need to set the *DISPLAY* environment variable to point at the X server running on your Windows 10 PC :

.. code-block:: bash

    export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0

.. note::

    The **Windows Terminal** application is very usefull as it can have multiple tabs and allows you to easily switch between Bash and Powershell.
