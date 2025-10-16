.. quickstart install

============
Installation
============

Paparazzi runs best on **Ubuntu 20.04 or higher**, so this quick installation guide is for Ubuntu users.

If you want to run it on windows, see the instructions: `Install on Windows`_.

Open a terminal and execute each lines below. If one fails, ask for help on `Github <https://github.com/paparazzi/paparazzi/discussions>`_.


Install Paparazzi
-----------------

Add paparazzi apt-repository and install dependencies:

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
    
Finally, launch Paparazzi with

.. code-block:: bash

    ./paparazzi

If all went well the Paparazzi Center should now be running. Please continue to the next page for a guided tour.

Install on Windows
------------------

Paparazzi do not run natively on windows, but you can run it in the *Windows Subsystem for Linux*.

Install the Windows Subsystem for Linux and the Ubuntu distribution by following the Microsoft documentation,
then install paparazzi like on a regular Ubuntu.

In older versions of WLS, you may need to install an X server like `VcXsrv <https://sourceforge.net/projects/vcxsrv/>`_
or `Xming <https://sourceforge.net/projects/xming/>`_.
You will then need to set the *DISPLAY* environment variable to point at the X server running on your Windows 10 PC :

.. code-block:: bash

    export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0

.. note::

    The **Windows Terminal** application is very usefull as it can have multiple tabs and allows you to easily switch between Bash and Powershell.
