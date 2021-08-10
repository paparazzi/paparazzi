.. quickstart install

======================
Quick Install
======================

Paparazzi runs best on **Ubuntu 16.04 or higher**, so this quick installation guide is for Ubuntu users. I you have an other OS or if you want more detailled installation, see the :doc:`../installation/index_installation` page.

Open a terminal and execute each lines below. If one fails, ask for help on gitter.

Version specific prerequisites
------------------------------

**If you have Ubuntu 20.04:**

.. code-block:: bash

    sudo apt-get install python-is-python3 gcc-arm-none-eabi gdb-multiarch

**If you have Ubuntu 18.04 or lower:**

.. code-block:: bash

    sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa
    sudo apt-get install gcc-arm-embedded
    sudo apt-get update

Install Paparazzi
-----------------

Add paparazzi apt-repository and install dependencies:

.. code-block:: bash

    sudo add-apt-repository -y ppa:paparazzi-uav/ppa
    sudo apt-get update
    sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim dfu-util

Clone the repository: 

.. code-block:: bash

    cd ~
    git clone --origin upstream https://github.com/paparazzi/paparazzi.git
    cd ~/paparazzi
    git remote update -p
    sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    
*Optionnal:* checkout on a stable version:

.. code-block:: bash

    git checkout -b v5.18 upstream/v5.18

Get the submodules and build Paparazzi. This step might take a long time the first time you launch it:

.. code-block:: bash

    make -j1

.. note::
    The ``-j1`` argument may not be necessary, but if you are not familiar with paparazzi, its safer to use it. However, it will make paparazzi build much slower.
    
Finally, launch Paparazzi with

.. code-block:: bash

    ./paparazzi

If all went well the Paparazzi Center should now be running. Please continue to the next page for a guided tour.
