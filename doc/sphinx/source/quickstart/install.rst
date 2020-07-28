.. quickstart main_quickstart install

======================
Quick Install
======================

Paparazzi runs best on **Ubuntu 16.04 or higher**, so this quick installation guide is for Ubuntu users. I you have an other OS or if you want more detailled installation, see the :doc:`../installation/index_installation` page.

Open a terminal and execute each lines below. If one fails, ask for help on gitter.

Add paparazzi repositories and install dependencies:

.. code-block:: bash

	sudo add-apt-repository -y ppa:paparazzi-uav/ppa
	sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa
	sudo apt-get update
	sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim gcc-arm-embedded

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

	git checkout -b v5.16 upstream/v5.16

Get the submodules and build Paparazzi. This step might take a long time the first time you launch it:

.. code-block:: bash

	make
	
	
Finally, launch Paparazzi with

.. code-block:: bash

    ./paparazzi
    
.. note::
   If it doesn't work, the previous step might have failed. In that case, recompile with with ``make -j1``, then try again to launch Paparazzi.

If all went well the Paparazzi Center should now be running. Please continue to the next page for a guided tour.
