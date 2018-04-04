.. quickstart main_quickstart install

======================
Quick Install
======================


Paparazzi is very easily installed on any laptop or workstation running the `Ubuntu Linux OS`_ or virtually any `Debian`_ based `Linux`_ or Apple Macintosh running `Mac OS X`_. There is also work being done to port Paparazzi to Windows.

Several steps are required to fully set up paparrazi but if you'r one of the one-line lovers, this part is for you.

To get the latest Paparazzi up and running on your **Ubuntu 12.04 or higher OS**, make sure you have a working internet connection, then just copy and paste the text below into your terminal and press [enter] ... and wait a while...

.. code-block:: bash

	sudo add-apt-repository -y ppa:paparazzi-uav/ppa && sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && sudo apt-get update && \
	sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim gcc-arm-embedded && cd ~ && git clone --origin upstream https://github.com/paparazzi/paparazzi.git && \
	cd ~/paparazzi && git remote update -p && \
	git checkout -b v5.12 upstream/v5.12 && sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && \
	make && ./paparazzi

If all went well the Paparazzi Center should now be running... **skip** the rest of this page and go fly!

If you are new you'll need to do some more things before you go fly like configuring your XML definition file detailing your airframe configuration. There is help here for that: `Airframe_Configuration`_

In case you have no autopilot hardware yet, no problem, you can get hardware `here`_ or just buy a ready to fly aircraft that can run Paparazzi Software like the `Parrot Drones`_ and run Paparazzi on your Parrot `ARDRone2`_, `Bebop`_ and Bebop2 (soon the Disco drone).

.. _`Ubuntu Linux OS`: https://www.ubuntu.com/
.. _`Debian`: https://www.debian.org/
.. _`Linux`: https://en.wikipedia.org/wiki/Linux
.. _`Mac OS X`: https://en.wikipedia.org/wiki/MacOS
.. _`Airframe_Configuration`: http://wiki.paparazziuav.org/wiki/Installation
.. _`here`: http://wiki.paparazziuav.org/wiki/Get_Hardware
.. _`Parrot Drones`: https://www.parrot.com/fr/drones
.. _`ARDRone2`: http://wiki.paparazziuav.org/wiki/AR_Drone_2
.. _`Bebop`: http://wiki.paparazziuav.org/wiki/Bebop
