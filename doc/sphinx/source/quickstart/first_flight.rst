.. quickstart first_flight

======================
Experimental Flight
======================

The easiest drone to fly with paparazzi is the **Bebop 2**, from Parrot. It is no longer produced but can still be found 2nd hand.

Using the Bebop 2 with Paparazzi
--------------------------------

Make sure to have a bebop with at least firmware v3.3.0. Update it from FreeFlight pro if needed.

- Start the Paparazzi Center (or Stop/remove all processes)
- Select the *bebop2* A/C and build it for the *ap* target
- Power up the Bebop
- Connect your laptop to the Bebop's wifi network
- Press the on/off button 4 times
- Press the "Upload" button and wait. The messages in the console should end with "DONE"
- Select the Flight UDP/Wifi session, then execute.

You should get the telemetry from the bebop.


Making the changes permanent
----------------------------

In the current state, you have to do the process all over again every time you restart the Bebop. However, you can start Paparazzi by default with the following steps:

- Power up the Bebop
- Connect your laptop to the Bebop's wifi network
- Press the on/off button 4 times
- Open a terminal and go to ``paparazzi/sw/tools/parrot``

Launch ``./bebop.py status``. You should get the current status of the drone's configuration. If it does not work, you may need to add the ``--host`` argument with the drone's IP address like so: ``./bebop.py --host 192.168.1.15 status``.

.. note::

    The default IP address of the bebop is ``192.168.42.1``. In this case, you don't need the ``--host`` argument.

Install the autostart script with ``./bebop.py install_autostart``.


Change network settings
-----------------------

You can change the network settings of the drone to make it connect to an existing WiFi network: ``./bebop.py configure_network mySSID managed dhcp``. You can replace ``dhcp`` by a fixed IP address to use static IP instead of getting the address from your external router DHCP in ``managed`` mode.


Calibration
-----------

The default values may be good enough to be able to fly, but it is recommanded that you calibrate the drone before flying. Go to the :doc:`../tutorials/beginner/sensor_calibration` page to learn how to do that.


Connect a Joystick
------------------

.. note::
    This part is optionnal but it is recommended to connect a joystick to have an easy way to control your drone, if something unexpected should happen.

Learn how to connect a joystick to your laptop to control your drone on this page: :doc:`../tutorials/beginner/use_joystick`




