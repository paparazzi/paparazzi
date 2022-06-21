.. user_guide simulation nps

.. _nps:

===========================
NPS
===========================

NPS (New Paparazzi Simulator) is a simulator with sensor and vehicle models that can use different FDM backends. 

By default, :ref:`jsbsim` is used as FDM (FlightDynamicModel), which allows for fairly complex airframes. JSBSim can be replaced by the FDM of your choice, 
such as :ref:`gazebo`, for better visualization.

NPS is capable of simulating rotorcraft and fixedwing airframes, with the possibility to add more complex aircrafts/hybrids if a proper model 
is built using one of the FDM backends.

Configure and Build
--------------------------

Add the nps target to your airframe with the fdm you want to use:

.. code-block:: xml

  <firmware name="rotorcraft or fixedwing">
    <target name="nps" board="pc">
      <module name="fdm"   type="jsbsim"/>
    </target>
    ...
  </firmware>

Then depending on the aircraft you want to simulate add a NPS simulator section which defines the model, actuators and sensor parameters used:

E.g for a simple quadrotor:

.. code-block:: xml

  <section name="SIMULATOR" prefix="NPS_">
    <define name="ACTUATOR_NAMES"  value="front_motor, right_motor, back_motor, left_motor" type="string[]"/>
    <define name="JSBSIM_MODEL" value="simple_quad" type="string"/>
    <define name="SENSORS_PARAMS" value="nps_sensors_params_default.h" type="string"/>
  </section>

The full list of available parameters for the simulator is:

+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| PARAMETER              | DESCRIPTION                                                                                                                                                                                                                                                                                                    |
+========================+================================================================================================================================================================================================================================================================================================================+
| NPS_ACTUATOR_NAMES     | mapping of the motors defined in the ``MOTOR_MIXING`` section to the actuators in the JSBSim model (the order is important, also make sure that your motors in JSBSim spin in the same direction as your real ones)                                                                                            |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| NPS_SENSORS_PARAMS     | the parameter file for the sensor simulation (noise/delay) under ``conf/simulator/nps/``                                                                                                                                                                                                                       |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| NPS_JSBSIM_MODEL       | name of the JSBSim model in ``conf/simulator/jsbsim/aircraft/`` (e.g. simple_quad), if not defined it defaults to AIRCRAFT_NAME                                                                                                                                                                                |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| NPS_JSBSIM_INIT        | the xml file containing the initial conditions (location, attitude, wind) for JSBSim in ``conf/simulator/jsbsim/aircraft/``. This define is optional and if not specified the initial position of the aircraft will be set to the flight plan location. Prior to v5.1 this was called ``INITIAL_CONDITITONS``. |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| NPS_JSBSIM_LAUNCHSPEED | if defined this sets an initial launchspeed in m/s for fixedwings, available since ``v5.1.0_testing-54-g2ac094f``.                                                                                                                                                                                             |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| NPS_JS_*               | Joystick mappings                                                                                                                                                                                                                                                                                              |
+------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

Running the Simulation
--------------------------

The most convenient way to start the simulation is via the Simulation session from the :ref:`paparazzicenter`. 
Just select e.g. the Quad_LisaM_2 example airframe and start the Simulation session with the simulator, GCS and server.

If you have added ``PAPARAZZI_HOME`` AND ``PAPARAZZI_SRC`` to the environmental variables of your terminal (See Setting up environment variables), 
you can also start it via the generic simulation launcher:

.. code-block:: C

    sw/simulator/pprzsim-launch --aircraft Quad_LisaM_2 --type nps

Options
^^^^^^^^^^^^^^^^^^^^^

Start the simulator with the *--help* option to list them all.

+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OPTION             | DESCRIPTION                                                                                                                                                                                                     |
+====================+=================================================================================================================================================================================================================+
| ``--ivy_bus``      | Set ivy bus broadcast address to use (default 127.255.255.255, 224.255.255.255 on OSX)                                                                                                                          |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--rc_script``    | Execute script with specified number to emulate RC input.                                                                                                                                                       |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--js_dev``       | Use joystick for radio control (specify index, normally 0),                                                                                                                                                     |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--spektrum_dev`` | Spektrum device to use for radio control (e.g. ``/dev/ttyUSB0``)                                                                                                                                                |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--fg_host``      | Host for FlightGear visualization (e.g. 127.0.0.1)                                                                                                                                                              |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--fg_port``      | Port on FlightGear host to connect to (Default: 5501)                                                                                                                                                           |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``--ivy_bus``      | FlightGear time offset in seconds (e.g. 21600 for 6h), this is useful if it is currently night at the location you are flying and you want to add an offset to fly in daylight. (Since v4.9_devel_413-g9d55d6f) |
+--------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

Typical Simulation
^^^^^^^^^^^^^^^^^^^^^

In general you go through the same steps as with the real aircraft:

- It should start on the ground and you have to wait a few seconds until the AHRS is aligned.
- If you have a (simulated) RC, you can now arm the motors and fly around in manual.
- To fly autonomously, make sure your AUTO2 mode is NAV, you can change it in the GCS->settings->system->auto2.

  - Switch to it if you are using an RC, otherwise you should already be in this mode.
  - Arm your motors: either via the resurrect button or by going to the Start Motors block of the Flight Plan.
  - Takeoff: via the takeoff button or the corresponding flight plan block.

- Execute your flight code.

Pausing or running the sim at a different speed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you start the simulation from a terminal, hit CTRL-z to pause it. You can then enter a different time factor (default 1.0) 
to make the simulation run slower or faster than real-time. Hit enter to resume the simulation or CTRL-z again to suspend it 
like any normal Unix process (use the fg (foreground) command to un-suspend it again).

Use a Joystick
^^^^^^^^^^^^^^^^^

You can use a joystick (or connect your RC transmitter as a joystick) to control the quad in the simulator.

.. code-block:: C

    sw/simulator/pprzsim-launch --aircraft Quad_LisaM_2 --type nps --js_dev 0

or directly calling the nps simsitl binary:

.. code-block:: C

    ./var/Quad_LisaM_2/nps/simsitl --js_dev 0

Joystick support uses the Simple DirectMedia Layer (SDL) library. Rather than specifying an input device name as one normally does on Linux, 
you just supply an index value (0, 1, 2,...) of the device you wish to use. Typically, the order of devices is the order in which you plugged 
them into your computer. The sim will display the name of the device being used to double check. If the -j option is used with no argument, 
the sim defaults to using device on index 0 (which is usually correct if you have only one joystick attached).

Also see Joystick#Calibration.

Troubleshooting
^^^^^^^^^^^^^^^^^^

- If you get an error like "JSBSim failed to open the configuration file: ``(null)/conf/simulator/jsbsim/aircraft/BOOZ2_A1.xml"``, you need to set 
  your ``$PAPARAZZI_SRC`` and ``$PAPARAZZI_HOME`` environment variables. Add the following to your .bashrc, change paths according to where you put Paparazzi. 
  Open a new terminal and launch the sim again.

.. code-block:: C

    export PAPARAZZI_SRC=~/paparazzi
    export PAPARAZZI_HOME=~/paparazzi

- If you did not install the jsbsim package your JSBSim installation under ``/opt/jsbsim`` will be used and you will have to set your 
  library path (either in your shell startup file or when running the sim on the command line), e.g.:

.. code-block:: C

    LD_LIBRARY_PATH=/opt/jsbsim/lib ./var/Quad_LisaM_2/nps/simsitl --fg_host 127.0.0.1

- If you get an error like ``"fatal error: gsl/gsl_rng.h: No such file or directory"``, you need to install the GNU Scientific Library and corresponding development packages (libgsl).
- If you get an error like ``"undefined reference to `pcre_compile'", edit file conf/Makefile.nps``, look for the line that begins with ``LDFLAGS`` and add ``-lpcre``, e.g.:

.. code-block:: C

    LDFLAGS += $($(TARGET).LDFLAGS) -lpcre

Simulating Multiple Aircraft
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When simulating multiple aircraft, the ``-udp_broadcast`` argument needs to be specified as part of the datalink invocation:

.. code-block:: C

    $PAPARAZZI_HOME/sw/ground_segment/tmtc/link -udp -udp_broadcast

In the case of Mac OS X, the IP ADDR must also be specified:

.. code-block:: C

    $PAPARAZZI_HOME/sw/ground_segment/tmtc/link -udp -udp_broadcast -udp_broadcast_addr <your_network_broadcast_ip_addr>

You can determine the ``IP ADDR`` for your network using ifconfig command:

.. code-block:: C

    $ ifconfig
    ...
    en0: flags=8863<UP,BROADCAST,SMART,RUNNING,SIMPLEX,MULTICAST> mtu 1500
        ether 60:03:08:8e:14:9e 
        inet6 fe80::6203:8ff:fe8e:149e%en0 prefixlen 64 scopeid 0x4 
        inet 192.168.1.59 netmask 0xffffff00 broadcast 192.168.1.255
        nd6 options=1<PERFORMNUD>
        media: autoselect
        status: active
    ...

Based on the above sample output, the invocation would look like the following:

.. code-block:: C

    $PAPARAZZI_HOME/sw/ground_segment/tmtc/link -udp -udp_broadcast -udp_broadcast_addr 192.168.1.255

.. _flightgear:

FlightGear
--------------------------

FlightGear Flight Simulator which can be used to visualize an aircraft and scenery. For the actual simulation, see Simulation.

See http://www.flightgear.org/

Installation
^^^^^^^^^^^^^^^

Debian/Ubuntu
~~~~~~~~~~~~~~~

The standard Debian/Ubuntu repositories contain mostly older FlightGear versions.

A lot has improved and changed over the years. To get the latest greatest Flightgear, as of writing this iv vv2020.3.6 and enjoy the 
improvements one can easily get that version by add in a PPA

For Ubuntu the latest edition of FlightGear is available from Launchpad PPA (contributed by Saikrishna Arcot), 
to add the PPA an install the latest Flightgear, this in your terminal:

.. code-block:: C

    sudo add-apt-repository ppa:saiarcot895/flightgear
    sudo apt-get update
    sudo apt-get install flightgear
    This will install v2020.3.1 or newer

Tip:

Not a Paparazzi issue but if you get a message box saying "zone.tab" missing copy an paste this in you terminal to fix it:

.. code-block:: C

    sudo apt-get install --reinstall flightgear-data-base

From source
~~~~~~~~~~~~~~~

A great page to read in case you want to install Flightgear from source can be found here

Adding Paparazzi 3D models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There are a few 3D UAV models that come with paparazzi:

+-----------------------+---------------------------------------------------+
| AIRFRAME              | DESCRIPTION                                       |
+=======================+===================================================+
| ``mikrokopter.xml``   | quadrotor frame                                   |
+-----------------------+---------------------------------------------------+
| ``hexa.xml``          | hexacopter                                        |
+-----------------------+---------------------------------------------------+
| ``easystar.xml``      | Multiplex EasyStar fixedwing                      |
+-----------------------+---------------------------------------------------+
| ``simple_bipe.xml``   | biplane/quadrotor hybrid (transitioning vehicle)  |
+-----------------------+---------------------------------------------------+


To make them available in flightgear, make a link from ``/usr/share/games/flightgear/Models/Aircraft/paparazzi`` to ``<paparazzi_dir>/conf/simulator/flightgear/``

.. code-block:: C

    sudo ln -s $PAPARAZZI_SRC/conf/simulator/flightgear/ /usr/share/games/flightgear/Models/Aircraft/paparazzi

Using FlightGear for Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For Flight Gear visualization of the simulation, version 2018.2.2 or higher is best.

NOTE: Only if you still wish to use version **v2.4** or lower for some reason, you must add the following to the firmware section of your airframe file:

.. code-block:: C

  <firmware name="fixedwing or rotorcraft">
     ...
     <define name="FG_2_4" value="1"/>
     ...
  </firmware>

Launch Flight Gear with the following command:

.. code-block:: C

    fgfs --fdm=null --native-gui=socket,in,30,,5501,udp

or to e.g. use the mikrokopter quadrotor model:

.. code-block:: C

    fgfs --fdm=null --native-gui=socket,in,30,,5501,udp --prop:/sim/model/path=Models/Aircraft/paparazzi/mikrokopter.xml

.. _jsbsim:

JSBSim
-----------

`JSBSim FDM <http://jsbsim.sourceforge.net/>`_ is an open source flight dynamics model (FDM) used in NPS.

Installation
^^^^^^^^^^^^^^

Debian Package
~~~~~~~~~~~~~~~~~~~~~

On Debian/Ubuntu you can install the ``paparazzi-jsbsim`` package.

.. code-block:: php

    sudo apt-get install paparazzi-jsbsim

If you don't have that in your sources, see Installation/Linux#Adding_the_APT_repository.

From Source
~~~~~~~~~~~~~~~

Compile JSBSIM from source (with specified date to make sure it works and API hasn't changed)

.. code-block:: php

    cvs -z3 -d:pserver:anonymous@jsbsim.cvs.sourceforge.net:/cvsroot/jsbsim co -D "23 Feb 2015" -P JSBSim 
    cd JSBSim
    ./autogen.sh
    ./configure --enable-libraries --enable-shared --prefix=/opt/jsbsim
    make
    sudo make install

When building a NPS simulator target, the build system will first try to find JSBSim via ``pkg-config`` and fall back to ``/opt/jsbsim``.

If you want to install to a different location, change the prefix to your liking. And you need to add a ``<makefile>`` 
section to your airframe file and add the correct flags to point to the include files and libraries, depending on where it is installed.

With the default installation to /usr/local/, this would look like

.. code-block:: php

    <makefile location="after">
        nps.CFLAGS += -I/usr/local/include/JSBSim
        nps.LDFLAGS += -L/usr/local/lib
    </makefile>

On OSX
~~~~~~~~~~~~~~~

Install the JSBSim libraries onto your system. This should already be installed with paparazzi-tools, but if it isn't:

.. code-block:: php

    sudo port install jsbsim

It uses code from the cvs repo, so it should be the most up-to-date source.

Troubleshooting
^^^^^^^^^^^^^^^^^^

If you get an error like "undefined reference to ``pcre_compile``, edit file ``conf/Makefile.jsbsim``, look for the line that begins with ``LDFLAGS`` and add ``-lpcre``, e.g.:

.. code-block:: php
        
    LDFLAGS += $($(TARGET).LDFLAGS) -lpcre

