.. user_guide simulation gazebo

.. _gazebo:

===========================
Gazebo
===========================

The nice sim framework Gazebo from here http://gazebosim.org/ can now be use from within Paparazzi. 
Are you doing work on UAS in combination with e.g. Vision based navigation? Then check it out. I might make testing your new work so much simpler.

Be warned, using it can be highly addictive, and might tempt you into buying a new computer with high specifications. 
To be able to use Gazebo a very good Video card is needed, consider upgrading you hardware if everything runs slowly.

Note that from stable version 5.14 onwards, only version 8 and 9 of Gazebo work in Paparazzi on Ubuntu 16.04 or higher.

Installation
-------------------

Make sure Gazebo is installed, version 9 if you are on Ubuntu 18 and higher

.. code-block:: php

  sudo apt install gazebo9 libgazebo9-dev

If for some reason this doesn't work, you can find some additional instructions here: http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0

Setup and Configure
-----------------------

This section shows what and how to configure to run a simulation in Gazebo through paparazzi.

Vehicle Model
^^^^^^^^^^^^^^^

Prepare your Gazebo aircraft model (example see ``conf/simulator/gazebo/models/ardrone/``):

Place the aircraft model in the ``conf/simulator/gazebo/models/`` folder, this folder is added to the search path of Gazebo when NPS is launched. 
Gazebo uses a **Front, Left, Up** coordinate system for aircraft, so make sure the **+x** axis points forwards. 
The model should include a link for each motor with the same names as those listed in ``NPS_ACTUATOR_NAMES`` (see below), e.g. 'nw_motor'. 

Camera links should have the name specified in ``.dev_name`` in the

.. code-block:: php

  corresponding video_config_t struct, see sw/airborne/boards/pc_sim.h and sw/airborne/modules/computer_vision/video_thread_nps.c

Additional models can be found in the ``sw/ext/tudelft_gazebo_models``. You will have to run ``git submodule init``, ``git submodule update`` to pull in the models.

World
^^^^^^^^^^^^^^^^

Prepare the world (see conf/simulator/gazebo/worlds/ardrone.world or any other world file you might find there).

.. note::

  The real-time update rate should be set to zero, as the simulation back-end is already handled by Paparazzi:

  .. code-block:: php

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate><!-- Handled by Paparazzi! -->
    </physics>


Spherical coordinates should be provided for navigation. At this moment, there is an issue where Gazebo incorrectly uses a **WSU** coordinate system instead of **ENU**. 
This can be fixed by setting the heading to 180 degrees as shown below:

.. code-block:: php

  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <latitude_deg>51.9906</latitude_deg>
    <longitude_deg>4.37679</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>180</heading_deg><!-- Temporary fix for issue https://bitbucket.org/osrf/gazebo/issues/2022/default-sphericalcoordinates-frame-should -->
  </spherical_coordinates>

Additional world models can be found in the ``sw/ext/tudelft_gazebo_models``. You will have to run git submodule init, git submodule update to pull in the models.

Airframe
^^^^^^^^^^^

Enhance your Paparzazzi airframe file to be able to use Gazebo (see ``examples/ardrone2_gazebo.xml``):

Select Gazebo as the FDM (Flight Dynamics Model) by adding it to the aircraft file

.. code-block:: php

  <target name="nps" board="pc">
    <module name="fdm" type="gazebo"/>
  </target>

Add actuator thrusts and torques to the ``SIMULATOR`` section:

.. code-block:: php

  <section name="SIMULATOR" prefix="NPS_">
    <define name="ACTUATOR_NAMES" value="nw_motor, ne_motor, se_motor, sw_motor" type="string[]"/>
    <define name="ACTUATOR_THRUSTS" value="1.55, 1.55, 1.55, 1.55" type="double[]"/>
    <define name="ACTUATOR_TORQUES" value="0.155, -0.155, 0.155, -0.155" type="double[]"/>
    ...
  <section>

The thrusts and torques are expressed in SI units (N, Nm) and should be in the same order as the ``ACTUATOR_NAMES``.

In the same section, bypass the AHRS and INS as these are not supported yet, so add this

.. code-block:: php

  <section name="SIMULATOR" prefix="NPS_">
    ...
    <define name="BYPASS_AHRS" value="1"/>
    <define name="BYPASS_INS" value="1"/>
    ...
  <section>

If you want to use visual based behavior, enable video thread simulation:

.. code-block:: php

  <section name="SIMULATOR" prefix="NPS_">
    ...
    <define name="SIMULATE_VIDEO" value="1"/>
    ...
  <section>

Specify the Gazebo world and aircraft name:

.. code-block:: php

  <section name="SIMULATOR" prefix="NPS_">
    ...
    <define name="GAZEBO_WORLD" value="my_world.world"/>
    <define name="GAZEBO_AC_NAME" value="my_uav"/>
  <section>

.. note:: Make sure all included modules work with NS.

At the current state of Paparazzi code (20180206), most of the modules that depend on video_thread are 
only built when the target ap (autopilot hardware) is selected as the target.

As a quick 'n dirty fix, try to remove the target attribute from the makefile element in the module xml, e.g.:

.. code-block:: php

  <makefile target="ap"> ---> <makefile>

It would be great if as a user you would improve this and make a Pull request of your code improvements 
to the main Paparazzi codebase, TIA
