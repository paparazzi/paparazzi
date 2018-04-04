.. quickstart main_quickstart first_simu

======================
Flight Simulation
======================

This will show the capabilities of Paparazzi. Doing this is not required but will make later steps easier.

Refer to the :doc:`../user_guide/software/simulation` article first.


For your first few simulations you should use the default flight plan (use the Microjet aircraft) and configuration files in Paparazzi. This will allow you to get the simulator up and running in as little time as possible so you can see what Paparazzi can do. Once you have familiarized yourself with what a default Paparazzi simulation can do you will probably want to start building a flight plan (where the simulated aircraft flies) for your local area. The best way to do this is to go to Google Maps and type in your local town. Once you have found your local town, get the Latitude and Longitude of somewhere where you would like to fly your simulated UAV. Once you have the latitude and longitude, go and edit the basic flight plan (this is the example flight plan) and change the HOME latitude and longitude to the one near your home town. When you change the HOME latitude and longitude, the whole flight plan and way points will dynamically move to their relative position around your chosen latitude and longitude. You can now download the Google maps for your chosen area to give it that realistic look.

Once you have done this you will know more about what you can configure with Paparazzi. The best place to start to learn how to configure Paparazzi is to edit your flight plan so you can do more complex things. There are a multitude of examples for you to use both on the wiki and within the supplied Paparazzi flight plan files.

To try out the simulation, choose an aircraft, e.g. Microjet

    #. Then choose "Target -> Sim" it will mean that you are building firmware for usage with "Simulation" session. Press "Build".

    #. After the process is done, choose "Simulation" session and click "Execute".

    #. You will get two windows: GCS and windows named after our aircraft (e.g. Microjet).

    #. Now we recommended choosing "Maps" -> "Maps Auto" (of course if you have an Internet connection) to see satellite maps in GCS.

    #. Then in the lower left pane there is a small runway where you choose "Takeoff".

If everything goes well you'll see that your virtual aircraft took off and is making circles around the "standby" point. All further interactions are made through changing modes inside the flight plan.
