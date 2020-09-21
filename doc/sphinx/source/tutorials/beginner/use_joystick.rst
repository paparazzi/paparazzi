.. tutorials beginner gcs_setup

===============
Use a Joystick
===============

You can use a joystick to command your drone via the datalink if you don't have an RC transmitter.

Currently supported joysticks are listed in the ``paparazzi/conf/joystick`` directory.

Learn how to configure a new joystick on this page: :doc:`../intermediate/create_joystick`

+ Open the airframe file of you aircraft and change the *radio_control* type to *datalink* : ``<module name="radio_control" type="datalink"/>``. Build and upload to the drone.

.. note:: For the first time you try it, remove the propeller blades from you drone : if your configuration is wrong, motors could start spinning and hurt you!

+ Start a session as usual for your drone
+ Start the "Joystick" tool : Tools->Joystick, and stop the program (it might already be crashed because of a bad options)
+ Edit the Joystick command : ``$PAPARAZZI_SRC/sw/ground_segment/joystick/input2ivy  -ac AC_NAME JOYSTICK_CONFIG_FILE.xml``. 
+ Replace **AC_NAME** by the name of the aircraft (it is probably good as it takes the current A/C), and replace **JOYSTICK_CONFIG_FILE** by a filename from ``paparazzi/conf/joystick``.

.. attention:: Initial sticks positions default to middle position until the axis has been moved. Move all axis to avoid bad surprises.

    You can also use the ``-c`` option (c meaning *check*) to prevent sending messages with bad values. Messages will start beeing sent only when all axis received events.
    
    ``$PAPARAZZI_SRC/sw/ground_segment/joystick/input2ivy -ac AC_NAME JOYSTICK_CONFIG_FILE.xml -c``

.. note:: Save the programs as a new session to avoid doing that all over again every time!
