.. developer_guide logging

=============
Logging
=============

The ability to log and record drone data is a crucial element in a drone's mission as well
as for debugging purposes. There are different methods for data logging, both over telemetry and
onboard on an external device, all of which are covered in this section.


Logging on Linux based OS
------------------------------
The approaches discussed in this section are only possible for systems that run a Linux based OS
with access to internal storage space on your autopilot, as is the case with most Parrot drones.

Printing to terminal
^^^^^^^^^^^^^^^^^^^^^^^^^

Perhaps one of the easiest approaches to logging is to make your module print to a terminal.
You can add your own debug messages by including the ``<stdio.h>`` header to get access to the
``fprintf`` function. Then, at any point in your code, you can use this function to print your own
message.

.. code-block:: C

  <stdio.h>
  fprintf(stderr, "<Your debug message>");

It can be useful (if you expect to use this method a lot) to create a macro for this command that
automatically includes additional information on where the message originates, like is done in the
``orange_avoider`` module.

.. code-block:: C

  #define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

This allows you to only have to write ``PRINT("<your debug message>")`` in your code and it will automatically
prepend ``"[orange_avoider->%s()] "`` to your message, which can help debugging if multiple modules
are printing information simultaneously.

When running in simulation ``fprintf`` automatically prints the message in the Paparazzi Center. You can also
read these messages on the real drone using the following steps:

1. Open a terminal window (Ctrl+Alt+T).
2. Make sure you are connected to the drone's WiFi access point, then open a telnet connection
   using ``telnet 192.168.42.1``.
3. Navigate to the Paparazzi folder using ``cd data/ftp/internal_000/paparazzi``.
4. You need to restart the autopilot for the messages to show up. Run ``killall -9 ap.elf``, then
   ``./ap.elf`` to restart the autopilot.
5. When you are finished and the drone is landed, kill the autopilot by pressing Ctrl+C twice, then
   disconnect using Ctrl+D.

.. note::

  The target address of the ``telnet`` command may be different than what is reported here. You can
  double check by going into the Makefile of your drone and looking for the ``HOST`` variable. For
  example, the Makefile for the Bebop drone can be found in ``conf/boards/bebop.makefile``

.. warning::

  It is very easy to accidentally start multiple autopilot processes when connected to the drone by
  WiFi. This can cause flickering in the GCS as it receives duplicate messages, as well as slowing down
  the execution time. You can terminate all autopilot processes simultaneously by running ``killall -9 ap.elf``

File Logger
^^^^^^^^^^^^^^

Logging data onboard on a Linux based OS is pretty straightforward. The Bebop drone, for example, is
equipped with onboard storage, which can be used to store the logs. First, it is necessary to add the
``logger_file`` module in your airframe and specifying the path to the internal storage.

.. code-block:: xml

  <firmware name="rotorcraft"/>
    <module name="logger_file">
      <define name="FILE_LOGGER_PATH" value="/data/ftp/internal_000"/>
    </module>
  </firmware>

Then, in the C file ``sw/airborne/modules/loggers/file_logger.c`` add the header of the module that
contains the variables that you want to log. Within ``file_logger.c`` you can specify which variables
you want to have logged in the same fashion as normal ``fprintf`` as described in the previous sesction.
The variable names and types have to be specified.

The file logger needs to be manually started and stopped by going to the Paparazzi GCS. In the module
settings tab navigate to Settings -> Modules. Here you can start and stop a log. To retrieve the log file
connect to the drone by WiFi and using either a browser or an FTP client (like the Linux standard Files application)
connect to ``ftp://196.168.42.1``. This will take you to the FTP folder of the Bebop. Browse to ``/Internal000/``.
Here we can find the .csv files, named ``00000.csv`` to ``0000X.csv`` depending on the flights you logged.
Copy the files to your computer for analysis.


Logging on ChibiOS based OS
-----------------------------