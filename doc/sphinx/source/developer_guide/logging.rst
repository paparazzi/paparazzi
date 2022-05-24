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

FlightRecorder
^^^^^^^^^^^^^^^^^

Logging on ChibiOS systems can be achieved using the ``flight_recorder`` module, which depends on a
few other modules that also need to be included in your airframe.

.. code-block:: xml

  <firmware name="rotorcraft">
    <module name="flight_recorder"/>
  </firmware>

Data is stored on an SD card using the ``pprzlog`` format, discussed in more detail in the next section.
The logging starts automatically soon after the drone is connected to power. The logger status is reported
on the ``LOGGER_STATUS`` message, which can be viewed using the GCS by opening Tools -> Messages.

The ``flight_recorder`` will record all messages that are specified within a ``FlightRecorder`` process
that should be included in your airframe's telemetry file.

.. code-block:: xml

  <telemetry>
    <process name="FlightRecorder">
      <mode name="default">
        <message name="ATTITUDE"    period="0.05"/>
        <message name="IMU_ACCEL"   period="0.02"/>
        <message name="IMU_GYRO"    period="0.02"/>
        <message name="IMU_MAG"     period="0.02"/>
        <!-- etc. -->
      </mode>
    </process>
  </telemetry>

These telemetry files are located within the ``conf/telemetry/`` folder, and are associated to a specific
airframe in the ``conf.xml``. For example, ``conf/userconf/tudelft/conf.xml`` specifies for a default Bebop:

.. code-block:: xml

  <conf>
    <aircraft
     name="Bebop_default"
     ac_id="20"
     airframe="airframes/examples/bebop.xml"
     radio="radios/dummy.xml"
     telemetry="telemetry/default_rotorcraft.xml"
     flight_plan="flight_plans/rotorcraft_basic.xml"
     settings="settings/rotorcraft_basic.xml settings/control/rotorcraft_speed.xml"
     settings_modules="modules/ahrs_float_mlkf.xml modules/air_data.xml modules/bebop_ae_awb.xml modules/bebop_cam.xml modules/geo_mag.xml modules/gps.xml modules/guidance_rotorcraft.xml modules/imu_common.xml modules/ins_extended.xml modules/nav_basic_rotorcraft.xml modules/stabilization_int_quat.xml modules/video_rtp_stream.xml"
     gui_color="#ffffbc3bce5b"
    />
  </conf>

Logger SD ChibiOS
^^^^^^^^^^^^^^^^^^^^

The ``flight_recorder`` module is essentially a wrapper for another logger module called ``logger_sd_chibios``,
which handles the log file creation/closing. The ``flight_recorder`` uses these functions to open a log file and
outputs to it a telemetry message (in binary) that by defaults contains all the messages specified in the telemetry XML
as discussed in `FlightRecorder`_. However, if more control over what actually goes into the log file is necessary
or desired, it is possible to directly use ``logger_sd_chibios`` to create custom log functions. First, the module
must be included in your airframe.

.. code-block:: xml

  <firmware name="rotorcraft">
    <module name="logger" type="sd_chibios"/>
  </firmware>

Then, in the .c file where you want the data to be logged from, you can create custom log functions that will write
to the log either in binary format, or directly in ASCII format. To write directly in ASCII:

.. code-block:: C

  #include "modules/loggers/sdlog_chibios.h"
  static inline void custom_log_function_ascii(void) {
    // Check that log file has been created correctly
    if (pprzLogFile != -1) {
      // Write whatever you want to this file using sdLogWriteLog()
      sdLogWriteLog(pprzLogFile, "<Your log message %f %f>", foo, bar);
    }
  }

To write in binary instead, you need to create a function that behaves in a similar way as the FlightRecorder, which
sends telemetry data directly to the log file instead of over the air.

.. code-block:: C

  #include "modules/loggers/sdlog_chibios.h"
  #include "modules/loggers/pprzlog_tp.h"

  // Any log file could be specified from the airframe
  // Set the default to the one created by logger_sd_chibios
  #ifndef MY_LOG_FILE
  #def MY_LOG_FILE flightrecorder_sdlog
  #endif

  // Create a function that sends all your desired messaged to the log
  static void custom_telemetry_send(struct transport_tx *trans, struct link_device *device) {
    // There can be more than one pprz_send function
    pprz_send_msg_YOUR_MSG(trans, device, AC_ID, &foo, &bar);
  }

  static bool log_tagged;
  static inline void custom_log_function_binary(void) {
    if (MY_LOG_FILE.file != NULL && *(MY_LOG_FILE.file) != -1) {
      if (log_tagged == false && GpsFixValid()) {
      // Write at least once ALIVE and GPS messages
      // to log for correct extraction of binary data
      DOWNLINK_SEND_ALIVE(pprzlog_tp, MY_LOG_FILE, 16, MD5SUM);
      // Log GPS for time reference
      uint8_t foo_u8 = 0;
      int16_t foo_16 = 0;
      uint16_t foo_u16 = 0;
      struct UtmCoor_f utm = *stateGetPositionUtm_f();
      int32_t east = utm.east * 100;
      int32_t north = utm.north * 100;
      DOWNLINK_SEND_GPS(pprzlog_tp, MY_LOG_FILE, &gps.fix,
          &east, &north, &foo_16, &gps.hmsl, &foo_u16, &foo_16,
          &gps.week, &gps.tow, &utm.zone, &foo_u8);
      log_tagged = true;
      }
    // Send custom telemetry function directly to log
    custom_telemetry_send(&pprzlog_tp.trans_tx, &(MY_LOG_FILE).device);
    }
  }

And finally, include your custom log function in your module periodic function (or whatever other place
that should trigger the log writing).

.. code-block:: C

  void your_module_periodic(void) {
    // If logging in ASCII
    custom_log_function_ascii();
    // If logging in binary
    custom_log_function_binary();
  }

Decoding FlightRecorder logs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To download and convert the data, you need to first connect the SD card to your laptop, navigate to the
FLIGHT_RECORDER folder and transfer to your computer the relevant logs, named ``fr_XXXX.LOG``. To convert
this binary file into Pprzlog format files the program ``sw/logalizer/sd2log`` has to be used.
Make sure the environment variables are set before running Paparazzi executables from the commandline.
They can be set in your Terminal by using

.. code-block:: bash

  export PAPARAZZI_HOME=~/paparazzi
  export PAPARAZZI_SRC=~/paparazzi

.. tip::

  You can add the ``export`` lines above to your ``.bashrc`` file to automatically set the environment variables
  every time a new Terminal is opened.

Once the environment variables are set you can run ``sd2log`` in your terminal:

.. code-block:: bash

  ~/paparazzi/sw/logalizer/sd2log fr_XXXX.LOG  # Output stored in var/logs
  ~/paparazzi/sw/logalizer/sd2log fr_XXXX.LOG <output_dir_path>  # Output stored in <output_dir_path>

This will produce the Paparazzi .log, .data, and .tlm files that are stored in ``var/logs``. It creates a timestamp from the .tlm and changes the filename
to the take-off time if a GPS mesage with correct time was available in the file, or the current local PC time if no
GPS was available. The .log file will be recreated either from the current configuration or the MD5-labeled files that
are stored in ``var/conf`` each time you build an aircraft.

.. note::

  For the decoding process to work properly you must run ``sd2log`` from the same folder that contains the
  ``messages.xml`` file used during compilation of the aircraft. This is often not an issue, but you may run into
  problems if you work with multiple branches within your paparazzi repository.

Pprzlog format
-----------------

The Pprzlog format creates multiple files that can be used to analyse flight data and replay the flight. A log is
split into the following files:

- A ``.log`` file, an XML file, which contains a copy of the whole configuration (airframes, flight plans, ...)
- A ``.data`` file, an ascii file, which contains the list of the received messages. Each message is time-stamped in
  seconds since the creation of the file and marked with the ID of the sending aircraft
- A ``.tlm`` file, a copy of the original ``.LOG`` file renamed with the same name as the ``.log`` and ``.data``
  file to make it easier to associate the original log with the decoded files

.. tip::
  Because ``sd2log`` outputs a copy of the original ``.LOG`` file, the decoding process can be called directly
  on the file still on the SD card while it's mounted on your computer.

The name of the files associated to a specific log is the same, and is generated from the date and time of creation.
The lines of the ``data`` file are formatted according to the message description listed in the ``conf/messages.xml``
file. For example:

.. code-block:: text

  30.5941 186 ATTITUDE 0.036228 0.018550 0.021443

contains an ``ATTITUDE`` message received at time 30.5941s, from aircraft 6. According to the ATTITUDE message
description:

.. code-block:: xml

  <message name="ATTITUDE" id="6">
    <field name="phi"   type="float" unit="rad" alt_unit="deg"/>
    <field name="psi"   type="float" unit="rad" alt_unit="deg"/>
    <field name="theta" type="float" unit="rad" alt_unit="deg"/>
  </message>

In this case, at the time the message was logged, the attitude of the drone corresponded to :math:`{\phi} = 0.036228`,
:math:`{\theta} = 0.018550`, and :math:`{\psi} = 0.021443` radians. Note that the appropriate ``messages.xml``
description, i.e. the one which has been used while the log was created, is itself stored in the associated ``.log``
file. It may differ from the current one in your ``conf/`` folder.

.. note::

  The ``.data`` files may be huge. They can be efficiently compressed, with the ``bzip2`` compression format seemingly
  performing better than others on these files.

Data plotting
----------------

There are different methods to visualize and process the data stored in log files.

Log Plotter
^^^^^^^^^^^^^^^
If no post-processing of the data is required, log data can be visualized using the Log Plotter, located in
``sw/logalizer/logplotter``. It can be launched either from the command line, or through the Paparazzi Center by
navigating to Tools -> Log Plotter. This tool can plot data from the same or different logs in the same window, as
well as offering the option to export the track as a KML file for Google Earth, or to a CSV file for further data
processing.

Log File Player
^^^^^^^^^^^^^^^^^^^
A flight can be replayed with the Log File Player (``sw/logalizer/play``), which can be started either from the
command line, or from the Paparazzi Center by navigating to Tools -> Log File Player, or even from the Session selection
box to start a complete replay session with the GCS, server, and player tool. In this last case, this agent then
acts as a substitute for the Data Link agent and will send onver the bus the messages that had been sent by the aircraft
while the log was recorded.

.. note::

  While replaying a log through the GCS, it is a good idea to disable a new log creation from the Server. This can
  be achieved by launching the Server process with the ``-n`` option.

While doing a log replay it can be very valuable to launch a Messages process (Tools -> Messages). This allows for the
use of the Tools -> Real Time Plotter and also displays all the data received from the aircraft.

If the Log File Player is launched with the ``-o`` option, the player will send to a serial port all the  binary
messages as they had been received through the modem during the flight. Additional options include ``-s`` to set
the baudrate (default 9600), and ``-d`` to set the device (default ``/dev/ttyUSB0``).

Paparazzi Log Parsing
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The Github repository `tudelft/paparazzi_log_parsing <https://github.com/tudelft/paparazzi_log_parsing>`_ provides
handy Matlab and Python tools to convert the log data into a dictionary-like structure for easy post-processing.
The Matlab and Python scripts are very similar and function in almost the same way except for certain language-specific
functionality, and contain an example file that illustrates how to use the functionalities provided by the repositories.


