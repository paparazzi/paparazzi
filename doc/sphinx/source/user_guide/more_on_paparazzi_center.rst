.. user_guide more_on_paparazzi_center

============================
More on the Paparazzi Center
============================

The Paparazzi Center is configured by two files : the aircrafts configuration file and the control panel file.

These files are located at ``conf/conf.xml`` and ``conf/control_panel.xml``. To make configuration easier, these files are often symbolic links to actual files, usually located in ``conf/userconf/``.

Conf
----

``conf.xml`` contains the configuration of each aircrafts:

- Name
- Id
- airframe
- flight plan
- radio
- telemetry
- settings
- GUI color

To hold you personnal confs, create a new file with this simple content in *conf/userconf/*, and use the *start.py* configuration utility to use it in Paparazzi :

.. code-block:: xml

    <conf>
    </conf>

Tools
-----

The *Tools* in the Paparazzi Center come from the ``conf/tools`` directory. Each xml file produce a new entry in the *Tools* menu. You can add you own tools by creating a new xml file in this directory, and restarting the Paparazzi Center.

There are many tools that you will probably never use. You can remove these from the menu by adding their name to the ``blacklisted`` file.


Sessions & Control panel
------------------------

``control_panel.xml`` contains the sessions. A session is a set of programs with arguments.

To define a new session, go to *Session->New* and give it a name. Start all the programs you want, with their correct arguments, then *Session->Save*.

.. warning::

    This file used to hold the programs accessible in the *Tools* menu. This is now only supported for backward compatibility


