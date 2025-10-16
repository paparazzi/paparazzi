.. user_guide more_on_paparazzi_center

============================
More on the Paparazzi Center
============================

The Paparazzi Center is configured by two files : the aircrafts configuration file (a.k.a. "the set") and the control panel file.

Select the ones you use in the paparazzi center: select the *set* in the top right drop-down,
and the control panel in the top left drop-down of the *Operation* tab.


Set
---

The set, also called *conf* contains the configuration of each aircrafts (a.k.a. **AC**):

- **Name**
- **Id**: a uniq number within the set, in the range [1-255].
- **GUI color**
- **airframe**: Describes nearly all the configuration.
- **flight plan**
- **radio**: configuration of the RC channels
- **telemetry**: Describes which messages will be send to the ground and/or recorded on board.
- **settings**: values that can be changed during the flight.


Make your own set by creating a new file with this simple content in *conf/userconf/*, and selecting it in the paparazzi center:

.. code-block:: xml

    <conf>
    </conf>

Tools
-----

The *Tools* in the Paparazzi Center come from the ``conf/tools`` directory. Each xml file produce a new entry in the *Add tool* menu. You can add you own tools by creating a new xml file in this directory, and restarting the Paparazzi Center.

There are many tools that you will probably never use. You can remove these from the menu by adding their name to the ``blacklisted`` file.


Sessions & Control panel
------------------------

The *control panel* contains the sessions. A session is a set of programs with their arguments.

Once all the needed tools are started, either by launching a session or by starting them individually,
you can save them as a session (hamburger button)

To define a new session, go to the *Operation* tab, start all the tools you want

.. warning::

    This file used to hold the programs accessible in the *Tools* menu. This is now only supported for backward compatibility


