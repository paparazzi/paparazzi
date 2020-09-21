.. user_guide radio

======
Radio
======

Radio files must be in the ``conf/radio/`` directory, and have this syntax:

.. code-block:: xml

    <radio name="My own FrSky X9D" data_min="900" data_max="2100" sync_min ="5000" sync_max ="15000" pulse_type="POSITIVE">
        <channel function="ROLL"     min="987" neutral="1503" max="2011" average="0"/>
        <channel function="PITCH"    min="987" neutral="1451" max="2011" average="0" reverse="1"/>
        <channel function="YAW"      min="990" neutral="1500" max="2011" average="0"/>
        <channel function="THROTTLE" min="998" neutral="998" max="2011" average="0"/>
        <channel function="MODE"    min="987" neutral="1500" max="2011" average="1"/>
    </radio>

.. note::

    The attributes ``data_min``, ``data_max``, ``sync_min``, ``sync_max`` are used only for receivers with **PPM** output. They must however be present otherwise the configuration will not build. Go to `http://wiki.paparazziuav.org/wiki/Radio_Control <http://wiki.paparazziuav.org/wiki/Radio_Control>`_ for more details.

Set your telemetry to receive PPM messages (*ppm* mode for rotorcrafts, and *Debug* FBW mode for fixedwings) and watch this message. Set the channels in this file in the same order than those received in the *PPM* message, and move the RC sticks to set correctly the *min*, *max*, and *neutral* values. Make sure that *min* < *max*.

If the channel needs to be reversed, add the attribute ``reverse="1"``.

Channels can be averaged by using the attribute ``average="1"``.




