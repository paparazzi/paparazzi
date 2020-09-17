.. developer_guide communication

=============
Communication
=============

We saw in the User Guide :doc:`../user_guide/communication` section that you can change which messages are sent, and at which rate. We will see in this section how you can define and send your own messages.

Define a new message
--------------------

By default, PprzLink default message definition is used. You can find it in ``sw/ext/pprzlink/message_definitions/v1.0/messages.xml``. To add your own messages, you first need to copy this file in your ``conf`` directory.

Add your message in that file on the model of the other messages. Make sure to add it in the appropriate message class (telemetry, datalink, ground, ...), and make sure to use a free ``id``. This *id* being encoded on a uin8_t, it must be comprise between 1 and 255 (0 is reserved). As you can see, there are not much left in the telemetry class...

Re-build paparazzi with ``make`` at the root directory. your message should now be present in the ``var/messages.xml`` file.


Send a telemetry message
------------------------

If you defined a new telemetry message, you now want the drone to send it. You can either send it manually from a module or use the Paparazzi periodic telemetry.

.. warning::

    This section is not written yet, go to `http://wiki.paparazziuav.org/wiki/Telemetry <http://wiki.paparazziuav.org/wiki/Telemetry>`_.
