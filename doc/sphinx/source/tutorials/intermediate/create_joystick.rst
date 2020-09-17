.. tutorials intermediate create_joystick

====================================
Create a new joystick configuration
====================================

New joystick configurations can be added in the ``paparazzi/conf/joystick`` directory.

Profile a joystick
==================

Test if your joystick is recognized: plug your joystick then run ``dmesg``. The message is different for every device, but the last lines should look like these::

    [49174.642275] usb 1-1: new low-speed USB device number 8 using xhci_hcd
    [49174.812307] usb 1-1: New USB device found, idVendor=046d, idProduct=c214, bcdDevice= 2.05
    [49174.812309] usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=0
    [49174.812310] usb 1-1: Product: Logitech Attack 3
    [49174.812311] usb 1-1: Manufacturer: Logitech
    [49174.823264] input: Logitech Logitech Attack 3 as /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/0003:046D:C214.000B/input/input37
    [49174.823608] hid-generic 0003:046D:C214.000B: input,hidraw4: USB HID v1.10 Joystick [Logitech Logitech Attack 3] on usb-0000:00:14.0-1/input0


Launch ``./sw/ground_segment/joystick/test_stick``. It will display joystick informations, then print current status::

    Available button: 5 (0x5)
    Available hats: 0 (0x0)
    Available axes: 4 (0x4)
    Axis 0 : parameters = [-32768,32768]
    Axis 1 : parameters = [-32768,32768]
    Axis 2 : parameters = [-32768,32768]
    Axis 3 : parameters = [-32768,32768]
    Input device name: "Amazing Joystick" on SDL device "0"
    buttons 0 0 0 0 0 | hat 0 | axes 0 0 0 0
    buttons 0 0 0 0 0 | hat 0 | axes 0 0 0 0
    buttons 0 0 0 0 0 | hat 0 | axes 0 0 0 0
    buttons 0 0 0 0 0 | hat 0 | axes 1 -40 0 0
    buttons 0 0 0 0 0 | hat 0 | axes -33 -83 0 0
    buttons 0 0 0 0 0 | hat 0 | axes -101 0 0 0

.. note:: Your joystick may need to be calibrated. Go to the :ref:`Calibration` section below.

Create a new file for your joystick in the ``paparazzi/conf/joystick`` directory with the syntax of the following example:

.. code-block:: xml

    <joystick>
      <input>
        <axis index="0" name="roll"  deadband="10" limit="1.00" exponent="0.7" trim="0"/>
        <axis index="1" name="pitch"/>
        <axis index="2" name="yaw"/>
        <axis index="3" name="thrust"/>
        <button index="0" name="fire"/>
        <button index="1" name="top_center"/>
        <button index="2" name="top_left"/>
        <button index="3" name="top_right"/>
        <button index="4" name="far_left"/>
      </input>

      <variables>
        <var name="mode" default="0"/>
        <set var="mode" value="0" on_event="top_left"/>
        <set var="mode" value="1" on_event="top_center"/>
        <set var="mode" value="2" on_event="top_right"/>
      </variables>

      <messages period="0.01">

        <message class="datalink" name="RC_4CH" send_always="true">
          <field name="mode"        value="mode"/>
          <field name="throttle"    value="Fit(-thrust,-127,127,0,127)"/>
          <field name="roll"        value="roll"/>
          <field name="yaw"         value="yaw"/>
          <field name="pitch"       value="pitch"/>
        </message>


        <!-- resurrect throttle on fire button -->
        <message class="ground" name="DL_SETTING" on_event="fire">
          <field name="index" value="IndexOfSetting('autopilot.kill_throttle')"/>
          <field name="value" value="0"/>
        </message>
        
        <!-- kill throttle on far_left button -->
        <message class="ground" name="DL_SETTING" on_event="far_left">
          <field name="index" value="IndexOfSetting('autopilot.kill_throttle')"/>
          <field name="value" value="1"/>
        </message>

      </messages>

    </joystick>


Inputs
------

Edit the *input* section according to your info given by *test_stick*. There are 3 kind of inputs :

- *axis*: "analog" stick that range from a min to a max value,
- *hat*: tiny stick or arrows that can have 8 directions (up, down, left, right, up-left, ...),
- *buttons*.

*name* and *index* attributes are mandatory for all.

Axis has 4 optionnal attributes:

- *deadband*: input values within the deadband output 0. Range in [0, 127].
- *exponent*: gives precise control around center values, and greater speed at high values. Range in [0, 1.0]. 0 has no effect, 1.0 has maximum effect.
- *limit*: limit the range of the output values, in percent. Range in [0, 1.0]. 1.0 has no effect.
- *trim*: set offset in output values. Range in [-127, 127].


These attributes are applied in that order :  deadband, exponent, limit, trim.

Variables
---------

In the *variables* section, you can define integer variables with the *var* tag, with the *name* and *default* attributes. The *set* tag allows to set a value to a variable on an event. An event is the name of a button or a hat.

Messages
--------

The *period* attribute on the *messages* section is the period in seconds at which inputs will be checked.

In this section, you define which messages will be sent, the value of each field, and the conditions required to send the message.

The *message* tag has two required attributes: the *name* and *class* of the message, and two optionnal attributes : *send_always* and *on_event*.

*send_always* is a boolean that default to *false*. If set to *true*, messages will keep be sent at the *period* rate. If set to *false*, message will be sent only when one of its field change value.

*on_event* defines the event/condition required to send the message. Complexes conditions are evaluated. Here are some examples:

- ``on_event="button11 || button10"``
- ``on_event="(button11 || button10) && pitch > 100"``

In the message node, all fields must be specified except the *ac_id* field, that is filled by *input2ivy*.

*value* is a "C like" expression made of axis and variables names, operators, and a set of utily functions.

Thoses functions are:

- ``Scale(toto, min, max)`` : scale toto from default min/max values [-128, 127] to [*min*, *max*] 
- ``Fit(x, min_in, max_in, min_out, max_out)`` : scale *x* from *min_in* *max_in* to *min_out*, *max_out*
- ``Bound(x, min, max)`` : bound x between *min* and *max*
- ``PprzMode(x)`` : scale input value to [0;1;2]. usefull for RC mode.
- ``JoystickID()`` : return the joystick ID.
- ``IndexOfEnum(NAME)`` : return the index of the enum member *NAME*
- ``IndexOfSetting('setting_name')`` : return the index of the setting *setting_name*.
- ``IndexOfBlock('block_name')`` : return the index of the block *block_name*.
- ``HatCentered(hat_name)``, ``HatUp(hat_name)``, ``HatRight(hat_name)``, ``HatRightUp(hat_name)``, ``HatDown(hat_name)``, ``HatRightDown(hat_name)``, ``HatLeft(hat_name)``, ``HatLeftUp(hat_name)``, ``HatLeftDown(hat_name)`` : return 1 or 0.


The operators are: *-*, *+*, *\**, *%*, *&&*, *||*, *<*, *>*

Some examples:

- ``value="roll"``
- ``value="(right-left)*127"``
- ``value="IndexOfSetting('autopilot.kill_throttle')"``
- ``value="Fit(-thrust,-127,127,0,127)"``
- ``value="IndexOfBlock('land here')"``


.. _Calibration:

Calibration
===========

Your joystick may need calibration. Uncalibrated joystick may send non-zero values when the sticks are in neutral position.

Install the joystick and the jstest-gtk packages via: 

    ``sudo apt-get install joystick jstest-gtk``

Use the graphical jstest-gtk tool (or the commandline jstest) to view/edit your joystick calibration and axis/button mappings. Start it via: 

    ``jstest-gtk``

**Store the calibration**

Your calibration and mapping will be lost once you unplug the joystick, so store your configuration via:

    ``sudo jscal-store /dev/input/js0``

If you replug your joystick the next time, udev should take care of automatically loading the appropriate configuration. 
