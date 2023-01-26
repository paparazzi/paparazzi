.. developer_guide debug

=========
Debugging
=========

Debugging can be essential when developing your own module. Here are some ways to help debug your code.

Messages
--------

There are a few messages that you can use to debug over telemetry.
You then can see the result with the **Messages** tool.

INFO_MSG
________

**INFO_MSG** allows you to send a char array. It is defined as:

.. code-block:: xml

    <message name="INFO_MSG" id="215">
        <field name="msg" type="char[]"/>
    </message>


Example:

.. code-block:: C

    #include <string.h>
    #include "modules/datalink/telemetry.h"
    ...
    void my_function() {
      char msg[] = "It works!";
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(msg), msg);
    }

PAYLOAD_FLOAT
_____________

**PAYLOAD_FLOAT** allows you to send an array of floats. It is defined as:

.. code-block:: xml

    <message name="PAYLOAD_FLOAT" id="237">
        <field name="values" type="float[]"/>
    </message>


Example:

.. code-block:: C

    #include "modules/datalink/telemetry.h"
    ...
    void my_function() {
      char buf[] = {42.2, 12.5, 8};
      DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3, buf);
    }


Shell
-----

When using **ChibiOS**, you can use the **shell** module to have a shell-like interface on the autopilot.

Setup shell
___________

In your airframe, add the *shell* module:

.. code-block:: xml

    <module name="shell">
        <configure name="SHELL_PORT" value="uart6"/>
        <configure name="SHELL_BAUD" value="B115200"/>
        <configure name="SHELL_DYNAMIC_ENTRIES_NUMBER" value="5"/>
    </module>

Connect your PC to the UART through a *serial to USB convertor*, and open the serial port with e.g. minicom or GtkTerm. Hit *ENTER* to be welcomed by a nice ``pprz >`` prompt !

To test it, type ``info`` in the prompt, then hit *ENTER*. Various info about the autopilot should be printed.


Add your own commands
_____________________

You can add your own commands to the shell.

.. note::
    
    You can define up to ``SHELL_DYNAMIC_ENTRIES_NUMBER`` commands. This number default to 5, but you can increase it with a *configure* in your airframe.

This example shows how to define your own commands in your module:

.. code-block:: C
    
    #if USE_SHELL
    #include "modules/core/shell.h"     // include this file
    
    // implement your command in a function with this prototype:
    static void cmd_mycommand(shell_stream_t *sh, int argc, const char *const argv[]) {
        // print to the shell with chprintf
        chprintf(sh, "My amazing command got %d args!\r\n", argc);
    }
    #endif
    
    ...
    
    void mymodule_init(void) {
    #if USE_SHELL
        // register your command
        shell_add_entry("mycmd", cmd_mycommand)
    #endif
    }


.. note::

    + Unlike what is usually done, the ``argc`` is the number of arguments *excluding* the command invocation. When no parameters is given to the command, *argc == 0*.
    + As done in this example, you can put all your shell related code behind a ``#if USE_SHELL ... #endif`` for your module to work regardless if the shell module is loaded or not.

.. admonition:: Using the serial over USB

    If you are using the serial over USB, you can use ``usb_serial`` or ``usb_serial_debug`` as the SHELL_PORT.

Debugger
--------

You can debug your code using GDB.

Build the firmware with debug symbols enabled. In the firmware section of the airframe, add:

.. code-block:: C

    <configure name="RTOS_DEBUG" value="1"/>

The ELF file is located in ``var/aircrafts/<$AIRCRAFT>/ap/obj/ap.elf``.

You can then debug the firmware using any SWD or JTAG probe.

coming soon™: how to debug using VsCode !

Log
---

You can use logs to help you debug your code. See the :ref:`logging` page.


