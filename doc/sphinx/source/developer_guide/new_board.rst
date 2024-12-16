.. developer_guide new_board

===========================
How to support a new board
===========================

To integrate a new card, it is necessary to create several files.

.. _board_dir:

Board directory
----------------

You'll find the complete set of boards in `sw/airborne/boards/ <https://github.com/paparazzi/paparazzi/tree/master/sw/airborne/boards/>`_. You can create a directory (and sub-directories) for your board. 

.. note::
    You need to define a structure like ``sw/airborne/boards/<manufacturer>/<board>/<vx.x>/``.

Within this directory, you must have :

1. A ``Makefile`` to generate the board.h file from board.cfg using `boardGen.pl <https://github.com/alex31/chibios_enac_various_common/blob/master/TOOLS/boardGen.pl>`_.
2. A ``board.cfg`` containing the definition of the pins used (see `documentation <https://github.com/alex31/chibios_enac_various_common/blob/master/TOOLS/DOC/boardGen.pdf>`_). It is preferable to use the generic pin names defined in the ``common_board.h`` in the directory `sw/airborne/arch/chibios <https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/arch/chibios/common_board.h>`_. Any information that is not common to the various boards can be inserted in the HEADER section of the ``board.cfg``.
3. A ``board.h`` file, which is the result of running ``make`` in this directory (this file must not be modified; you have to run `boardGen.pl` for any change in `board.cfg`).
4. A ``mcuconf_board.h`` file, which defines the microcontroller's configuration (clock, peripherals, etc.). If the microcontroller is supported by ChibiOS, an example can be found in `sw/ext/chibios/demos/STM32/<board_name>/cfg/mcuconf.h <https://github.com/paparazzi/ChibiOS/tree/paparazzi/demos/STM32>`_.


.. _linker_script:

Linker Script
----------------

To compile, you need the linker script developed for your microcontroller.
Check the directory `sw/airborne/arch/chibios <https://github.com/paparazzi/paparazzi/tree/master/sw/airborne/arch/chibios>`_ to see if the linker script corresponding to your microcontroller is already present. If not, retrieve the linker script from `ChibiOS <https://github.com/paparazzi/ChibiOS/tree/paparazzi/os/common/startup/ARMCMx/compilers/GCC/ld>`_.



Board makefile
----------------
In the directory `conf/boards/ <https://github.com/paparazzi/paparazzi/tree/master/conf/boards>`_, you'll find the Makefile used for compiling.
It is necessary to create a new Makefile for each board, named ``<board_name>_<version>.makefile`` (e.g. ``tawaki_2.0.makefile``).
The name of this file will be the name used in the airframe file when defining the target:

.. code-block:: xml
    
     <target name="ap" board="<board_name>_<version>">
     </target>

In this Makefile, it is necessary to configure several points :

* ``BOARD_DIR`` must point to the directory defined in :ref:`board_dir` with relative path from `sw/airborne/boards <https://github.com/paparazzi/paparazzi/tree/master/sw/airborne/boards>`_ and ``BOARD_CFG`` must point to the file `sw/airborne/arch/chibios/common_board.h <https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/arch/chibios/common_board.h>`_.
* check floating-point unit type to configure ``USE_FPU_OPT`` (single-precision and a double-precision).
* modify the linker script in ``CHIBIOS_BOARD_LINKER`` to use the correct one (see :ref:`linker_script`).
* modify the architecture used by ``CHIBIOS_BOARD_PLATFORM`` and ``CHIBIOS_BOARD_STARTUP``.
* this file can also contain LED, UART, radio and actuator default definitions.


Using this Makefile, we need to create an XML file defining the board. This file must have the same name has the makefile ``<board_name>_<version>.xml`` and located in `conf/module/boards <https://github.com/paparazzi/paparazzi/tree/master/conf/modules/boards>`_.


Board module
----------------

It's very useful to define a module in `conf/module/ <https://github.com/paparazzi/paparazzi/tree/master/conf/modules/>`_, naming it board_****.xml, to load all the sensor modules on board. This is also an opportunity to finish configuring the communication ports with the sensors.

.. note::
    It's important to fill in the header of this file, as it represents the documentation for this board.  


Airframe test
----------------

When integrating a new board, you can define an airframe file in `conf/airframes/test_boards <https://github.com/paparazzi/paparazzi/tree/master/conf/airframes/test_boards>`_.