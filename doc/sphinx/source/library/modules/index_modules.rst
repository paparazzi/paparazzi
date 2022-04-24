.. library modules

=============
Modules List
=============

Over the years the Paparazzi project has been populated by a moltitude of various modules that allow to implement various features and abilities to 
airframes. This page gathers them all and provides a description and insight of each module, together with a link to the actual code of the module. 


.. toctree::
    :hidden:

    modules_list/cv_opticflow
    modules_list/adc_generic

.. list-table:: Analog-digital converters
   :widths: 25 25 25 50
   :header-rows: 1

   * - Name
     - Directory
     - File
     - Description
   * - :doc:`modules_list/adc_generic`
     - ascs
     - `adc_generic.xml <http://docs.paparazziuav.org/latest/module__adc_generic.html>`_
     - autopilot internal 10 bit AD converter
   * - max11040
     - ascs
     - `max11040.xml <http://docs.paparazziuav.org/latest/module__max11040.html>`_
     - 24 bit 16 channel AD converter
   * - mcp355x
     - ascs
     - `mcp355x.xml <http://docs.paparazziuav.org/latest/module__mcp355x.html>`_
     - 22 bit 1 channel AD converter

.. list-table:: Vision Modules
   :widths: 25 25 25 50
   :header-rows: 1

   * - Name
     - Directory
     - File
     - Description
   * - :doc:`modules_list/cv_opticflow`
     - computer_vision
     - `cv_opticflow.xml <http://docs.paparazziuav.org/latest/module__cv_opticflow.html>`_
     - optical flow module 