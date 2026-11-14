.. user_guide tools

======================
Paparazzi Center tools
======================

These are the tools of the *Add tool* menu of the Paparazzi Center, one per file in ``conf/tools``.
Most of them can also be started from a terminal with the options listed below.

The description and options of each tool come from its ``--help`` text, stored in the ``<doc>`` block
of its xml file. After changing the options of a tool, update that block with
``tests/tools/test_tools.py --update`` (``make test_tools`` checks that they match).
A ``<description>`` written by hand in the ``<doc>`` block is kept.

.. paparazzi-tools::
