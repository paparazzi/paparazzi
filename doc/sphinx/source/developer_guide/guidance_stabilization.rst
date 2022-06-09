.. developer_guide guidance_stabilization

===========================
Guidance and Stabilization
===========================

hello

Structure overview
---------------------

Graph/image with function calls and files involved in stab/guidance pipeline

Configuring your airframe
---------------------------

Adding stab/guidance modules to airframe + selecting type

Guidance
------------
Discuss vertical / horizontal split and give overview of input + output
Discuss link to MODES (only rotorcraft?)

Vertical Guidance
^^^^^^^^^^^^^^^^^^^^^
hello

Horizontal Guidance
^^^^^^^^^^^^^^^^^^^^^^
hello

Autopilot Modes
^^^^^^^^^^^^^^^^^
List of MODES and what they control

MODULE Mode
^^^^^^^^^^^^^
Explain MODULE Mode and how to create your own

Reference model
^^^^^^^^^^^^^^^^^^
Brief discussion of setpoint reference model

Stabilization
-----------------
Discuss structure + overview of input/output
Link to previous section to show how each module plugs into stabilization depending on airframe configuration

Motor Mixing
-----------------
(Not sure this belongs to Guidance and Stabilization)
Describe what happens between output of Stabilization and setting actuator values

Setting actuator values directly
----------------------------------
Alternative to using stabilization to output COMMANDS, instead calculating actuator values and setting them
in the command_laws directly
