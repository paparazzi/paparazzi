.. user_guide simulation_main

===========================
Simulation
===========================

Paparazzi currently has two different simulator targets with different degrees of realism and intended purpose:

- **sim**: The basic fixedwing simulator written in OCaml without IMU simulation or any sensor models (noise, bias, etc) and mainly intended to validate your :ref:`flightplans` logic.
- **nps**: NPS is a more advanced rotorcraft and fixedwing simulator with sensor models and commonly uses JSBSim as FDM (Flight Dynamic Model). 
  Other FDM's can be integrated easily. At the moment CRRCSIM, YASIM and JSBSIM are tried as FDM backend.
- **gazebo**: There is someting brand new developed going, using laters Master you can start using the Gazebo engine in Paparazzi. 
  Take a look on that page to see if it offers what you are looking for.

A FDM is a set of mathematical equations used to calculate the physical forces acting on a simulated aircraft, such as thrust, lift, and drag.

.. toctree::
    :maxdepth: 1

    simulation/sim
    simulation/nps
    simulation/gazebo
    simulation/hitl