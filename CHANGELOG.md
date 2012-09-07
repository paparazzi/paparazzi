Paparazzi 4.0.2
===============

Maintenance release

- fix some illegal xml chars in xml description for xtend_rssi and pwm_meas modules
- fix ins_arduimu_basic by adding an empty ahrs_update_gps function
- fix some ivy includes on OSX for weird installation cases
- fix ACTUATORS_PWM_NB for fixedwings with USE_SERVOS_7AND8
- updated kestrel weather meter agent
- fix uart_tunnel if leds are not available on some boards
- fix GPS_LLA message
- fix make failure when trying to get google maps version, but no internet connection
  [#187] (https://github.com/paparazzi/paparazzi/issues/187)


Paparazzi 4.0.1
===============

Maintenance release

- Serious fix for settings:
    - Handler was not called if module was not specfied as well.
      Now an error is generated with info on which file to fix.
    - Settings for adaptive fw stabilization fixed.
- Fixes for photogrammetry module.
- Fixes for digital_cam modules.
- Tool added: mergelogs


Paparazzi 4.0
=============

Changes since old_master tag

General
-------

- Settings for the telemetry modes are automatically generated from the XML file
  [#118] (https://github.com/paparazzi/paparazzi/pull/118)
- Documentation node for modules
  [#182] (https://github.com/paparazzi/paparazzi/pull/182)
- Automatic conversion of units in airframe file, settings and messages.
  See http://paparazzi.enac.fr/wiki/Units
- Fix rc_settings: this enables you to change some settings in flight
  directly from the RC, is useful if you are alone or don't have a GCS.
- Prefer compiler found in PATH over /opt/paparazzi/arm-multilib
  [#231] (https://github.com/paparazzi/paparazzi/issues/231)
- Usability improvements for calibration scripts and
  added 3D view of magnetometer data with fitted ellipsoid

New hardware support
--------------------

- Support for new autopilot boards
    - [Umarim] (http://paparazzi.enac.fr/wiki/Umarim_v10)
    - [Umarim Lite] (http://paparazzi.enac.fr/wiki/Umarim_Lite_v2)
    - [NavGo] (http://paparazzi.enac.fr/wiki/NavGo_v3)
    - [Lisa/M 2.0] (http://paparazzi.enac.fr/wiki/Lisa/M_v20)
- IMU Aspirin 2.1 support
- BlackMagicProbe JTAG support

Airborne
--------

- All control gains are now positive
  [#127] (https://github.com/paparazzi/paparazzi/pull/127)
- RC input follows sign conventions
  [#124] (https://github.com/paparazzi/paparazzi/issues/124)
- A modification of the transport layer (pprz and xbee)
  in order to allow to select the device at the message level.
- New modules:
    - xtend_rssi
      [#88] (https://github.com/paparazzi/paparazzi/pull/88)
    - open_log
      [#82] (https://github.com/paparazzi/paparazzi/pull/82)
- Subsystem for new ahrs estimation algorithms: float_cmpl_rmat
- Improvements for AHRS int_cmpl_quat and float_cmpl_rmat
    - Correction of centrifugal acceleration
    - Proper handling of BODY_TO_IMU rotations
- All status LEDs configurable (with sensible defaults for the boards):
  SYS_TIME_LED, AHRS_ALIGNER_LED, BARO_LED, GPS_LED, RADIO_CONTROL_LED
- Possibility to use two 2-way switches for the mode instead of one 3-way switch
- GPS NMEA parser usable for basic position and fix
  [#120] (https://github.com/paparazzi/paparazzi/issues/120)

Rotorcraft firmware specific
----------------------------

- Stabilization/supervision commands with standard PPRZ range
  [#169] (https://github.com/paparazzi/paparazzi/pull/169)
- Additional motor arming options
  [#174] (https://github.com/paparazzi/paparazzi/pull/174)
- Replaced INV_M with NOMINAL_HOVER_THROTTLE (in %)
  To use a fixed value instead of the adaptive vertical filter
  [#177] (https://github.com/paparazzi/paparazzi/pull/177)
- Some fixes when changing vertical guidance modes
- Same behaviour (gains) for AP_MODE_HOVER and NAV when holding position
  [#82] (https://github.com/paparazzi/paparazzi/pull/82)

Fixedwing firmware specific
---------------------------

- Using a gyro (with IR sensors) is done via imu subsystem now as well

Simulator
---------

- JSBSim interface updated for new FGAccelerations class
- FlightGear interface defaults to version 2.6, define FG_2_4 for 2.4
- NPS simulator [#205] (https://github.com/paparazzi/paparazzi/pull/205)
    - has it's own nps target (instead of sim)
    - fdm type renamed from nps to jsbsim
    - waypoint altitude fixed
    - Improved ground interaction for JSBSim, can now initialize on ground
      [#222] (https://github.com/paparazzi/paparazzi/pull/222)
    - Radio control via joystick now uses SDL (so works on OSX as well)
      [#232] (https://github.com/paparazzi/paparazzi/pull/232)


STM32 architecture
------------------

- Luftboot USB bootloader
- Updated ADC defines for lisa/m
  You should now be able to use ADC_1, ADC_2, ADC_3 for the ADCs on the ANALOG1
  [#159] (https://github.com/paparazzi/paparazzi/issues/159)
- Enable second spektrum receiver via
  ```<configure name="USE_SECONDARY_SPEKTRUM_RECEIVER" value="1"/>```
- Enable new I2C driver via
  ```<configure name="USE_NEW_I2C_DRIVER"/>```
