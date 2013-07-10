Paparazzi 5.0.0_stable
======================

Stable version release

General
-------

- STM libs completely replaced by libopencm3
- [gcc-arm-embedded] (https://launchpad.net/gcc-arm-embedded) is the new recommended toolchain
- Use findlib (ocamlfind) for ocaml packages. Faster build.
  [#274] (https://github.com/paparazzi/paparazzi/pull/274)
- Building/Running the groundsegment on an ARM (e.g. RaspberryPi).
- Input2ivy uses SDL for joysticks (cross-platform, works on OSX as well now)
  [#220] (https://github.com/paparazzi/paparazzi/pull/220)
- Option to change text papget color using a combobox
  [#194] (https://github.com/paparazzi/paparazzi/pull/194)
- Redundant communications
  [#429] (https://github.com/paparazzi/paparazzi/pull/429)
- Log also contains includes like procedures now, so replay if these missions is possible.
  [#227] (https://github.com/paparazzi/paparazzi/issues/227)
- Paparazzi Center
    - New simulation launcher with dialog and detection of available ones.
      [#354] (https://github.com/paparazzi/paparazzi/pull/354)
    - Checkbox to print extra configuration information during build.
- GCS:
    - Fix panning with mouse if there are no background tiles.
      [#9] (https://github.com/paparazzi/paparazzi/issues/9)
    - Higher zoom level for maps.
      [#277] (https://github.com/paparazzi/paparazzi/issues/277)

Hardware support
----------------

- initial support for STM32F4
    - Apogee autopilot
    - KroozSD autopilot
- Parrot AR Drone 2 support: raw and sdk versions
- CH Robotics UM6 IMU/AHRS
- GPS/INS XSens Mti-G support
- GPS Sirf support
- GPS Skytraq now usable for fixedwings as well
  [#167] (https://github.com/paparazzi/paparazzi/issues/167)
- Mikrokopter V2 BLDC
  [#377] (https://github.com/paparazzi/paparazzi/pull/377)
- PX4Flow sensor
  [#379] (https://github.com/paparazzi/paparazzi/pull/379)
- Dropped AVR support

Airborne
--------

- State interface with automatic coordinate transformations
  [#237] (https://github.com/paparazzi/paparazzi/pull/237)
- New AHRS filter: Multiplicative quaternion linearized Kalman Filter
- New SPI driver with transaction queues.
    - Fix transactions with zero length input.
      [#348] (https://github.com/paparazzi/paparazzi/issues/348)
- Peripherals: Cleanup and refactoring.
    - MPU60x0 peripheral supporting SPI and I2C with slave.
- UDP datalink.
- Magnetometer current offset calibration.
  [#346] (https://github.com/paparazzi/paparazzi/pull/346)
- Gain scheduling module.
  [#335] (https://github.com/paparazzi/paparazzi/pull/335)

Rotorcraft firmware specific
----------------------------

- Quadshot transitioning vehicle support.
- Care Free Mode


Paparazzi 4.2.1_stable
======================

Maintenance release

- fix elf PT_LOAD type in lpc21iap LPC USB download
- fix electrical.current estimate in sim
- fix LPC+xbee_api in rotorcraft
- fix conversion of vsupply to decivolts if offset is used
- more robust dfu flash script, only upload to Lisa/M


Paparazzi 4.2.0_stable
======================

Stable version release

Since last stable v4.0:
- Total energy control
- Improve Google map tiles download
- Several updates on the Digital Cam and photogrammetry modules
- WMM210 model
- Rate limiter in airframes control laws
- Uart flow control (stm32)
- Bug fix for INS and AHRS filters
- AP/FBW separation using spi or uart
- Sensors fix and addition (GPS, current, baro)

Paparazzi 4.1.1_testing
=======================

Second release candidate for next stable release.

- Run AP and FBW on separate boards
  [#297] (https://github.com/paparazzi/paparazzi/pull/297)
- Separate Board Files for yapa_v2.0
  [#303] (https://github.com/paparazzi/paparazzi/pull/303)
- Add UART hardware flow control (for STM32 only)
  [#289] (https://github.com/paparazzi/paparazzi/pull/289)
- Add ezosd current sensor module
  [#292] (https://github.com/paparazzi/paparazzi/pull/292)
- Paparazzi Center: fix coloration by passing input buffer line by line, set language to english
- Rate limiter bugfix
- DC_SHOT message photo numbers shown in GCS
- home mode height can be set different from security height
- Ahrs float_dcm uses magnetic heading while not inflight for better initial guess
  [#299] (https://github.com/paparazzi/paparazzi/pull/299)
- Outback Challenge "Safety" Rules 1 and 2
- Added WMM2010 Geo model used in ahrs int_cmpl_quat for rotorcrafts
  [#288] (https://github.com/paparazzi/paparazzi/pull/288)
- Fix missing gain definitions from airframe file in total energy control
- Added stm32loader to sw/tools
- Change vsupply to be a uint16 to enable reporting of voltages higher than 25.5V
  [#294] (https://github.com/paparazzi/paparazzi/issues/294)
- Digital cam module: release camera button on init
- Fix second order term in propagation of x-position in HFF

Paparazzi 4.1.0_testing
=======================

First release candidate for next stable release.

- Total energy control
  [#251] (https://github.com/paparazzi/paparazzi/pull/251)
- PPM input on UART1 RX for Lisa/M autopilots
- Rate Limiter for Flaps, Gears and servo hatches
  [#252] (https://github.com/paparazzi/paparazzi/pull/252)
- GPS acceleration compensation in ahrs_float_dcm
  [#255] (https://github.com/paparazzi/paparazzi/pull/255)
- Improved gravity heuristic for int_cmpl_quat
- Some updates on digital_cam and photogrammetry modules
  [#250] (https://github.com/paparazzi/paparazzi/pull/250)
- Replace wget by OCaml Http_client from netclient lib to download files
  [#276] (https://github.com/paparazzi/paparazzi/pull/276)


Paparazzi 4.0.3
===============

Maintenance release

- fix google map version download
- fix BMP scripts for stm32
- fix lisa/m 2.0 default voltage
- fix gpsd2ivy for libgps3.5
- improve some makefiles


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
