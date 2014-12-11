Paparazzi 5.3.0_testing
=======================

First release candidate for v5.4 stable release.

General
-------

- Flight plans: option to `call` functions once without checking return value
  [830] (https://github.com/paparazzi/paparazzi/pull/830)
- Paparazzi Center settings improvements
  [#834] (https://github.com/paparazzi/paparazzi/pull/834)
- replay: ignore non-telemetry messages to remove warnings
  [#894] (https://github.com/paparazzi/paparazzi/issues/894)
- maps: put google tiles in var/maps/Google instead of var/maps
  [#902] (https://github.com/paparazzi/paparazzi/issues/902)
- Paparazzi Center: improve warning coloring
  [#910] (https://github.com/paparazzi/paparazzi/issues/910)
- add INFO_MSG with printing to GCS console
  [#929] (https://github.com/paparazzi/paparazzi/pull/929)
- Remove array delimiters on Ivy messages
  [#942] (https://github.com/paparazzi/paparazzi/pull/942)
- improve test framework
  [#933] (https://github.com/paparazzi/paparazzi/pull/933)
  [#945] (https://github.com/paparazzi/paparazzi/pull/945)
- GCS: save size in layout
  [#968] (https://github.com/paparazzi/paparazzi/issues/968)
- link/GCS: improve datalink/telemetry report and display link page in GCS for single link
  [#999] (https://github.com/paparazzi/paparazzi/pull/999)
- build: default to parallel make for aircrafts
  [#1002] (https://github.com/paparazzi/paparazzi/pull/1002)
- select_conf.py fixes and also selects control_panel.xml
  [#1001] (https://github.com/paparazzi/paparazzi/pull/1001)
- DFU flashing: CRC support and longer timeout for Krooz
  [#997] (https://github.com/paparazzi/paparazzi/pull/997)
  [#691] (https://github.com/paparazzi/paparazzi/pull/691)
- messages: possibility to add descriptions
  [#987] (https://github.com/paparazzi/paparazzi/pull/987)
- messages: mission modules use LLA waypoints in 1e7deg instead of float
  [#986] (https://github.com/paparazzi/paparazzi/pull/986)
- GCS: alert/console window: always insert messages at the end
  [#996] (https://github.com/paparazzi/paparazzi/issues/996)
- improve dependency handling for modules
  [#1007] (https://github.com/paparazzi/paparazzi/pull/1007)
- Settings: display unconfirmed settins with "?" as current value
  [#1013] (https://github.com/paparazzi/paparazzi/pull/1023)
- messages/plotter: improve drag-and-drop of fields from messages to realtime plotter
  [#1020] (https://github.com/paparazzi/paparazzi/pull/1020)

Simulation
----------

- OCaml sim: simulate sys_time
  [#962] (https://github.com/paparazzi/paparazzi/issues/962)
- OCaml sim: use unconnected socket for flightgear viz
  [#915] (https://github.com/paparazzi/paparazzi/issues/915)
- NPS: add commandline option to set time_factor
- radio_conrol spektrum for sim target
  [#851] (https://github.com/paparazzi/paparazzi/pull/851)
- OCaml sim: sliders in simulated RC always sensitive
  [#821] (https://github.com/paparazzi/paparazzi/issues/821)

Airborne
--------

- pass dt to ahrs/ins propagation
  [#818] (https://github.com/paparazzi/paparazzi/pull/818)
- cleanup math lib and convert lots of macros to functions
  [#819] (https://github.com/paparazzi/paparazzi/pull/819)
- radio_control spektrum also usable for intermcu
  [#847] (https://github.com/paparazzi/paparazzi/pull/847)
- Replace telemetry macros with functions
  [#931] (https://github.com/paparazzi/paparazzi/pull/931)
  [#1027] (https://github.com/paparazzi/paparazzi/pull/1027)
- arch: rename arch/omap to arch/linux
  [#982] (https://github.com/paparazzi/paparazzi/pull/982)
- radio_control: cleanup channel defines and possibility to send less than available via intermcu
  [#975] (https://github.com/paparazzi/paparazzi/pull/975)
- state interface: change computation order in stateCalcPositionLla_i
  [#1013] (https://github.com/paparazzi/paparazzi/pull/1013)
- ARDrone2: Handle memory full FTP upload error
  [#967] (https://github.com/paparazzi/paparazzi/issues/967)
- rotorcraft: force MODE_STARTUP instead of KILL until ahrs is aligned
  [#983] (https://github.com/paparazzi/paparazzi/pull/983)
- rotorcraft: fix NavCircleCount()
- rotorcraft: datalink: check ac_id of RC_4CH message
- rotorcraft: allow to turn off motors in failsafe mode
  [#989] (https://github.com/paparazzi/paparazzi/pull/989)

Modules
-------

- Convert air_data subsystem to module with QNH and true airspeed support
  [#853] (https://github.com/paparazzi/paparazzi/pull/853)
- add airspeed_ms45xx_i2c module
  [#852] (https://github.com/paparazzi/paparazzi/pull/852)
- airspeed_ets, retry after failed transaction
- add temperature adc module
  [#857] (https://github.com/paparazzi/paparazzi/pull/857)
- clean up digital_cam, usable for rotorcrafts, show real photo coordinates in GCS
  [#936] (https://github.com/paparazzi/paparazzi/pull/936)
- modules: add basic mavlink module
  [#1028] (https://github.com/paparazzi/paparazzi/pull/1028)
- modules: improved video/images sending for ARDrone2
  [#1021]: (https://github.com/paparazzi/paparazzi/pull/1021)

Drivers/HW support
------------------

- stm32: usb_serial (CDC) impelmentation for transparent_usb telemetry
  [#998] (https://github.com/paparazzi/paparazzi/pull/998)
- stm32: add usb_tunnel
  [#1014] (https://github.com/paparazzi/paparazzi/pull/1014)
- Add Furuno NMEA based GPS
  [#959] (https://github.com/paparazzi/paparazzi/pull/959)
- Driver for MPU9250
  [#953] (https://github.com/paparazzi/paparazzi/pull/953)
- Driver for AKM8963 magnetometer
  [#947] (https://github.com/paparazzi/paparazzi/pull/947)
- linux: add basic I2C and SPI drivers
  [#961] (https://github.com/paparazzi/paparazzi/pull/961)
  [#979] (https://github.com/paparazzi/paparazzi/pull/979)
- actuators: basic esc32 motor controller implementation via CAN for STM32F1
  [#1004] (https://github.com/paparazzi/paparazzi/pull/1004)
- basic support for new Parrot Bebop
  [#1003] (https://github.com/paparazzi/paparazzi/pull/1003)


Paparazzi 5.2.1_stable
======================

Maintenance release

- build system: remove 'load' target as it is a builtin directive in Make 4.0
- fix FlightGear visualization on 32bit systems
- flight plans: set primitive should not delay next stage
  [#824] (https://github.com/paparazzi/paparazzi/pull/824)
- flight plans: fix return primitive
- generators: gen_airframe: don't force float if unit = code_unit
- sys_time: up to 16 sys_time timers by default (was 8)
- OCaml: fix Pprz.sprint_value for uint32, e.g. for NatNet
  [#831] (https://github.com/paparazzi/paparazzi/issues/831)
- Rotorcraft: auto-enable UNLOCKED_HOME_MODE if HOME mode is used on RC
  [#823] (https://github.com/paparazzi/paparazzi/issues/823)
- Rotorcraft: only go to HOME mode if in NAV
  [#850] (https://github.com/paparazzi/paparazzi/issues/850)
- Fixedwing: If USE_BARO_BOARD: separate baro timer
  Before baro_periodic was running at PERIODIC_FREQUENCY in sensors_task,
  which is too fast for ms5611 if periodic freq is > 100Hz.
- INS alt_float: if USE_BARO_BOARD, dt is 1/BARO_PERIODIC_FREQUENCY
  [#848] (https://github.com/paparazzi/paparazzi/pull/848)
- STM32: backport fix for using multiple ADs
  [#822] (https://github.com/paparazzi/paparazzi/pull/822)
- LPC21: don't override T0CCR values when setting ppm and trig_ext
- IMU driver for Lisa/M/MX 2.1
  [#817] (https://github.com/paparazzi/paparazzi/pull/817)
- support for HBmini board
  [#864] (https://github.com/paparazzi/paparazzi/pull/864)


Paparazzi 5.2.0_stable
=======================

Stable version release.

General
-------

- ocaml/link: fix uint32 parsing
  [#809] (https://github.com/paparazzi/paparazzi/pull/809)
- modules: use VPATH to make it easier to load external modules
  [#760] (https://github.com/paparazzi/paparazzi/pull/760)
- ground_segment: try to update AGL even if GPS is lost
  [#742] (https://github.com/paparazzi/paparazzi/issues/742)
- ground_segment: different icons for the different applications of paparazzi
  [#787] (https://github.com/paparazzi/paparazzi/pull/787)
- generators: fix generated longitude in 1e7deg on 32bit systems
  [#808] (https://github.com/paparazzi/paparazzi/issues/808)
- lat/lon int in 1e7deg instead of 1e7rad
  [#810] (https://github.com/paparazzi/paparazzi/pull/810)
- GCS: fix inaccurate waypoint updates
  [#762] (https://github.com/paparazzi/paparazzi/issues/762)
- GCS: add exponentiation operator to papgets
- logalizer: export lat/lon with 9 decimal places
- GCS: display AP mode forward correctly
  [#748] (https://github.com/paparazzi/paparazzi/pull/748)
- GCS: also center aircraft on uppercase C and fix listed keys in help
  [#803] (https://github.com/paparazzi/paparazzi/issues/803)
- settings: add spin button widget for numeric input instead of sliders
  [#795] (https://github.com/paparazzi/paparazzi/issues/795)
- tools: flash via bmp: reset and detach from target after uploading
  [#746] (https://github.com/paparazzi/paparazzi/issues/746)
- tools: DFU flashing: allow 1 BIT SQUARED vendor
  [#778] (https://github.com/paparazzi/paparazzi/pull/778)
- tools: add tcp_aircraft_server
  [#750] (https://github.com/paparazzi/paparazzi/pull/750)
- tools: add tool to syncronize video on replay
  [#770] (https://github.com/paparazzi/paparazzi/pull/770)
  [#776] (https://github.com/paparazzi/paparazzi/pull/776)
  [#789] (https://github.com/paparazzi/paparazzi/pull/789)
- messages: rename class to msg_class
  [#812] (https://github.com/paparazzi/paparazzi/pull/812)
- messages: add GEO_MAG message to send magnetic field
  [#735] (https://github.com/paparazzi/paparazzi/pull/735)
- messages: add speed setpoint to GUIDANCE_H_REF_INT message
  [#763] (https://github.com/paparazzi/paparazzi/issues/763)
- NPS: block and setting messages only parsed for correct AC_ID
  [#777] (https://github.com/paparazzi/paparazzi/pull/777)
- NPS: fix warnings when compiling with clang
  [#790] (https://github.com/paparazzi/paparazzi/pull/790)
- gaia: command line options for environment simulator
  [#799] (https://github.com/paparazzi/paparazzi/issues/799)
- build: only set PAPARAZZI_SRC if not already externally set
  [#800] (https://github.com/paparazzi/paparazzi/issues/800)

Airborne
--------

- fixedwing: correct altitude setpoint during NavGlide
  [#785] (https://github.com/paparazzi/paparazzi/pull/785)
- flight plans: don't set nav_pitch to 0 at each stage init
  [#727] (https://github.com/paparazzi/paparazzi/pull/727)
- modules: gps_ubx_ucenter: fix version check for Ublox 7
- modules: add HackHD digital camera control module
- modules: add CSV file logger for ARDrone
  [#788] (https://github.com/paparazzi/paparazzi/pull/788)
- modules: geo_mag: fixedwing compatibility
  [#806] (https://github.com/paparazzi/paparazzi/issues/806)
- ardrone2: mag freeze fix
  [#767] (https://github.com/paparazzi/paparazzi/pull/767)
- boards: add support for navstik
  [#744] (https://github.com/paparazzi/paparazzi/pull/744)
- boards: fix PWM on lisa_l
- boards: lisa_m_2.0: baro defaults to BARO_MS5611_SPI
- peripherals: ms5611: check temp an pressure range
  [#758] (https://github.com/paparazzi/paparazzi/issues/758)
- imu: aspirin_2_spi: wait 1.5s before configuring mag
  [#779] (https://github.com/paparazzi/paparazzi/pull/779)
- imu: body_to_imu adjustable during runtime via settings
  [#783] (https://github.com/paparazzi/paparazzi/pull/783)
- ahrs: int_cmpl_quat: fix rate integration range/resolution
  [#782] (https://github.com/paparazzi/paparazzi/pull/782)
- ins: properly define INS_VFF_R_GPS
  [#741] (https://github.com/paparazzi/paparazzi/issues/741)
- ins: allow define of VFF_R_SONAR_OF_M
  [#764] (https://github.com/paparazzi/paparazzi/issues/764)
- electrical: min bat level check
  [#745] (https://github.com/paparazzi/paparazzi/issues/745)
- electrical: fix for negative currents
  [#753] (https://github.com/paparazzi/paparazzi/issues/753)
- electrical: 32bit for bat low and critical counters for longer delays
  [#805] (https://github.com/paparazzi/paparazzi/issues/805)
- messages: dist_home and dist_wp in meters for fixedwings and rotorcrafts
  [#784] (https://github.com/paparazzi/paparazzi/pull/784)
- superbitrf: save bind settings to flash (if USE_PERSISTENT_SETTINGS)
  [#792] (https://github.com/paparazzi/paparazzi/issues/792)

Rotorcraft Firmware
-------------------

- RC input: zero yaw command if throttle is zero
  [#737] (https://github.com/paparazzi/paparazzi/pull/737)
- prevent motor arming in kill mode
  [#740] (https://github.com/paparazzi/paparazzi/pull/740)
- change rotorcraft nav API to use points rather than wp id
  [#749] (https://github.com/paparazzi/paparazzi/pull/749)
- use roll/pitch RC deadbands in attitude mode
  [#773] (https://github.com/paparazzi/paparazzi/pull/773)
- route precision fix
  [#775] (https://github.com/paparazzi/paparazzi/pull/775)
- guidance_v: limit z_ref in update_ref_from_zd_sp
  [#754] (https://github.com/paparazzi/paparazzi/pull/754)
- mission module for rotorcrafts
  [#759] (https://github.com/paparazzi/paparazzi/pull/759)
- guidance_v: only limit throttle if RC ok
  [#766] (https://github.com/paparazzi/paparazzi/pull/766)
- navigation: fix waypoint initialization in ENU
  [#791] (https://github.com/paparazzi/paparazzi/pull/791)

STM32 architecture
------------------

- fix I2C bitrate on F4
  [#729] (https://github.com/paparazzi/paparazzi/pull/729)
- fix sys_time_usleep
  [#739] (https://github.com/paparazzi/paparazzi/pull/739)
- fix spektrum on F4
  [#732] (https://github.com/paparazzi/paparazzi/pull/732)
- timer frequency cleanup
  [#734] (https://github.com/paparazzi/paparazzi/pull/734)
- split spektrum uart rx and config pin for Apogee
  [#733] (https://github.com/paparazzi/paparazzi/pull/733)
- add suport for pwm input
- fix reset for I2C3
  [#751] (https://github.com/paparazzi/paparazzi/pull/751)
- simplify actuators_pwm
  [#757] (https://github.com/paparazzi/paparazzi/pull/757)
- boards: add files for Lisa/M and Lisa/MX v2.1
  [#813] ((https://github.com/paparazzi/paparazzi/pull/813)
- spektrum: configure the bind pin to be pullup/pulldown
  [#814] (https://github.com/paparazzi/paparazzi/pull/814)


Paparazzi 5.1.1_testing
=======================

Second release candidate for v5.2 stable release.

General
-------

- GCS: higher default maps zoom level
  [#725] (https://github.com/paparazzi/paparazzi/pull/725)
- Allow settings/modules/flightplans outside the conf dir
  [#723] (https://github.com/paparazzi/paparazzi/pull/723)
- optitrack: Give feedback about following drones and fix gps
  [#718] (https://github.com/paparazzi/paparazzi/pull/718)
- dfu-util: only attempt verify for version >= 0.7
  [#697] (https://github.com/paparazzi/paparazzi/issues/697)
- dfu-util: fix DFU_SIZE on OSX
- add prototype for python based airframe file editor

Airborne
--------

- Modules: rewrite humid_sht using gpio interface (supporting STM as well as LPC now)
  [#721] (https://github.com/paparazzi/paparazzi/pull/721)
- INS int: removed INS_SONAR_VARIANCE_THRESHOLD, INS_SONAR_MIN_RANGE defaults to 1mm
- fix compilation of SuperbitRF telemetry subsystem
- possibility to poweron gps and imu via gpio at init
  [#706] (https://github.com/paparazzi/paparazzi/pull/706)
- bmp085 peripheral: Fix calibration reading with errors

Simulation
----------

- NPS: simulate sonar sensor
  [#720] (https://github.com/paparazzi/paparazzi/pull/720)
- JSBSim, NPS: fix roll input sign
- NPS: fixedwing simulation improvements
  - If NPS_JSBSIM_LAUNCHSPEED is defined, set it as initial launchspeed.
  - Only launch when launch button is pressed in GCS instead of immediately at takeoff block.

Rotorcraft firmware
-------------------

- add some functions to set heading via flight plan
  [#724] (https://github.com/paparazzi/paparazzi/pull/724)
- fix reference in hover with USE_SPEED_REF, set DEFAULT_CIRCLE_RADIUS to 5m
  [#716] (https://github.com/paparazzi/paparazzi/issues/716)
  [#717] (https://github.com/paparazzi/paparazzi/pull/717)
- implement approaching_time for "go" flight plan primitve
  [#715] (https://github.com/paparazzi/paparazzi/pull/715)

Fixedwing firmware
------------------

- stabilization adaptive: loiter correction and reference generator update
  [#711] (https://github.com/paparazzi/paparazzi/pull/711)
- stabilization new/adaptive: USE_GYRO_PITCH_RATE defaults to TRUE
- dual mcu: Workaround RADIO_CONTROL_NB_CHANNELS differs in driver and radio.h
  [#700] (https://github.com/paparazzi/paparazzi/pull/700)


Paparazzi 5.1.0_testing
=======================

First release candidate for next stable release.

General
-------

- lots of cleanup
- GCS: higher max zoom level
  [#632] (https://github.com/paparazzi/paparazzi/issues/632)
- GCS: different aircraft icons (added flying wing, quadrotor)
- GCS: proper GPSd home icon resizing
  [#601] (https://github.com/paparazzi/paparazzi/issues/601)
- GUI for selecting the desired active list of airframes (select_conf.py)
  [#536] (https://github.com/paparazzi/paparazzi/issues/536)
- paparazzi center: flash mode selection via drop down menu
  [#597] (https://github.com/paparazzi/paparazzi/pull/597)
- add support for new telemetry types
  [#589] (https://github.com/paparazzi/paparazzi/pull/589)
- build aircraft firmware in var/aircrafts/'ac_name'
  [#601] (https://github.com/paparazzi/paparazzi/issues/601)
- parallel build of firmwares with J=AUTO
  [#683] (https://github.com/paparazzi/paparazzi/pull/683)
- add simple verify to flashing via dfu-util
  [#673] (https://github.com/paparazzi/paparazzi/pull/673)
- app_server: connection between the ground station and a GCS Android application
  (https://github.com/paparazzi/PPRZonDroid)
- GCS: adapt home (from GPSd) icon to zoom level
  [#679] (https://github.com/paparazzi/paparazzi/issues/679)

Simulation
----------

- simulate datalink loss
  [#631] (https://github.com/paparazzi/paparazzi/issues/631)
- FlightGear viz: daytime everywhere
  [#555] (https://github.com/paparazzi/paparazzi/issues/555)

Hardware support
----------------

- remove sys_plugs for STM32, finally dropping old toolchain support
  [#688] (https://github.com/paparazzi/paparazzi/pull/688)
- Support for all 3 ADCs of F4
  [#551] (https://github.com/paparazzi/paparazzi/issues/551)
- new peripheral drivers:
  - ST LIS302DL accelerometer (SPI)
  - ST L3GD20 gyro (SPI)
  - ST LSM303DLHC 3D accelerometer and magnetometer (I2C)
  - converted barometers MS5611, BMP085 to generic peripherals
    [#515] (https://github.com/paparazzi/paparazzi/pull/515)
- I2C watchdog for STM32
  [#662] (https://github.com/paparazzi/paparazzi/pull/662)
- Dual PWM servo driver
  [#678] (https://github.com/paparazzi/paparazzi/pull/678)
- SBus radio control driver (single and dual receivers)
  [#485] (https://github.com/paparazzi/paparazzi/pull/485)
  [#674] (https://github.com/paparazzi/paparazzi/pull/674)
  [#693] (https://github.com/paparazzi/paparazzi/pull/693)

Airborne
--------

- API function to periodically send telemetry messages: register_periodic_telemetry
  [#472] (https://github.com/paparazzi/paparazzi/pull/472)
- generic gpio interface
  [#498] (https://github.com/paparazzi/paparazzi/issues/498)
  [#651] (https://github.com/paparazzi/paparazzi/issues/651)
- gps_ubx_ucenter module improvements
  [#646] (https://github.com/paparazzi/paparazzi/issues/646)
  [#653] (https://github.com/paparazzi/paparazzi/issues/653)
- HOTT telemetry module added
  [#591] (https://github.com/paparazzi/paparazzi/pull/591)
- GPS subsystem to stream external position data to the vehicle over UDP
  [#630] (https://github.com/paparazzi/paparazzi/pull/630)
- INS reset/realign API updates
  [#644] (https://github.com/paparazzi/paparazzi/pull/644)
- INS alt_float: remove ALT_KALMAN_ENABLED
  [#594] (https://github.com/paparazzi/paparazzi/issues/594)
- IIR filter for horizontal position Kalman Filter
  [#677] (https://github.com/paparazzi/paparazzi/pull/677)
- barometer interface via ABI
  [#525] (https://github.com/paparazzi/paparazzi/pull/525)
  - baros always output pressure in Pascal
  - standard atmosphere model is used to convert pressure to altitude
  - INS_BARO_SENS is hence not needed anymore
- sonar interface using ABI
  [#654] (https://github.com/paparazzi/paparazzi/pull/654)
- AHRS int_cmpl_quat frequency scaling
  [#371] (https://github.com/paparazzi/paparazzi/pull/371)

Rotorcraft firmware
-------------------

- AR Drone 2 updates
  [#626] (https://github.com/paparazzi/paparazzi/issues/626)
  [#598] (https://github.com/paparazzi/paparazzi/pull/598)
- guidance improvements
  [#539] (https://github.com/paparazzi/paparazzi/pull/539)
  [#546] (https://github.com/paparazzi/paparazzi/pull/546)
- horizontal guidance: dynamically adjustable max_speed
  [#664] (https://github.com/paparazzi/paparazzi/pull/664)
- adaptive thrust estimation limits
  [#495] (https://github.com/paparazzi/paparazzi/issues/495)
- improve in_flight detection heuristic
  [#469] (https://github.com/paparazzi/paparazzi/pull/469)
- stabilization quaternion: fix scale of angle in 2nd order model
  [#664] (https://github.com/paparazzi/paparazzi/pull/663)
- HITL using reference position to fake GPS
  [#640] (https://github.com/paparazzi/paparazzi/pull/640)
- add a HOME mode
  [#562] (https://github.com/paparazzi/paparazzi/pull/562)
- nav: split waypoint proximity check from time spend at waypoint
  [#690] (https://github.com/paparazzi/paparazzi/pull/690)

Fixedwing firmware
------------------

- alt_float: remove ALT_KALMAN_ENABLED
  [#594] (https://github.com/paparazzi/paparazzi/issues/594)
- extra navigation routines as modules
  [#512] (https://github.com/paparazzi/paparazzi/pull/512)
- NavSetAltitudeReferenceHere added


Paparazzi 5.0.5_stable
======================

Maintenance release

- fix field order in HFF_DBG message
- fix altitude in some extra nav routines
- fix create_module tool
- fix RCLost macro
- add GetAltRef() for flight plan compatibility with v5.2


Paparazzi 5.0.4_stable
======================

Maintenance release

- fix perl script compile_all_test_targets
- add pcre lib to jsbsim, cleanup shell commands
- fix InsideX for sectors (GetPosX and GetPosY in nav.h (FW) are in local coordinates)
  [#602] (https://github.com/paparazzi/paparazzi/issues/602)
- stm32: enable correct error interrupts for i2c2 and i2c3


Paparazzi 5.0.3_stable
======================

Maintenance release

- fix Paparazzi Center on Mac OS (detection of child processes exitting)
  [#290] (https://github.com/paparazzi/paparazzi/issues/290)
- state interface: fix stateCalcHorizontalSpeedNorm_i
- fix/improve dependency generation for building firmware
- abort with meaningful error if ARM toolchain is not found


Paparazzi 5.0.2_stable
======================

Maintenance release

- add launch and noground options to pprzsim-launch
- fixedwing: fix initialzation of trim commands, including yaw
- fixedwing: fixes to use ins_alt_dot from ins_alt_float (with USE_BAROMETER)
  [#511] (https://github.com/paparazzi/paparazzi/pull/511)
- state interface: fix local/global coordinate validity checks
- state interface: fix local ned/enu to ecef conversion
- lib/ocaml: update leap_seconds to 16 (last one was on June 30, 2012)


Paparazzi 5.0.1_stable
======================

Maintenance release

General
-------

- joystick hat support for input2ivy
  [#460] (https://github.com/paparazzi/paparazzi/pull/460)
- high speed logger: fix mag channels
- math: fix ecef_of_[ned|enu]_i
- fix google maps version download parsing

Rotorcraft/Fixedwing firmwares
------------------------------

- rotorcraft: add MODE_STARTUP
  [#467] (https://github.com/paparazzi/paparazzi/pull/467)
- ARDrone2: GPS satellite informaton
  [#474] (https://github.com/paparazzi/paparazzi/pull/474)
- fixedwing: init state interface before sensors and ins
- fixedwing: don't overwrite yaw command with trim

Simulator
---------

- NPS: explicitly add pcre lib
- NPS: make radio_control tpye datalink work
- don't try to use ADC_CHANNEL_CURRENT in SITL

Drivers and architecture specific
---------------------------------

- fixes for imu_drotek_10dof_v2
- init spi_slave_hs for imu_chimu
- mpu60x0_i2c: only copy ext data if i2c_bypass is false
- aspirin_v2.2: unselect baro at startup
- aspirin_2_spi: default AHRS_PROPAGATE_FREQUENCY is 512
- STM32F4: fix ppm input timer frequency for TIM2
- add i2c3 initialization


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


Paparazzi 4.2.2_stable
======================

Maintenance release

- DFU upload matches Lia board by default as well
- partial compatibilty with Aspirin2.2
  [#369] (https://github.com/paparazzi/paparazzi/pull/369)
- fix failsafe vertical setpoint in rotorcraft firmware
- fix plotprofile building on Ubuntu 13.04
- circle-count without rewinding when flying in opposite direction
  [#441] (https://github.com/paparazzi/paparazzi/pull/441)
- add yaw trim
  [#444] (https://github.com/paparazzi/paparazzi/pull/444)
- add XSens Mti-G 700 support
  [#443] (https://github.com/paparazzi/paparazzi/pull/443)


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


Paparazzi 4.0.4
===============

Maintenance release

- fix google map version parsing for new maps site using https
- minor fix for hff
- use GPS_TRIGGERED_FUNCTION for ins_chimu_spi


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
