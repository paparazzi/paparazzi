# Airframe

The airframe file is the most important configuration file and contains all the hardware and software settings for your airframe. It describes what hardware you have and which firmware, sensors, algorithms, etc. you want to use and also holds your configuration parameters. All gains, trims, and behavior settings are defined with standard XML elements.

The XML airframe configuration file is located in `conf/airframes/**/`, and it looks like this (this is *not* a complete example):

<details>

<summary>Airframe example</summary>

``` XML
<!DOCTYPE airframe SYSTEM "../airframe.dtd">

<airframe name="myAmazingAirframe">

<description>
    This is my airframe
</description>

<firmware name="rotorcraft">
    <configure name="PERIODIC_FREQUENCY" value="500"/>
    <configure name="AHRS_PROPAGATE_FREQUENCY" value="$(PERIODIC_FREQUENCY)"/>

    <target name="ap" board="tawaki_2.0">
      <module name="radio_control" type="sbus"/>
      <configure name="BARO_PERIODIC_FREQUENCY" value="50"/>
      <define name="RADIO_KILL_SWITCH" value="RADIO_GAIN1"/>
    </target>
    <target name="nps" board="pc">
      <module name="fdm" type="jsbsim"/>
      <module name="radio_control" type="ppm"/>
    </target>

    <module name="gps" type="ublox">
    <configure name="GPS_BAUD" value="B115200"/>
    </module>

    <module name="telemetry" type="xbee_api"/>
    <module name="ins" type="ekf2"/>
    <module name="actuators" type="dshot"/>
    <module name="stabilization" type="indi"/>
    <module name="guidance" type="indi"/>
    <module name="motor_mixing"/>
</firmware>

<servos driver="DShot">
    <servo name="FR" no="3" min="0" neutral="100" max="2000"/>
    <servo name="BR" no="4" min="0" neutral="100" max="2000"/>
    <servo name="BL" no="2" min="0" neutral="100" max="2000"/>
    <servo name="FL" no="1" min="0" neutral="100" max="2000"/>
</servos>

<commands>
    <axis name="ROLL" failsafe_value="0"/>
    <axis name="PITCH" failsafe_value="0"/>
    <axis name="YAW" failsafe_value="0"/>
    <axis name="THRUST" failsafe_value="0"/>
</commands>

<command_laws>
    <set servo="FR" value="autopilot_get_motors_on() ? actuators_pprz[0] : -MAX_PPRZ"/>
    <set servo="BR" value="autopilot_get_motors_on() ? actuators_pprz[1] : -MAX_PPRZ"/>
    <set servo="BL" value="autopilot_get_motors_on() ? actuators_pprz[2] : -MAX_PPRZ"/>
    <set servo="FL" value="autopilot_get_motors_on() ? actuators_pprz[3] : -MAX_PPRZ"/>
</command_laws>

<section name="GUIDANCE_H" prefix="GUIDANCE_H_">
    <define name="MAX_BANK" value="30" unit="deg"/>
    <define name="PGAIN" value="41"/>
    <define name="DGAIN" value="108"/>
    <define name="IGAIN" value="20"/>
    <define name="NGAIN" value="0"/>
    <!-- feedforward -->
    <define name="AGAIN" value="0"/>
    <define name="REF_MAX_SPEED" value="0.5"/>
    <define name="REF_MAX_ACCEL" value="2."/>
</section>

<section name="MIXING" prefix="MOTOR_MIXING_">
    <define name="TYPE" value="QUAD_X"/>
    <define name="REVERSE" value="TRUE"/>
</section>

</airframe>
```

</details>

-------------------

Lets break it up in digestable chunks.


```XML
<!DOCTYPE airframe SYSTEM "../airframe.dtd">
```

This first line is always a doctype declaration. It specifies the relative path to `airframe.dtd`, which is located in the `conf/airframe` directory. In the above example, our airframe file  is located in a subdirectory of `conf/airframe`, which explains the `../`.

The root element `airframe` has the `name` attribute. You can add a description of your airframe is the `description` element.

```XML
<airframe name="myAmazingAirframe">

  <description> This is my airframe </description>

  <firmware name="rotorcraft">
    <target name="ap" board="tawaki_2.0">
      <module name="radio_control" type="sbus"/>
    </target>

    <module name="gps" type="ublox">
      <configure name="GPS_BAUD" value="B115200"/>
    </module>

    <module name="telemetry" type="xbee_api"/>

  </firmware>

  ...

</airframe>
```

## Firmware

The next element is the `firmware` node. Its `name` attribute can take two values: `fixedwing` or `rotorcraft`.

### Target

The `target` tag define what autopilot hardware you will be using.

The `name` attribute can be `sim` or `nps` for simulation, or `ap` for real hardware.

When using `ap`, the `board` attribute specifies which board you are using, e.g. `tawaki_2.0`, `apogee_1.0_chibios`, `cube_orangeplus`, ...  
All boards can be found in `/conf/boards`.

You can have multiple targets in an airframe file. Its common to have a simulation target along with the `ap` target.

### Modules

**Modules** are the building block of paparazzi. They include code and configuration for that code.

Include modules in you firmware with the `module` element. Modules can be restricted to a particular target by including it as a child element of this target.

The `module` element has a mandatory attribute `name` and an optionnal attribute `type`. <br/>
The included module is the file `<name>[_<type>].xml` from the `conf/modules` directory.

:::{admonition} Example
The following element will add the `actuators_dshot` module:
```XML
<module name="actuators" type="dshot"/>
```
:::




## Defines, configures and sections

Firmware and modules are configured in two ways: defines and configures.
The documentation associated with each module list the available configurations.

### Configures

Configures can only be used in the *firmware* element, either as direct child of the *firmware* node or as child of a *target* or *module*.

Configures have this syntax:

```XML
<configure name="MODEM_PORT" value="UART2">
```

There is generaly a default value. If the default value suits you, its not needed to redefine it.

:::{note} Configures create a makefile variable that can later be used in the build process, e.g. `$(MODEM_PORT)`
:::


### Defines

Defines have a similar syntax. A unit can optionaly be specified to automatically convert from this unit to the standard unit:

```XML
<define name="MAX_ROLL" value="50" unit="deg"/>
```

Some defines do not need a value:

```XML
<define name="DEBUG_ALT_KALMAN">
```

Defines can be used as child of the *firmware* node as well as in a *section* node. In a *section* node, the section prefix in prefixed to the define name. 

:::{admonition} Example
  ```XML
  <section name="MIXING" prefix="MOTOR_MIXING_">
      <define name="TYPE" value="QUAD_X"/>
      <define name="REVERSE" value="TRUE"/>
  </section>
  ```

  This part of the airframe will produce this code in `airframe.h`:

  ```C
  #define MOTOR_MIXING_TYPE       QUAD_X
  #define MOTOR_MIXING_REVERSE    TRUE
  ```
:::

:::{note}
  - A define in the *firmware* node will produce a compilation flag for GCC, e.g. `-DUSE_INS_NAV_INIT=TRUE`.
  - A define in a *section* node will produce `#define` directives in the generated `airframe.h` file.
:::

:::{warning} Although syntax is similar, defines and configures are not interchangeable
:::

### Sections

A section is a group of defines with an optional prefix. See above.

(servo_section)=
## Servos

Actuators are configured in the *servos* element. The *driver* attribute is related to the loaded actuator module.

For each *servo* is configured:
- **name**: name by which it will be referenced in later part of the airframe
- **no**: servo number. Usually related to the autopilot board
- **min**, **neutral**, **max**: values in the driver's unit.

```XML
<servos driver="DShot">
    <servo name="FR" no="3" min="0" neutral="100" max="2000"/>
    <servo name="BR" no="4" min="0" neutral="100" max="2000"/>
    <servo name="BL" no="2" min="0" neutral="100" max="2000"/>
    <servo name="FL" no="1" min="0" neutral="100" max="2000"/>
</servos>
```

If you have multiple actuators modules, there should be a *servos* node for each of them


:::{admonition} Example

  In this example, two actuator modules are loaded: **dshot** for the 4 motors, et **pwm** for a servomotor.

  ```XML
  <firmware name="rotorcraft">
    ...
    <module name="actuators" type="dshot"/>
    <module name="actuators" type="pwm"/>
    ...
  </firmware>
  
  <servos driver="DShot">
    <servo name="FR" no="3" min="0" neutral="100" max="2000"/>
    <servo name="BR" no="4" min="0" neutral="100" max="2000"/>
    <servo name="BL" no="2" min="0" neutral="100" max="2000"/>
    <servo name="FL" no="1" min="0" neutral="100" max="2000"/>
  </servos>

  <servos driver="Pwm">
    <servo name="SWITCH" no="1" min="1000" neutral="1500" max="2000"/>
  </servos>
  
  ```

:::


## Commands

The commands lists the abstract commands you need to control the aircraft.

Each command is also associated with a failsafe value which will be used if no controller is active, for example during initialization of the autopilot board. The range of these values is [-9600, 9600]. For *THROTTLE*, the range is [0, 9600] and in the corresponding servo definition the neutral and min are usually the same for PWM based servos (see below).


``` XML
  <commands>
    <axis name="ROLL" failsafe_value="0"/>
    <axis name="PITCH" failsafe_value="0"/>
    <axis name="YAW" failsafe_value="0"/>
    <axis name="THRUST" failsafe_value="0"/>
  </commands>
```



## Command laws

The command laws section acts as the interface between the commands, which are calculated by the stabilisation module, and the servos, which are the physical actuators. The operation of the line 
```XML
<set servo="<servo_name>" value="<value>"/>
```
can be understood as we apply the **value** in the servo name **servo_name**.
From this, we can deduce that the servo definition (see section {ref}`servo_section`) must contain a corresponding **name**.

Regarding the values attributed to actuators, when employing the INDI method or a control law utilising **actuators_pprz**, the following line should be used:
```XML
autopilot_get_motors_on() ? actuators_pprz[0] : -MAX_PPRZ
```
This allows the value calculated by the control law to be assigned only when the motors are started, otherwise the motors are stopped (-MAX_PPRZ). In the case of servomotors, it may be necessary to assign the value 0, which corresponds to a neutral value.

**actuators_pprz** is an array in which the control law assigns the values calculated at each iteration.

This gives us a similar block for a quadcopter: 
``` XML
  <command_laws>
    <set servo="FR" value="autopilot_get_motors_on() ? actuators_pprz[0] : -MAX_PPRZ"/>
    <set servo="BR" value="autopilot_get_motors_on() ? actuators_pprz[1] : -MAX_PPRZ"/>
    <set servo="BL" value="autopilot_get_motors_on() ? actuators_pprz[2] : -MAX_PPRZ"/>
    <set servo="FL" value="autopilot_get_motors_on() ? actuators_pprz[3] : -MAX_PPRZ"/>
  </command_laws>
```

Or for a fixedwing:
``` XML
  <command_laws>
    <let var="aileron"            value="@ROLL  * 0.3"/>
    <let var="elevator"           value="@PITCH * 0.7"/>  
    <set servo="THROTTLE"         value="@THROTTLE"/>
    <set servo="ELEVON_LEFTSIDE"  value="$elevator + $aileron"/>
    <set servo="ELEVON_RIGHTSIDE" value="$elevator - $aileron"/>
  </command_laws>
```

where transitory variables (**aileron** and **elevator**) are created to assign a control combination to the servo.
