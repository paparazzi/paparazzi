.. user_guide flight_plan

============
Flight Plans
============

.. warning::

    This section work in progress, but an amazing doc is available at `http://wiki.paparazziuav.org/wiki/Flight_Plans <http://wiki.paparazziuav.org/wiki/Flight_Plans>`_. Go check it!

A **flight plan** is a XML document that describe how you want your AC to travel. It can easily express complex behavior.

It will be translated to C code at build time, and flashed in your AC. Modifying it then requires to build again the firmware, and upload it your AC.


.. admonition:: Expressions

    Some flight plan attributes are evaluated as C expressions.

    Since some operators are not compliant with the XML specifications (``&``, ``<``, ``>`` characters are not allowed), you **must** use some alternate naming: 

    .. csv-table::
        :header: "C operator", "substitute"
        
        "<", "@LT"
        ">", "@GT"
        "<=", "@LEQ"
        ">=", "@GEQ"
        "&&", "@AND"
        "||", "@OR"
        "->", "@DEREF"


DTD and Structure
-----------------

The flight plans are stored in the ``conf/flight_plans`` directory. The flight plan editor can be used to create basic flight plans via the GUI.

The formal description of the flight plan file is given in the DTD (located in ``conf/flight_plans/flight_plan.dtd``).
This DTD must be referenced in the header of your flight plan XML document with its relative path using the following line:


.. code-block::
    
    <!DOCTYPE flight_plan SYSTEM "flight_plan.dtd">


Extract from the DTD:

.. code-block::

    <!ELEMENT flight_plan (header?,waypoints,sectors?,variables?,modules?,includes?,exceptions?,blocks)>

A flight plan is composed of two mandatory elements: waypoints and blocks.

.. warning :: The order of the elements must be respected, or the parsing will trigger an error.

The root ``flight_plan`` element is specified with several attributes:

**Mandatory attributes**:

name
    The name of the mission (a text string)
lat0, lon0
    Defines the latitude and longitude coordinates of the reference point {0,0} in WGS84 degree coordinates
    
max_dist_from_home
    A radius representing the maximum allowed distance (in meters) from the HOME waypoint.
    Exceeding this value (ie flying outside the circle with this radius) will trigger an exception.
    It is up to you to define the block to be executed (ie what to do) for the exception.
    
ground_alt
    The ground altitude (in meters), Above Sea Level where you are flying.
    It defines the GROUND_ALT constant value which can be used in combination with a waypoint <height> parameter to define a waypoint height.
    
security_height
    The height (over ground_alt) used by the circle-home failsafe procedure and in other flight procedures such as formation flight and anti-collision avoidance.
    Warnings are produced if you place a waypoint **lower** than security_height (usually the case for the landing point).
    
alt
    The default altitude of waypoints (Above Sea Level).
    So if your ground altitude is 400 then alt needs to be a value greater than ground altitude and above any obstructions in the flight plan.

**Optional attributes**:

wp_frame
    Frame in wich waypoints X and Y coordinates are expressed.
    Default to ``UTM``, but can be set to ``LTP`` (`local tangent plane <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates>`_), which means that X and Y will align to east and north at the HOME point.

qfu
    Defines the global constant QFU. It usually is the magnetic heading in degrees (north=0, east=90) of the runway, the opposite of wind direction. This constant may be used in the mission description. It is also used by the simulator as the original course of the aircraft. So if you want to take off and climb to the West you would use qfu=270.

home_mode_height
    Allows to override *security_height* as failsafe height in home mode. If home_mode_height Is set lower than security_height, the later is used.
    This attribute is useful if you need to return home at a high altitude rather than a low altitude.

.. error:: TODO

geofence_max_alt
    TODO

geofence_max_height
    TODO

geofence_sector
    TODO

.. admonition:: example

    Here is an example of such a line in the top of a flight plan:

    .. code-block::

        <flight_plan alt="250" ground_alt="185" lat0="43.46223" lon0="1.27289" name="Example Muret" max_dist_from_home="300" qfu="270" security_height="25" >

    Note that a flight plan could also contain optional include's and exceptions cases.

    In English the above flight plan says the name is *Example Muret*.

    The reference coordinates for the 0,0 point is: 43.46223 (lat) and 1.27289 (long).

    The flying site 0,0 location is 185m above sea level. The security height is 25m above 0,0 point or 210m above sea level.

    The default (ie if not defined in a waypoint this alt is used) altitude is 250m (above sea level).

    The home mode block altitude is defined to be 150m above sea level.
    Also, for security, a circle is defined with a radius that's 300m from 0,0 position. This is the max_dist_from_home value.
    Fly 301m from 0,0 and an exception is triggered.
    A useful block is to trigger/go to the home mode block and return to home when the aircraft flies outside the safety circle.

    Example flight plans are helpful for study before you build your own from scratch. 

Waypoints
---------

The waypoints are the geographic locations used as anchor to specify the trajectories. A waypoint is specified by it's name and coordinates.

You can't add or delete waypoints during the flight, but you can move them at any time.

.. note::
    
    There is a hard limit to max 255 waypoints, but it is usually a bad idea to use as many waypoints.
    If you have such a need, you may be interested in the **Mission** (TODO) mode.

Waypoint position is defined either in relative coordinates with the **x** and **y** attributes, or in absolute coordinates with the **lat** and **lon** attributes.

**alt** is an optionnal parameter that can be used to assign an altitude to a particular waypoint that is different from the globally defined *alt* parameter of the flightplan.

The **height** attribute can be used to set the waypoint height relative to the ground altitude (*ground_alt*) of the flight plan.

.. code-block::

    <waypoints>
        <waypoint name="HOME" x="0.0" y="30.0"/>
        <waypoint name="BRIDGEOVERRIVER" x="-100.0" y="60.0" alt="270."/>
        <waypoint name="MyBarn" x="-130.0" y="217.5" alt="3000."/>
        <waypoint name="_MYHELPERSPOT" x="-30.0" y="50" height="50."/>
        <waypoint name="4" x="-30.0" y="50." alt="ground_alt + 50"/>
        <waypoint name="" x="-30.0" y="60" height="50."/>
        <waypoint name="_MYOTHERHELPERSPOT" x="-70.0" y="90" height="70."/>
        <waypoint name="TOWER" lat="48.858249" lon="2.294494" height="324."/>
    </waypoints>

**Tips**

+ Waypoints are easily adjusted with the flight plan editor.
+ If a waypoint name starts with an underscore ( _ ), the waypoint is not displayed in the GCS, except in editor mode.
+ The maximum number of waypoints is 254.
+ A waypoint named HOME is required if the failsafe HOME mode procedure is used.
+ A waypoints index/reference pointer is derived by prefixing the waypoint name with ``WP_``. Useful when a call function uses the waypoints reference index vs. it's name.


Sectors
-------

Flat Sectors can be described as an area defined by a list of waypoint corners.
Such an area will be displayed in the Ground Control Station (GCS) by colored lines connecting the cornerpoints.
A function is generated to check if a point, usually the aircraft itself, is inside this sector.
For a sector named *MyBigGarden* the generated function for the example here would be ``bool_t InsideMyBigGarden(float x, float y);``
where x and y are east and north coordinated, in meters, relative to the geographic reference of the flight plan.
Note that sector names are not allowed to contain spaces.

.. note:: The edges of the polygon should not cross each other. 

For example, with the following element in a flight plan:

.. code-block::

    <sectors>
        <sector name="MyBigGarden" color="red">
            <corner name="_1"/>
            <corner name="_2"/>
            <corner name="_3"/>
            <corner name="_4"/>
        </sector>
    </sectors>


It is then possible to add an exception clause to your flightplan.

For example if the aircraft for some reason flies outside this sector, the airframe will fly to a standby waypoint.
The exclamation mark (!) means the boolean operator NOT in this example.
In regular language one would describe "*If my airframe is NOT inside the MyBigGarden sector anymore then deroute it to the standby waypoint*".
In Flightplan "Speak" this is written like:

.. code-block::

    <exception cond="! InsideMyBigGarden(GetPosX(), GetPosY())" deroute="standby"/>


**Tip:** The *color* attribute is optionnal. If not defined, the color will default to the AC color.


Variables
---------

It is possible to declare a list of variables that will be automatically created during the flight plan generation
and available for the rest of the system from the generated flight plan header and of course inside the flight plan itself.
With appropriate attributes, it is also possible to make the variables accessible from the telemetry as a setting.

The following code will produce a float variable initialized to 0:

.. code-block::

    <variables>
        <variable var="my_var"/>
    </variables>

The type and the initial value can be changed with the type and init attributes:

.. code-block::

    <variables>
        <variable var="my_var" init="10" type="int"/>
    </variables>

To produce an automatic setting for a variable, at least min, max and step attributes need to be specified:

.. code-block::

    <variables>
        <variable var="my_var" min="0." max="10." step="0.1"/>
    </variables>

They will appear under the Flight Plan settings tab in the GCS.
More attributes can be specified: **shortname**, **unit**, **alt_unit**, **alt_unit_coef**, **values**.
See `Settings <https://wiki.paparazziuav.org/wiki/Settings>`_ page for more information about these options. 


Modules
-------

Additional modules can be added to the airframe using the modules element inside the flight plan. The same syntax is used as in the airframe file:

.. code-block::

    <modules>
    <module name="demo_module">
      <define name="MY_DEFINE" value="0"/>
      <configure name="MY_CONF" value="0"/>
      ...
    </module>
    </modules>

.. note:: If a module is only used in the flight plan, it is obviously better to add it here, and not in the aiframe file.


Exceptions
----------

Exceptions consist in conditions checked periodically (at the same pace as the navigation control), allowing the control to jump to a given block. Here is the syntax of exceptions:

.. code-block::

    <exception cond="..." deroute="...">

where cond is an expression and deroute is the name of the block we want to switch to as soon as the condition is true. 

Here are some example of exceptions: 

.. code-block::

    <exception cond="PowerVoltage() @LT 10" deroute="go_down"/>
    <exception cond="(GetAltRef()+10 @GT GetPosAlt())" deroute="go_up"/>
    <exception cond="(autopilot_flight_time @GT 840)" deroute="quick_land"/>
    
Exceptions can be local to a block or global to the flight plan, in the ``<exceptions>`` element. 


Procedures
----------

Procedures are libraries which can be included in flight plans. They are composed of waypoints, sectors and blocks.
The header of a procedure may contain some parameters which are replaced by arguments when the procedure is included. 

A parameter is just a name. A parameter is optional if it is declared with a default value. An example with a required and an optional parameter: 

.. code-block::

    <param name="alt"/>
    <param name="radius" default_value="75"/>
    
Procedures are called with the include element in a flight plan. A procedure call requires: 

+ the name of the procedure file, the name given to this inclusion
+ values for the parameters
+ backlinks for block name exits of the procedure

.. error:: TODO
    

Blocks
------


.. error:: TODO


















