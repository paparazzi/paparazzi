.. tutorials main_tutorials interfacing_with_paparazzi

==========================
Interfacing with Paparazzi
==========================

There are four ways to interact with and to extend paparazzi capabilities:

- Using Pprzlink with the current ground station tools
- Using Pprzlink independently of the ground station tools
- Adding a dedicated board on the drone and using Pprzlink to communicate directly with the autopilot (usually with the serial transport)
- Adding a module to the autopilot itself

A combination of these options can be used at the same time to achieve specific goals.


Using Pprzlink with the ground station
--------------------------------------

The easier way to interact with Paparazzi is to create a new agent as part of the ground station. All parameters and configuration files are easily accessible, making it easy to interact with the drone or display new data on the GCS. As example, the Interactive Informatics team of ENAC used it to design novel human-drone interactions for safety pilots and adaptable interactions for pilots with disabilities.

The pprzlink messages that will interest us are those of the `ground` class. Most of those messages are emited by the server that compile the data from all drones and add an abstraction layer.

See the message list here: http://docs.paparazziuav.org/latest/paparazzi_messages.html#GROUND_CLASS.

This python example shows the reception and emission of messages. For the reception, we subscribe to the FLIGHT_PARAM and ENGINE_STATUS messages at lines 25 and 27 to monitor the position of the drone, its throttle level and battery voltage. For the emission, we forge, then emit an INTRUDER message in the `send_dummy_intruder` method to display a dummy intruder on the Ground Control Station.

.. code-block:: python
    :linenos:

    #!/usr/bin/python3
    import sys
    import time
    import os
    import math
    
    # Make sure to add PAPARAZZI_HOME and PAPARAZZI_SRC to your environment variables.
    # They must point to your paparazzi directory.
    PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
    if PPRZ_HOME is None:
        raise Exception("PAPARAZZI_HOME environment variable is not set !")
    sys.path.append(os.path.join(PPRZ_HOME, "var/lib/python"))
    
    from pprzlink.ivy import IvyMessagesInterface
    from pprzlink.message import PprzMessage
    
    IVY_APP_NAME = "Basic Interfacing Test"  # This is the name of your app on the Ivy bus
    DEFAULT_BUS = "127.255.255.255:2010"     # This is your network mask, followed by the port
    
    class BasicInterfacing:
        def __init__(self, ivy_bus=DEFAULT_BUS):
            # start Ivy interface
            self.ivy = IvyMessagesInterface(agent_name=IVY_APP_NAME, ivy_bus=ivy_bus)
            # subscribe to the "FLIGHT_PARAM" message of class "ground"
            self.ivy.subscribe(self.flight_param_cb, PprzMessage('ground', 'FLIGHT_PARAM'))
            # subscribe to the "ENGINE_STATUS" message of class "ground"
            self.ivy.subscribe(self.engine_status_cb, PprzMessage('ground', 'ENGINE_STATUS'))

        def flight_param_cb(self, sender, msg):
            print("AC {} at pos {}, {} and alt {}".format(msg.ac_id, msg.lat, msg.long, msg.alt))

        def engine_status_cb(self, sender, msg):
            print("AC {} at {}% throttle, {}V".format(msg.ac_id, msg.throttle, msg.bat))
        
        def send_dummy_intruder(self):
            msg = PprzMessage('ground', 'INTRUDER')
            msg['id'] = 1
            msg['name'] = "DUMMY"
            msg['lat'] = int((43.463 + 0.001*math.sin(0.2*time.time()))*1e7)
            msg['lon'] = int((1.273 +  0.001*math.cos(0.2*time.time()))*1e7)
            msg['alt'] = 500
            msg['course'] = 360 - math.degrees(0.2*time.time())%360
            msg['speed'] = 10
            msg['climb'] = 0
            msg['itow'] = 0
            self.ivy.send(msg)


    if __name__ == '__main__':
        bi = BasicInterfacing()
        try:
            while(True):
                time.sleep(0.5)
                bi.send_dummy_intruder()
                
        finally:
            bi.ivy.shutdown()

Using Pprzlink independently of the ground station tools
--------------------------------------------------------

Do exactly the same thing as above, but listen to messages from the `Telemetry` class.


