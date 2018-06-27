# Description
These scripts can be used to connect Bebop 2 drones to a Wi-Fi router, so that (for instance) they may be used in a swarm via a central hub.

The drone will then function as follows:
	Upon starting up, it will search for the router. If the router is found, it will connect to it.
	If the router is not found within a specific time, the drone will become the access point, as "normal".
	The "4 button press" will also be removed, so this will not have to be done before loading Paparazzi on the drone.

# How to set it up
To connect the Bebop 2 to a router:
1. Open pprz_swarmhub.conf
2. Write the name of the router after WIFI_SSID
3. Start up the Bebop2 and connect to it normally with your computer's WiFi
4. Start up the router
5. Run connect2ssid.sh
6. The Bebop2 should now turn off and connect to the router. To check that this happened, you can connect to the router and try to ping to it/see all connections.
7. Add to the airframe file of the bebop, in the ap target, the following lines:
~~~~
<configure name="HOST" value="192.168.42.$(AC_ID)" />
<configure name="MODEM_HOST" value="192.168.42.40" />
~~~~
