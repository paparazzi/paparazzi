# Iridium Link
The Iridium Link tool configures the ground modem and initiates a call to the air modem. The ground modem firmware acts as an forwarder to the IR9523 OEM modem. This way the ground can be configured to call every number and forward all data over UDP.

![Iridium Link Program](https://raw.githubusercontent.com/tudelft/iridium/master/tools/iridium_link.png)

## How to make a call
- Connect your ground modem with a serial cable to your computer (make sure it has the correct firmware)
- Start the `iridium_link.py` and enter the correct Serial port
- Press 'Open', this will setup the modem with the correct settings
- Press 'Register' to register the iridium device in the network
- Enter the correct phone number in the dialog
- Press 'Call' to initiate a call (this can take some time)

## Prerequisites
- pygtk 2.24.2
- pyserial 3.1.1
- Twisted 16.4.1

## FAQ

### I get a 'NO CARRIER' back
Please make sure you see the 'Registered' box checked, if not make sure you pressed 'Register'. Make sure the phone number is also correct. If both are correct try to press 'CSQ' which will return a signal quality, if this is '0' make sure you have placed your antenna aming at a clear sky.

### Where do I get the data
This tool will output data by default on UDP port '4242' and data can be send to UDP port '4243' on the local host. This will work by default with the Paparazzi Autopilot UDP link.

### I changed the SIM pin number
Make sure you also update the PIN number into the tool (this will be changeable in later versions). If not it won't be able to register.

### I cannot register to the network
If the 'Registered' box is never checked make sure your SIM card is correctly placed in the ground modem. Also make sure that your SIM card is activated by your provider.

### The ground modem doesn't respond
Make sure the ground modem is connected and turned on. Also make sure that the ground modem has the correct firmware (The ground and air modem have different versions).
