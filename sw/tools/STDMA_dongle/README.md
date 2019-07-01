STDMA_dongle.md

### Description
This tool allows Bluegiga dongle to communicate using STDMA (Self-organized Time-Division Multiple Access).

Using this, drones with a Bluegiga USB Bluetooth dongle can communicate with eachother and exchange their RSSI (i.e., signal strenth, which is a measure of distance). This has been tested with up to 3 ARDrones.

It was used in: Coppola, M., McGuire, K.N., Scheper, K.Y.W. et al. Auton Robot (2018). https://doi.org/10.1007/s10514-018-9760-3

### Commands functions
make clean && make --> makes it for the computer
EMBED=1 --> this builds it for the drone, rather than for your computer
Once connected to the drone --> make upload_program EMBED=1 HOST=192..........

### Setting up the communication between laptops/drones
Turn on the drones and go to:

	cd $PAPARAZZI_HOME/sw/tools/STDMA_dongle

The command

	make clean && Make

will build `bluetooth_proximity`. Which is an application that can run on your computer. This is very useful for broadcasting from your computer if you want your computer to simulate the presence of a drone, for instance.
You can run it with

	./bluetooth_proximity

If you want the program to actually run on the ARDrones, then you have to build their embedded version and upload it. Note that this must be done every time the drone stops being active (e.g. when you change the battery). It starts running as soon as it is uploaded.

To build and run for the drone:
	make clean && make EMBED=1

To upload to the drones, select your **DRONE** type in the Makefile or add the option **DRONE=...** to the command (tested with ARDrone):
	make upload_program EMBED=1 HOST=192.168.40.200
Note the last numbers may change depending on the IP address of your drones.
Make sure to check that you are connected to the drones when doing this step!
