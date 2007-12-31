i2c-tiny-usb test application - http://www.harbaum.org/till/i2c_tiny_usb
------------------------------------------------------------------------

Adapted to Melexis 90614 IR sensors.

This simple test application is meant to demonstrate libusb
interfacing to the i2c-tiny-usb interface.

This is no useful application, if you are only interesting in 
using the i2c-tiny-usb interface in your linux box please
use the kernel driver. 

Linux
-----

This demo application has been developed under and for linux. Just
make sure you have libusb installed. To use this program just
compile by typing "make" and run the resulting i2c_usb.

Be sure that the i2c-tiny-usb kernel driver is not loaded while
running the test application. Otherwise the test application will
fail with the follwing error message:

USB error: could not claim interface 0: Device or resource busy

This is due to the fact that no two drivers may access the interface
at the same time.

Windows
-------

This program can be compiled for windows. This has been tested 
under Linux using xmingw and the windows port of libusb 
(see http://libusb-win32.sourceforge.net). To install the 
driver plug the device in and install the driver from
the win directory. Then run testapp/i2c_usb.exe

This program may also be compiled under windows using cygwin or
mingw (which is part of cygwin). In order to use cygwin simply 
copy usb.h win32-linusb to /cygwin/usr/include and libusb.a to
/cygwin/lib and do a "make -f Makefile.cygwin". Don't forget to 
distribute /cygwin/bin/cygwin1.dll with your file to allow it to 
run in non-cygwin environments as well. No dll is required when using 
mingw. In that case copy usb.h to /cygwin/usr/include/mingw and 
libusb.a to /cygwin/lib/mingw. Finally do a "make -f Makefile.mingw".

MacOS X
-------

The program can be compiled under MacOS as well. The fink version
of linusb has to be installed and a simple "make -f Makefile.macos"
will build the native MacOS X version.
