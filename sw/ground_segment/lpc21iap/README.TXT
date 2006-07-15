simple answer
=============

[preliminary]

link your airborne programs using the bootloader-specific linker script
make AIRCRAFT=TJ1 LDSCRIPT=arm7/LPC2148-ROM-bl.ld clean_ac fbw.compile
make AIRCRAFT=TJ1 LDSCRIPT=arm7/LPC2148-ROM-bl.ld ap.compile


apt-get install libusb
( warning on sid I had to do apt-get install libusb-0.1-4 )
apt-get install libusb-dev
(maybe this also needs a version?)

cp $PAPARAZZI_SRC/conf/system/udev/rules/10-paparazzi.rules /etc/udev/rules.d/
cd $PAPARAZZI_SRC/sw/airborne/arm7/test/bootloader/
make
lpc21isp -control bl.hex /dev/ttyS0 38400 12000
cd $PAPARAZZI_SRC/sw/ground_segment/lpc21iap/
make
./lpc21iap $PAPARAZZI_SRC/var/(your aircraft)/ap/ap.elf
./lpc21iap $PAPARAZZI_SRC/var/(your aircraft)/fbw/fbw.elf

lpc21iap
========

This is a download application for Philips LPC processors. So far only the LPC214x series is supported (as only they have USB function). The built-in bootloader serial download for the LPC processors is not the fastest. An USB download might be much faster and convenient.

An entire LPC2148 is re-flashed (erase, download, flash and verify 500kB) in a couple of seconds. Multiple LPC processors can be used and flashed with specific flash images by having different serial numbers.

Bertrik Sikken provided a neat and easy-to-use USB library that is the base for the embedded software. The lpc21isp software from Martin Maurer inspired the PC software. Use it to download the bootloader. The libusb package is the API to have an easy access to the USB with Linux, Cygwin and Windows.

Get libusb from sourceforge, install it. You might have to fiddle with hotplug or udev when using Linux for appropriate USB device access rights. Make lpc21iap and the bootloader software for the target. Flash the bootloader bl.hex with lpc21isp.

The first four 4096 bytes sectors are reserved for the bootloader. Your application should be linked to 0x4000 (don't forget the '-n' option when linking!). The bootloader interrupt vectors are re-directed to 0x4000 execept the reset and the irq vector. After a reset the bootloader will check the Vbus (P0.23) pin (or any other, change it easily in crt.s) if it is set. If yes, the bootloader is started. Otherwise it will jump to 0x4000. The irq vector will read the reset vector from the VIC register. If you want to use USB in your application, change either the bootpin in crt.s or set it to one manually by writing '1' to GPIO P0.23.

If you want to flash the first four sectors with lpc21iap you have to use the RAM version bl_ram.elf.

The used USB vendor/device ID is made up (0x7070/0x1234). Someone willing to donate one?

Protocol
========

The Philips LPC21 ISP and IAP commands are used to communicate with the device. They are transparently transferred over USB. Either of the protocols can be used (each command set includes commands the other one does not support). Some proprietary bootloader BTL commands are added as well.

Every command consists of up to three phases. In the first phase the command plus parameters are given to the device and immediately executed. The host might receive a result in the second phase (not mandatory). The third phase is used to transfer data to/from the device. Some commands consist only of one phase (eg. GO), some need all three phases (eg. WRITE_TO_RAM).

The IAP and ISP commands are mapped to (endpoint 0) vendor requests. The switching between command and result is done by the direction bit in bmRequestType (Host-to-device = command). The bRequest is defined as:

    REQ_ISP_COMMAND     0xF0
    REQ_IAP_COMMAND     0xF1
    REQ_BTL_COMMAND     0xF2
    REQ_DATA_TRANSFER   0xF3

The wValue and wIndex of the request are reserved, write them as zero. The wLength defines the number of bytes to be transferred to/from device in this request. Command, result and data is transferred during the data phase of the vendor request. wLength is 20 bytes (5 words) in the command phase, 32 bytes (3 + 5 words) in result phase and can be any size in data phase. Little endian format is used for any data.

The command consists of 5 words as defined by Philips IAP/ISP definition. Word 0 is the command (eg. ASCII value 'G' for ISP_GO in ISP or value 56 in IAP for IAP_COMPARE. Word 1-4 give additional parameters.

The result size is 8 words. Word 0 gives the result and word 1-2 contains additional response information. Word 3-7 contain the original command that the result came from.

Some commands require additional data to be transferred. This data is send with one or more DATA_TRANSFER requests. The total length of data is defined in the command (eg. READ_MEMORY) phase. Data transfer sizes bigger than 4kBytes have to be split in several data transfer requests.

Example:

direction   host->device
request     REQ_ISP_COMMAND
command[0]  ISP_WRITE_TO_RAM
command[1]  (startaddress)
command[2]  (size)
command[3]  (undefined)
command[4]  (undefined)

direction   host<-device
request     REQ_ISP_COMMAND
result[0]   CMD_SUCCESS
result[1]   (undefined)
result[2]   (undefined)
result[3]   ISP_WRITE_TO_RAM
result[4]   (startaddress)
result[5]   (size)
result[6]   (undefined)
result[7]   (undefined)

direction   host->device
request     REQ_DATA_TRANSFER
data[0]     (data)
  ...
data[n]     (data)

direction   host->device
request     REQ_DATA_TRANSFER
data[n+1]   (data)
  ...
data[size]  (data)
