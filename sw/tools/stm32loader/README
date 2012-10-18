STM32Loader
===========

Python script which will talk to the STM32 bootloader to upload and download firmware.

Original Version by: Ivan A-R <ivan@tuxotronic.org>


Usage: ./stm32loader.py [-hqVewvr] [-l length] [-p port] [-b baud] [-a addr] [file.bin]
    -h          This help
    -q          Quiet
    -V          Verbose
    -e          Erase
    -w          Write
    -v          Verify
    -r          Read
    -l length   Length of read
    -p port     Serial port (default: /dev/tty.usbserial-ftCYPMYJ)
    -b baud     Baud speed (default: 115200)
    -a addr     Target address

    ./stm32loader.py -e -w -v example/main.bin


Example:
stm32loader.py -e -w -v somefile.bin

This will pre-erase flash, write somefile.bin to the flash on the device, and then perform a verification after writing is finished.

