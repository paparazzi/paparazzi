
ARCH      = atmega128

# bootloader base in bytes
BL_BASE = 0x1800

bootloader.elf: bootloader.c
	avr-gcc -Wall -mmcu=$(ARCH) -Ttext $(BL_BASE) -o $@ $<


dump_flash:
        uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --download of=flash_dump.hex

clean:
	rm -f *~ *.o bootloader.elf