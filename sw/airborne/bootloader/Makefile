
ARCH      = atmega128

# mega128 fuses
#
# normal
# low_fuse  e0
# high fuse 99
# ext_fuse  ff
# lock fuse ff
#
# internal 1MHz 
#
# low_fuse e1
#

# bootloader base in bytes
BL_BASE = 0x1800

bootloader.elf: bootloader.c
	avr-gcc -Wall -mmcu=$(ARCH) -Ttext $(BL_BASE) -o $@ $<

bootloader.hex: bootloader.elf
	avr-objcopy -O ihex $< $@

upload_bl: bootloader.hex erase
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --upload if=$<

test_prog.elf: test_prog.c
	avr-gcc -Wall -mmcu=$(ARCH) -o $@ $<

test_prog.hex: test_prog.elf
	avr-objcopy -O ihex $< $@

upload_test: test_prog.hex erase
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --upload if=$<

rd_fuses:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --rd_fuses

reset_on_bl:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --wr_fuse_h=0x98

reset_on_app:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --wr_fuse_h=0x99

internal_1M:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --wr_fuse_l=0xe1

external_16M:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --wr_fuse_l=0xe0

upload_by_bl: test_prog.hex
	uisp -dserial=/dev/ttyS0 -dspeed=38400 -dprog=avr910 -dpart=auto -v=4 --upload if=$<

dump_flash:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --download of=flash_dump.hex
erase:
	uisp -dlpt=/dev/parport0 -dprog=stk200 -v=3 --erase

clean:
	rm -f *~ *.o bootloader.hex bootloader.elf test_prog.hex test_prog.elf