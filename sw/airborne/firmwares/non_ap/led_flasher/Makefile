
# Quiet compilation
Q=@

MCU = attiny25

PROG = dragon_isp
DEV = usb
#PROG = stk200
#DEV = /dev/parport0

APPNAME = blitzer

CC      = avr-gcc
OBJCOPY = avr-objcopy
RM		= rm

CFLAGS  = -Wall -g -mmcu=$(MCU) -Os -fpack-struct -fshort-enums -Wall -Wstrict-prototypes

SOURCES =	blitzer.c

all: $(APPNAME)

clean:
	$(Q)$(RM) -f core $(APPNAME).o $(APPNAME) $(APPNAME).elf $(APPNAME).hex

app: $(APPNAME)

$(APPNAME):	$(SOURCES) Makefile
	$(CC) $(CFLAGS) $(SOURCES) -o $(APPNAME).elf
	$(OBJCOPY) -O ihex -R .eeprom $(APPNAME).elf $(APPNAME).hex

flash: $(APPNAME)
	avrdude -c $(PROG) -P $(DEV) -p $(MCU) -B 10 -U flash:w:$(APPNAME).hex

flash_fuses:
	avrdude -c $(PROG) -P $(DEV) -p $(MCU) -B 10 -U hfuse:w:0xdf:m
	sleep 2
	avrdude -c $(PROG) -P $(DEV) -p $(MCU) -B 10 -U lfuse:w:0x62:m

# (this is the default on new chips)
#
# Fuse high byte:
# 0xdf = 1 1 0 1   1 1 1 1 <-- BODLEVEL0 (Brown out trigger level bit 0)
#        ^ ^ ^ ^   ^ ^ ^------ BODLEVEL1 (Brown out trigger level bit 1)
#        | | | |   | +-------- BODLEVEL2 (Brown out trigger level bit 2)
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (WDT not always on)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ DWEN (debug wire is enabled)
#        +-------------------- RSTDISBL (reset pin is enabled)
# Fuse low byte:
# 0x62 = 0 1 1 0   0 0 1 0
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (internal calibrated osc)
#        | |  +--------------- SUT 1..0 (slowly rising power)
#        | +------------------ CKOUT (clock output disabled)
#        +-------------------- CKDIV8 (divide clock by eight enabled)

