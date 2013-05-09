# Quiet compilation
Q=@

#PROC_AP PROC_FBW PROC_TINY PROC_TINYJ
#PROC = TINYJ
ALLFLAGS = -DPROC_$(PROC)

LIBNAME	= usbstack
APPNAME = bl
APPNAME_RAM = $(APPNAME)_ram

# Package definitions
PKG_NAME	= target
DATE		= $$(date +%Y%m%d)
DATEL		= $$(date +%Y%m%d-%H%M)

LINKFILE	= lpc2148-rom.ld
LINKFILE_RAM	= lpc2148-ram.ld


#
# try to find the paparazzi multilib toolchain
#
include $(PAPARAZZI_SRC)/conf/Makefile.arm-embedded-toolchain

TAR = tar

CFLAGS  = -I./ -c -W -Wall -Os -g -mcpu=arm7tdmi -mthumb-interwork $(ALLFLAGS)
# -DDEBUG
ASFLAGS = $(ALLFLAGS)
LFLAGS  = -T $(LINKFILE) -nostartfiles -n
LFLAGS_RAM  = -T $(LINKFILE_RAM) -nostartfiles -n
CPFLAGS = -O ihex
ODFLAGS	= -x --syms

OBJS =	crt.o startup.o printf.o console.o bootloader.o

LIBOBJS =	usbhw_lpc.o usbcontrol.o usbstdreq.o usbinit.o usbdescrip.o

all: lib app app_ram

clean:
	$(Q)$(RM) -f $(LIBNAME).a $(LIBOBJS)
	$(Q)$(RM) -f $(APPNAME).hex $(APPNAME).elf $(OBJS) *.lst $(APPNAME).dmp $(APPNAME).map
	$(Q)$(RM) -f $(APPNAME_RAM).hex $(APPNAME_RAM).elf $(APPNAME_RAM).dmp $(APPNAME_RAM).map

# build lib
lib: $(LIBOBJS)
	$(RM) -f $(LIBNAME).a
	$(AR) -r $(LIBNAME).a $(LIBOBJS)

app: $(APPNAME).elf
	$(Q)echo "...copying"
	$(CP) $(CPFLAGS) $(APPNAME).elf $(APPNAME).hex
	$(DMP) $(ODFLAGS) $(APPNAME).elf > $(APPNAME).dmp
#	$(Q)echo "....loading"
#	./lpc21isp -control bl.hex com4 38400 12000
#	./lpc21isp -control -termonly bl.hex com4 115200 12000

app_ram: $(APPNAME_RAM).elf
	$(Q)echo "...copying"
	$(CP) $(CPFLAGS) $(APPNAME_RAM).elf $(APPNAME_RAM).hex
	$(DMP) $(ODFLAGS) $(APPNAME_RAM).elf > $(APPNAME_RAM).dmp

$(APPNAME).elf: $(OBJS) $(LIBNAME).a $(LINKFILE)
	$(Q)echo "..linking"
	$(CC) $(OBJS) $(LIBNAME).a $(LFLAGS) -o $(APPNAME).elf -Wl,-Map,$(APPNAME).map

$(APPNAME_RAM).elf: $(OBJS) $(LIBNAME).a $(LINKFILE_RAM)
	$(Q)echo "..linking RAM"
	$(CC) $(OBJS) $(LIBNAME).a $(LFLAGS_RAM) -o $(APPNAME_RAM).elf -Wl,-Map,$(APPNAME_RAM).map

crt.o: crt.S
	$(Q)echo ".assembling"
	$(CC) -c $(ASFLAGS) -Wa,-ahlms=crt.lst crt.S -o crt.o

# Builds archive tar file
arch: clean
	cd .. && $(TAR) --exclude={CVS,cvs} -cvzf $(PKG_NAME)-$(DATEL).tar.gz $(PKG_NAME)

# Builds release tar file
dist: clean
	cd .. && $(TAR) --exclude={CVS,cvs} -cvzf $(PKG_NAME)-$(DATE).tar.gz $(PKG_NAME)

# recompile if the Makefile changes
$(OBJS) $(LIBOBJS): Makefile

.PHONY: all clean lib app app_ram arch dist
