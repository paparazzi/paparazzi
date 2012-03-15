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
TOOLCHAIN=$(shell find -L /opt/paparazzi/arm-multilib ~/sat -maxdepth 1 -type d -name arm-none-eabi 2>/dev/null | head -n 1)

#
# found the new paparazzi toolchain, use it
#
ifneq ($(TOOLCHAIN),)
TOOLCHAIN_DIR=$(shell dirname $(TOOLCHAIN))
GCC_BIN_DIR=$(TOOLCHAIN_DIR)/bin
GCC_LIB_DIR=$(TOOLCHAIN_DIR)/arm-none-eabi/lib
GCC_BIN_PREFIX=$(GCC_BIN_DIR)/arm-none-eabi
CC      = $(GCC_BIN_PREFIX)-gcc
LD      = $(GCC_BIN_PREFIX)-ld -v
AR      = $(GCC_BIN_PREFIX)-ar
AS      = $(GCC_BIN_PREFIX)-as
CP      = $(GCC_BIN_PREFIX)-objcopy
OD      = $(GCC_BIN_PREFIX)-objdump
NM      = $(GCC_BIN_PREFIX)-nm
SIZE    = $(GCC_BIN_PREFIX)-size

#
# If we can't find the toolchain (in /opt/paparazzi/arm-multilib or ~/sat) then try picking up the compilers from the path
#
else

# see if we can find the new arm-none-eabi, otherwise use the older arm-elf
HAVE_ARM_NONE_EABI_GCC := $(shell which arm-none-eabi-gcc)
ifeq ($(strip $(HAVE_ARM_NONE_EABI_GCC)),)
GCC_PREFIX = arm-elf
else
GCC_PREFIX = arm-none-eabi
GCC_LIB_DIR=$(shell dirname `which arm-none-eabi-gcc`)/../arm-none-eabi/lib
endif

CC      = $(shell which $(GCC_PREFIX)-gcc)
LD      = $(shell which $(GCC_PREFIX)-ld) -v
AR      = $(shell which $(GCC_PREFIX)-ar)
AS      = $(shell which $(GCC_PREFIX)-as)
CP      = $(shell which $(GCC_PREFIX)-objcopy)
OD      = $(shell which $(GCC_PREFIX)-objdump)
NM      = $(shell which $(GCC_PREFIX)-nm)
SIZE    = $(shell which $(GCC_PREFIX)-size)
endif

RM	= rm
TAR	= tar

MULTILIB = $(shell if $(CC) --print-multi-lib | grep thumb2 > /dev/null ; then echo "yes"; else echo "no"; fi)

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
	$(RM) -f $(LIBNAME).a $(LIBOBJS)
	$(RM) -f $(APPNAME).hex $(APPNAME).elf $(OBJS) *.lst $(APPNAME).dmp $(APPNAME).map
	$(RM) -f $(APPNAME_RAM).hex $(APPNAME_RAM).elf $(APPNAME_RAM).dmp $(APPNAME_RAM).map

# build lib
lib: $(LIBOBJS)
	$(RM) -f $(LIBNAME).a
	$(AR) -r $(LIBNAME).a $(LIBOBJS)

app: $(APPNAME).elf
	$(Q)echo "...copying"
	$(CP) $(CPFLAGS) $(APPNAME).elf $(APPNAME).hex
	$(OD) $(ODFLAGS) $(APPNAME).elf > $(APPNAME).dmp
#	$(Q)echo "....loading"
#	./lpc21isp -control bl.hex com4 38400 12000
#	./lpc21isp -control -termonly bl.hex com4 115200 12000

app_ram: $(APPNAME_RAM).elf
	$(Q)echo "...copying"
	$(CP) $(CPFLAGS) $(APPNAME_RAM).elf $(APPNAME_RAM).hex
	$(OD) $(ODFLAGS) $(APPNAME_RAM).elf > $(APPNAME_RAM).dmp

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
