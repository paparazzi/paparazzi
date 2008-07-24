###############################################################################
#
# Makefile to generate standalone test code for PHYTEX LPC3180 board
# (C) 2008, Dirk Behme, dirk.behme@gmail.com
#
# Known problems:
# - All source files must be in one folder.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#
###############################################################################
# Only edit the following section
###############################################################################

FILENAME = lpc3180

###############################################################################
# specific stuff
###############################################################################

CC	= arm-none-eabi-gcc
LD	= arm-none-eabi-ld
DISASM  = arm-none-eabi-objdump
SREC	= arm-none-eabi-objcopy
TAR	= tar
MV	= mv

###############################################################################
# compiler flags
###############################################################################

CC_COMPILER	=  -c

###############################################################################
# ARM compiler flags
###############################################################################

CC_ARM	= -mcpu=arm926ej-s -mfpu=vfp -mfloat-abi=softfp

###############################################################################
# Warnings
###############################################################################

CC_WARNINGS	= -Wall

###############################################################################
# Includes
###############################################################################

CC_INCLUDE	= -I.

###############################################################################
# LD Flags
###############################################################################

LDFLAGS	= -nostdlib -Map $(patsubst %,%.map,$(FILENAME)) -T sdram.ld -L/opt/codesourcery/arm-none-eabi/arm-2008q1/lib/gcc/arm-none-eabi/4.2.3 -L/opt/codesourcery/arm-none-eabi/arm-2008q1/arm-none-eabi/lib -L/opt/codesourcery/arm-none-eabi/arm-2008q1/lib/gcc/arm-none-eabi/4.2.3

#LDLIBRARIES = -lm -lc -lgcc
LDLIBRARIES = -lmopt -lc -lgcc

###############################################################################
# Language-Flags, add debug information
###############################################################################

ADDED_CFLAGS	= -g

###############################################################################
# Produce listing of assembler output
###############################################################################

CLIST 		= -Wa,-a=$(patsubst %.o,%.lst,$@)

###############################################################################
# => Resulting C-Flags
###############################################################################

DFLAGS		= $(CC_COMPILER)	\
		  $(CC_OPTIM_TARGET)	\
		  $(CC_WARNINGS)	\
		  $(CC_INCLUDE)		\
		  $(CC_ARM)		\
		  $(ADDED_CFLAGS)
CFLAGS		= $(DFLAGS) $(CLIST)

###############################################################################
# Source- and Target-Definitions
###############################################################################

OTarget	    := $(FILENAME)   			# Name of output Object

###############################################################################
# Tar-Stuff
###############################################################################

SOURCEFILES :=  *.S *.h *.c Makefile

###############################################################################
# Phonies
###############################################################################

.PHONY: default
.PHONY: clean
.PHONY: tar

###############################################################################
# Make-Rules
#
# "make" or "make default"	:  compile $(FILENAME)
# "make clean"			:  cleanup - remove all *.o, *.d files
# "make tar"	: tar all *.h, *.c, *.S files and the Makefile, zip it
###############################################################################

default: init.o main.o vfp.o
		$(LD) $(LDFLAGS) -o $(FILENAME) init.o main.o vfp.o $(LDLIBRARIES)
		$(DISASM) -D $(FILENAME) > $(FILENAME).dis
		$(SREC) -O srec $(FILENAME) $(FILENAME).srec

init.o: init.S  Makefile
		$(CC) $(CFLAGS) -o init.o init.S

main.o: main.c Makefile
		$(CC) $(CFLAGS) -o main.o main.c

vfp.o: vfp.c Makefile
		$(CC) $(CFLAGS) -o vfp.o vfp.c

clean:
		$(RM) *.o *.lst *.dis *.map *.bin $(FILENAME)

distclean:
		$(RM) *.o *.lst *.dis *.map *.bin *.srec $(FILENAME) *~




