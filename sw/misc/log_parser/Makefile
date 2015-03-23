
# Quiet compilation
Q=@

#LIBNAME	= efsl
APPNAME = linuxtest

# Package definitions
PKG_NAME	= linuxtest
DATE		= $$(date +%Y%m%d)
DATEL		= $$(date +%Y%m%d-%H%M)

# Tool definitions
CC      = gcc
LD      = ld
AR      = ar
AS      = as
CP      = objcopy
OD		= objdump
RM		= rm
TAR		= tar

CFLAGS  = -Wall -I../../inc/ -I../../conf -L.

SOURCES =	linuxtest.c

all: $(APPNAME)

clean:
	$(Q)$(RM) -f core $(APPNAME).o $(APPNAME) $(APPNAME).obj $(APPNAME).exe

app: $(APPNAME)

$(APPNAME):	$(SOURCES) Makefile
	$(CC) $(CFLAGS) $(SOURCES) -o $(APPNAME)
#    -l$(LIBNAME)
