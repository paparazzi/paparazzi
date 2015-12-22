# Build shared pprz math library
#

# Quiet compilation
Q=@

PAPARAZZI_SRC ?= ../../..

CC= gcc
CFLAGS= -fpic
INCLUDES= -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/sw/airborne

PAPARAZZI_HOME ?= $(PAPARAZZI_SRC)

# build in $(PAPARAZZI_HOME)/var/build/math
BUILDDIR ?= $(PAPARAZZI_HOME)/var/build/math

PREFIX ?= /usr/local

LIB_INSTALLDIR=${PREFIX}/lib
INCLUDE_INSTALLDIR=${PREFIX}/include/pprz
PKGCONFIG_INSTALLDIR=${PREFIX}/lib/pkgconfig
PKGCONFIG_FILE=pprzmath.pc

SRC= $(wildcard *.c)
OBJ= $(addprefix $(BUILDDIR)/,$(SRC:.c=.o))

LIBNAME=libpprzmath

all:
	@cat README.md

shared_lib: $(OBJ)
	@echo CREATE shared $(LIBNAME)
	$(Q)$(CC) -shared -o $(BUILDDIR)/$(LIBNAME).so $(OBJ) -lm

install_shared_lib: shared_lib
	test -d $(LIB_INSTALLDIR) || mkdir -p $(LIB_INSTALLDIR)
	cp $(BUILDDIR)/$(LIBNAME).so $(LIB_INSTALLDIR)
	test -d $(INCLUDE_INSTALLDIR)/math || mkdir -p $(INCLUDE_INSTALLDIR)/math
	cp *.h $(INCLUDE_INSTALLDIR)/math
	cp ../../include/std.h $(INCLUDE_INSTALLDIR)
	test -d $(PKGCONFIG_INSTALLDIR) || mkdir -p $(PKGCONFIG_INSTALLDIR)
	sed -e 's#PREFIX#$(PREFIX)#g' $(PKGCONFIG_FILE).in > $(PKGCONFIG_INSTALLDIR)/$(PKGCONFIG_FILE)

$(BUILDDIR)/%.o: %.c
	$(Q)test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	$(Q)$(CC) -c $< $(CFLAGS) $(INCLUDES) -o $@

clean:
	$(Q)rm -f $(BUILDDIR)/*.o $(BUILDDIR)/$(LIBNAME).so

.PHONY: all clean shared_lib install_shared_lib
