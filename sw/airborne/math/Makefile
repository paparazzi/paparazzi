# Build shared pprz math library
#

CC= gcc
CFLAGS= -fpic
INCLUDES= -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/sw/airborne

# build in ../../../var/build/math
ifndef BUILDDIR
BUILDDIR=../../../var/build/math
endif

ifndef PREFIX
PREFIX=/usr
endif
LIB_INSTALLDIR=${PREFIX}/lib
INCLUDE_INSTALLDIR=${PREFIX}/include/pprz
PKGCONFIG_INSTALLDIR=${PREFIX}/lib/pkgconfig
PKGCONFIG_FILE=pprzmath.pc

SRC= $(wildcard *.c)
OBJ= $(addprefix $(BUILDDIR)/,$(SRC:.c=.o))

LIBNAME=libpprzmath

all:
	@cat README

shared_lib: $(OBJ)
	$(CC) -shared -o $(BUILDDIR)/$(LIBNAME).so $(OBJ)

install_shared_lib: shared_lib
	test -d $(LIB_INSTALLDIR) || mkdir -p $(LIB_INSTALLDIR)
	cp $(BUILDDIR)/$(LIBNAME).so $(LIB_INSTALLDIR)
	test -d $(INCLUDE_INSTALLDIR)/math || mkdir -p $(INCLUDE_INSTALLDIR)/math
	cp *.h $(INCLUDE_INSTALLDIR)/math
	cp ../../include/std.h $(INCLUDE_INSTALLDIR)
	test -d $(PKGCONFIG_INSTALLDIR) || mkdir -p $(PKGCONFIG_INSTALLDIR)
	sed -e 's#PREFIX#$(PREFIX)#g' $(PKGCONFIG_FILE).in > $(PKGCONFIG_INSTALLDIR)/$(PKGCONFIG_FILE)

$(BUILDDIR)/%.o: %.c
	test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	$(CC) -c $< $(CFLAGS) $(INCLUDES) -o $@

clean:
	rm -f $(BUILDDIR)/*.o $(BUILDDIR)/$(LIBNAME).so

