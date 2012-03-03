# Quiet compilation
Q=@

DEBUG		=n
MACHINE := $(shell uname -m)
AS86		=as86 -0 -a
LD86		=ld86 -0
AS		=as
LD		=ld
LDFLAGS		=-lm
HOSTCC		=gcc
CC		=gcc
MAKE		=make
CPP		=$(CC) -E
AR		=ar
STRIP		=strip
MKDIR		=mkdir
OCAMLC	 	=ocamlc

CFLAGS		=-Wall -Wstrict-prototypes -I/usr/X11R6/include -I`$(OCAMLC) -where`
ifeq ($(DEBUG),y)
  CFLAGS		+=-g -O 
else
  CFLAGS		+=-O3 
endif 		

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  CFLAGS          += -fPIC
else
  CFLAGS          += -march=i486 -falign-loops=2 -falign-jumps=2 -falign-functions=2 -DARCH_I386
endif

LDFLAGSX	=-lX11 -L/usr/X11R6/lib

#BINDIR		=bin-$(shell uname -m)
BINDIR		=.

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Linux")
  OBJFILES=pprzlib.o hdlc.o demod_afsk12.o demodml.o costabi.o gen_hdlc.o ml_hdlc.o demod.cmo hdlc.cmo
  ALLTARGETS=$(BINDIR)/multimon multimon.cma
endif
ifeq ("$(UNAME)","linux")
  OBJFILES=pprzlib.o hdlc.o demod_afsk12.o demodml.o costabi.o gen_hdlc.o ml_hdlc.o demod.cmo hdlc.cmo
  ALLTARGETS=$(BINDIR)/multimon multimon.cma
endif
ifeq ("$(UNAME)","Darwin")
  OBJFILES=demodml.o ml_hdlc.o demod.cmo hdlc.cmo
  ALLTARGETS=multimon.cma
endif

all:    $(ALLTARGETS) 

multimon.cma:   $(OBJFILES)
	@echo OLD $@
	$(Q)ocamlmklib -o multimon $^


$(BINDIR)/%.s:	%.c
		$(CC) $(CFLAGS) -S -o $@ $<

$(BINDIR)/%.o:	$(BINDIR)/%.s
		$(AS) -c -o $@ $<

$(BINDIR)/%.o:	%.c
	@echo CC $<
	$(Q)$(CC) $(CFLAGS) -c -o $@ $<

SRC_L2		=hdlc.c pprz.c
SRC_L1		= demod_afsk48p.c demod_display.c
SRC_MISC	= unixinput.c xdisplay.c

SRC_GEN		=gen.c gen_dtmf.c gen_sin.c gen_zvei.c gen_hdlc.c costabi.c

OBJ_L2		=$(SRC_L2:%.c=$(BINDIR)/%.o)
OBJ_L1		=$(SRC_L1:%.c=$(BINDIR)/%.o)
OBJ_MISC	=$(SRC_MISC:%.c=$(BINDIR)/%.o)

OBJ_GEN		=$(SRC_GEN:%.c=$(BINDIR)/%.o)

$(BINDIR):
		$(MKDIR) $(BINDIR)

$(BINDIR)/multimon:	$(OBJ_L2) $(OBJ_L1) $(OBJ_MISC)
	@echo LD $@
	$(Q)$(CC) $^ $(LDFLAGS) $(LDFLAGSX) -o $@

$(BINDIR)/gen:		$(OBJ_GEN)
			$(CC) $^ $(LDFLAGS) -o $@

$(BINDIR)/mkcostab:	$(BINDIR)/mkcostab.o
			$(CC) $^ $(LDFLAGS) -o $@

costabi.c costabf.c:	$(BINDIR)/mkcostab
			$(BINDIR)/mkcostab


libtest: pprzlib.o demodml.c demod.ml test.ml
	$(OCAMLC) -custom -o $@ pprzlib.o demodml.c -I +lablgtk2 unix.cma lablgtk.cma demod.ml test.ml

hdlc_test : multimon.cma test_gen_hdlc.ml
	$(OCAMLC) -o $@ -custom -I +lablgtk2 -thread unix.cma threads.cma lablgtk.cma gtkThread.cmo -I . $^ -cclib -ljack

hdlc.cmo : hdlc.cmi

%.cmo : %.ml
	$(OCAMLC) -c $<

%.cmi : %.mli
	$(OCAMLC) $<

clean:
	rm -fr *.cm* mkcostab
	$(RM) -f core `find . -name '*.[oas]' -print`
	$(RM) -f core `find . -name 'core' -print`
	$(RM) -f core costabi.c costabf.c *~
	$(RM) $(BINDIR)/multimon

depend dep:
	$(CPP) -M $(CFLAGS) $(SRC_MISC) $(SRC_L1) $(SRC_L2) $(SRC_GEN) mkcostab.c > $(BINDIR)/.depend

ifeq ($(BINDIR)/.depend,$(wildcard $(BINDIR)/.depend))
include $(BINDIR)/.depend
endif
