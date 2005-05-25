DEBUG		=n
OS		=$(shell uname)

CFLAGS		=-Wall -Wstrict-prototypes -I/usr/X11R6/include
ifeq ($(OS),SunOS)
ifeq ($(DEBUG),y)
CFLAGS		+=-g -O -DSUN_AUDIO -DARCH_SPARC
else
CFLAGS		+=-O3 -DSUN_AUDIO -DARCH_SPARC
endif
LDFLAGSX	=-lX11 -L/usr/X11R6/lib -R/usr/X11R6/lib -lsocket -lnsl
else
ifeq ($(DEBUG),y)
CFLAGS		+=-g -O -march=i486 -falign-loops=2 -falign-jumps=2 \
		  -falign-functions=2 -DARCH_I386
else
CFLAGS		+=-O3 -march=i486 -falign-loops=2 -falign-jumps=2 \
		  -falign-functions=2 -DARCH_I386
endif
LDFLAGSX	=-lX11 -L/usr/X11R6/lib
endif


BINDIR		=bin-$(shell uname -m)

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

all:		$(BINDIR) $(BINDIR)/multimon $(BINDIR)/gen

$(BINDIR)/%.s:	%.c
		$(CC) $(CFLAGS) -S -o $@ $<

$(BINDIR)/%.o:	$(BINDIR)/%.s
		$(AS) -c -o $@ $<

$(BINDIR)/%.o:	%.c
		$(CC) $(CFLAGS) -c -o $@ $<

SRC_L2		=hdlc.c pocsag.c pprz.c
SRC_L1		=demod_afsk12.c demod_afsk24.c demod_afsk24_2.c demod_afsk48p.c demod_afsk48.c
SRC_L1		+=demod_hapn48.c demod_fsk96.c
SRC_L1		+=demod_poc5.c demod_poc12.c demod_poc24.c
SRC_L1		+=demod_dtmf.c demod_zvei.c demod_display.c
SRC_MISC	=unixinput.c costabf.c xdisplay.c

SRC_GEN		=gen.c gen_dtmf.c gen_sin.c gen_zvei.c gen_hdlc.c costabi.c

OBJ_L2		=$(SRC_L2:%.c=$(BINDIR)/%.o)
OBJ_L1		=$(SRC_L1:%.c=$(BINDIR)/%.o)
OBJ_MISC	=$(SRC_MISC:%.c=$(BINDIR)/%.o)

OBJ_GEN		=$(SRC_GEN:%.c=$(BINDIR)/%.o)

$(BINDIR):
		$(MKDIR) $(BINDIR)

$(BINDIR)/multimon:	$(OBJ_L2) $(OBJ_L1) $(OBJ_MISC)
			$(CC) $^ $(LDFLAGS) $(LDFLAGSX) -o $@

$(BINDIR)/gen:		$(OBJ_GEN)
			$(CC) $^ $(LDFLAGS) -o $@

$(BINDIR)/mkcostab:	$(BINDIR)/mkcostab.o
			$(CC) $^ $(LDFLAGS) -o $@

costabi.c costabf.c:	$(BINDIR)/mkcostab
			$(BINDIR)/mkcostab

clean:
		$(RM) -f core `find . -name '*.[oas]' -print`
		$(RM) -f core `find . -name 'core' -print`
		$(RM) -f core costabi.c costabf.c *~


depend dep:
		$(CPP) -M $(CFLAGS) $(SRC_MISC) $(SRC_L1) $(SRC_L2) $(SRC_GEN) mkcostab.c > $(BINDIR)/.depend

ifeq ($(BINDIR)/.depend,$(wildcard $(BINDIR)/.depend))
include $(BINDIR)/.depend
endif
