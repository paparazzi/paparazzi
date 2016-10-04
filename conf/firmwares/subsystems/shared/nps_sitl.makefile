# Hey Emacs, this is a -*- makefile -*-

#
# NPS SITL Simulator
#
# SITL specific makefile
#

# glib is still needed for some components (such as radio input)
nps.CFLAGS  += $(shell pkg-config glib-2.0 --cflags)
nps.LDFLAGS += $(shell pkg-config glib-2.0 --libs)

nps.srcs += $(NPSDIR)/nps_main_sitl.c
