# Hey Emacs, this is a -*- makefile -*-

#
# NPS SITL Simulator
#
# SITL specific makefile
#
# include Makefile.nps instead of Makefile.sim
$(info >>>Calling nps.makefile)
nps.MAKEFILE = nps
include $(CFG_SHARED)/nps_common.makefile
nps.srcs += $(NPSDIR)/nps_main_sitl.c
