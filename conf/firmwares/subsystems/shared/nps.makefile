# Hey Emacs, this is a -*- makefile -*-

#
# NPS SITL Simulator
#
# SITL specific makefile
#
include $(CFG_SHARED)/nps_common.makefile

nps.srcs += $(NPSDIR)/nps_main_sitl.c
