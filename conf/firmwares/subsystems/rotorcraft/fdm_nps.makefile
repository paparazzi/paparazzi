# Hey Emacs, this is a -*- makefile -*-

include $(CFG_ROTORCRAFT)/fdm_jsbsim.makefile

$(warning The fdm_nps subsystem has been renamed, please replace <subsystem name="fdm" type="nps"/> with <subsystem name="fdm" type="jsbsim"/> in your airframe file.)
