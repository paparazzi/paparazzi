# Hey Emacs, this is a -*- makefile -*-

ins_srcs   += $(SRC_SUBSYSTEMS)/ins.c
ins_srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough_utm.c


ap.CFLAGS += $(ins_CFLAGS)
ap.srcs   += $(ins_srcs)

sim.CFLAGS += $(ins_CFLAGS)
sim.srcs   += $(ins_srcs)

jsbsim.CFLAGS += $(ins_CFLAGS)
jsbsim.srcs   += $(ins_srcs)

nps.CFLAGS += $(ins_CFLAGS)
nps.srcs   += $(ins_srcs)

