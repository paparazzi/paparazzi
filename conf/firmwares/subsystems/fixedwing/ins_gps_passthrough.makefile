# Hey Emacs, this is a -*- makefile -*-

ins_CFLAGS = -DINS_TYPE_H=\"subsystems/ins/ins_gps_passthrough_utm.h\"
ins_srcs   += $(SRC_SUBSYSTEMS)/ins.c
ins_srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough_utm.c


ap.CFLAGS += $(ins_CFLAGS)
ap.srcs   += $(ins_srcs)

sim.CFLAGS += $(ins_CFLAGS)
sim.srcs   += $(ins_srcs)

nps.CFLAGS += $(ins_CFLAGS)
nps.srcs   += $(ins_srcs)

