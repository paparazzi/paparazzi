# Hey Emacs, this is a -*- makefile -*-

ins_CFLAGS = -DINS_TYPE_H=\"subsystems/ins/ins_gps_passthrough.h\"
ins_srcs   += $(SRC_SUBSYSTEMS)/ins.c
ins_srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough.c


ap.CFLAGS += $(ins_CFLAGS)
ap.srcs   += $(ins_srcs)

nps.CFLAGS += $(ins_CFLAGS)
nps.srcs   += $(ins_srcs)
