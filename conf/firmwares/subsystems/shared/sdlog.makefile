# Hey Emacs, this is a -*- makefile -*-

sdlog_CFLAGS = -DDOWNLINK
sdlog_srcs = subsystems/datalink/downlink.c modules/loggers/pprzlog_tp.c pprzlink/src/pprzlog_transport.c

ap.CFLAGS += $(sdlog_CFLAGS)
ap.srcs += $(sdlog_srcs)

