# Hey Emacs, this is a -*- makefile -*-

sdlog_CFLAGS = -DDOWNLINK -DUSE_PPRZLOG
sdlog_srcs = subsystems/datalink/downlink.c subsystems/datalink/pprzlog_transport.c

ap.CFLAGS += $(sdlog_CFLAGS)
ap.srcs += $(sdlog_srcs)

