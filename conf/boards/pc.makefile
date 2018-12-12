#
# pc.makefile
#
# Linux PC
#

ARCH=sim
BOARD = pc
BOARD_VERSION=

# specify host platform for Rust builds
UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	RUST_ARCH = x86_64-apple-darwin
else
	RUST_ARCH = x86_64-unknown-linux-gnu
endif

BOARD_CFG = \"boards/pc_sim.h\"

$(TARGET).ARCHDIR = $(ARCH)

MODEM_HOST ?= 127.0.0.1

