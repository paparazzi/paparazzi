#
# Copyright (C) 2013 Gautier Hattenberger
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

#
# PPRZ datalink makefile to include to your project
#
# define PPRZ_DATALINK_DIR variable to your extract folder
# define PPRZ_DATALINK_PORT variable to your UART port
# add PPRZ_DATALINK_CFLAGS to your CFLAGS
# and PPRZ_DATALINK_SRCS to your sources list
#

PPRZ_DATALINK_CFLAGS = -DPPRZ_DATALINK_EXPORT -I$(PPRZ_DATALINK_DIR)/datalink
PPRZ_DATALINK_CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(PPRZ_DATALINK_PORT) -DPPRZ_UART=$(PPRZ_DATALINK_PORT) -DDOWNLINK_TRANSPORT=PprzTransport
PPRZ_DATALINK_CFLAGS += -DDATALINK=PPRZ
PPRZ_DATALINK_SRCS = $(PPRZ_DATALINK_DIR)/datalink/downlink.c $(PPRZ_DATALINK_DIR)/datalink/pprz_transport.c
PPRZ_DATALINK_OBJS = $(PPRZ_DATALINK_SRCS:.c=.o)

