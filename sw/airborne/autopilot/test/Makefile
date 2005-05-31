# $Id$

all:
	@echo "call with 'make TARGET=... compile (or load)'"

TARGET=check_uart

ARCH      = atmega128
VARINCLUDE = ../../../var/include
INCLUDES = -I .. -I ../../../include -I $(VARINCLUDE)
#INCLUDES = -I ../ -I ../../../include

LOCAL_CFLAGS= $(CTL_BRD_FLAGS)

CONF_DIR = ../../../../conf
CONF_XML = $(CONF_DIR)/conf.xml

ifneq ($(MAKECMDGOALS),clean)
  AIRFRAME_XML = $(CONF_DIR)/$(shell echo `../../../lib/ocaml/xml_get.out $(CONF_XML) "files" "airframe"`)
  CTL_BRD_VERSION=$(shell echo `../../../lib/ocaml/xml_get.out $(AIRFRAME_XML) "airframe" "ctl_board"`)
endif



ifeq ($(CTL_BRD_VERSION),V1_2_1)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2_1
endif

ifeq ($(CTL_BRD_VERSION),V1_2)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2
endif

test_modem.srcs  = test_modem.c ../modem.c 

check_uart.srcs  = check_uart.c ../uart.c

tx_adcs.srcs  = tx_adcs.c ../uart.c ../adc.c

test_v2xe.srcs  = test_v2xe.c

uart_tunnel.srcs  = uart_tunnel.c ../uart.c
check_spi.srcs  = check_spi.c ../uart.c ../spi.c ../link_fbw.c

check_spi.o : INCLUDES = -I ../../../include -I ../ -I ../../fly_by_wire

test_fp: test_fp.c ../flight_plan.h
	cc -o $@ -I sim_avr -I .. -I ../../fly_by_wire ../nav.c test_fp.c -lm

include ../../../../conf/Makefile.local
include ../../../../conf/Makefile.avr



clean: avr_clean
