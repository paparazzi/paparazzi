

all:
	@echo "call with 'make TARGET=... compile (or load)'"

TARGET=check_uart

LOCAL_CFLAGS= $(CTL_BRD_FLAGS)

ARCH      = atmega8
INCLUDES = -I ../ -I ../../../include -I ../../../var/include

CONF_DIR = ../../../../conf
CONF_XML = $(CONF_DIR)/conf.xml
XML_GET=../../../lib/ocaml/xml_get.out

ifeq ($(CTL_BRD_VERSION),V1_2_1)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2_1
endif

ifeq ($(CTL_BRD_VERSION),V1_2)
CTL_BRD_FLAGS=-DCTL_BRD_V1_2
endif

ifeq ($(CTL_BRD_VERSION),V1_1)
CTL_BRD_FLAGS=-DCTL_BRD_V1_1
endif

rc_transmitter.srcs  = rc_transmitter.c ../ppm.c ../uart.c

setup_servos.srcs  = setup_servos.c ../uart.c ../servo.c

check_uart.srcs  = check_uart.c ../uart.c

tx_adcs.srcs = tx_adcs.c ../uart.c ../adc_fbw.c

check_spi.srcs = check_spi.c ../uart.c ../spi.c


include ../../../../conf/Makefile.local
include ../../../../conf/Makefile.avr

clean: avr_clean
