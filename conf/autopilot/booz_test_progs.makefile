
################################################################################
#
#
#  Test program for the booz board
#
#
#
#
# every "firmware" makefile should have a description of available targets
# possible options for each of them, susbsystems and associated params for each of them
# 
#
#
#
################################################################################

ARCHI=arm7

BOARD_CFG=\"boards/booz_1.0.h\"
SRC_ARCH=$(ARCHI)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/lpc21

BOARD_CFG=\"boards/booz_1.0.h\"

#
# default configuration expected from board files
#
# MODEM_PORT = UART1
# MODEM_BAUD = B57600


#
# test_telemetry : Sends ALIVE telemetry messages
#
# used configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#
test_telemetry.ARCHDIR = $(ARCHI)
test_telemetry.ARCH = arm7tdmi
test_telemetry.TARGET = test_telemetry
test_telemetry.TARGETDIR = test_telemetry

test_telemetry.CFLAGS += -I$(PAPARAZZI_SRC)/conf
test_telemetry.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry.CFLAGS += -DPERIPHERALS_AUTO_INIT
test_telemetry.srcs   += test/test_telemetry.c
test_telemetry.CFLAGS += -DUSE_LED
test_telemetry.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./10.))' -DTIME_LED=1
test_telemetry.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

test_telemetry.CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_telemetry.srcs   += $(SRC_ARCH)/uart_hw.c

test_telemetry.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
test_telemetry.srcs   += downlink.c pprz_transport.c

