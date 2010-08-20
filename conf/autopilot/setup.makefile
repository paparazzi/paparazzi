#
# setup.makefile
#
#


CFG_SETUP=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/SETUP


SRC_SETUP=.
SRC_SETUP_ARCH=$(SRC_SETUP)/$(ARCH)
SRC_SETUP_TEST=$(SRC_SETUP)/

SETUP_INC = -I$(SRC_SETUP) -I$(SRC_SETUP_ARCH)



$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)


# a test program to tunnel between both uart

tunnel.CFLAGS += -DFBW -DLED
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c


# a test program to setup actuators
setup_actuators.CFLAGS += -DFBW -DLED -DTIME_LED=1 -DACTUATORS=\"servos_4015_hw.h\" -DSERVOS_4015 -DUSE_UART0 -DUART0_BAUD=B9600 -DDATALINK=PPRZ -DPPRZ_UART=Uart0
setup_actuators.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c pprz_transport.c setup_actuators.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/servos_4015_hw.c main.c


