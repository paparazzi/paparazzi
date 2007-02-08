# Makefile for the Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000





# a test program to setup actuators 
setup_actuators.ARCHDIR = $(ARCHI)
setup_actuators.ARCH = arm7tdmi
setup_actuators.TARGET = setup_actuators
setup_actuators.TARGETDIR = setup_actuators

setup_actuators.CFLAGS += -DFBW -DCONFIG=\"tiny.h\" -DLED -DTIME_LED=1 -DACTUATORS=\"servos_4015_hw.h\" -DSERVOS_4015 -DUSE_UART0 -DUART0_BAUD=B9600 -DDATALINK=PPRZ -DPPRZ_UART=Uart0
setup_actuators.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c pprz_transport.c setup_actuators.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/servos_4015_hw.c main.c


# a test program to tunnel between both uart
tunnel.ARCHDIR = $(ARCHI)
tunnel.ARCH = arm7tdmi
tunnel.TARGET = tunnel
tunnel.TARGETDIR = tunnel

tunnel.CFLAGS += -DFBW -DCONFIG=\"tiny.h\" -DLED -DTIME_LED=1
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c