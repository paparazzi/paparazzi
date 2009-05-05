
ARCHI=geode

SRC_RDY=readyboard
SRC_FMS=fms
SRC_BOOZ=booz

PERIODIC_FREQ = 512

ap.ARCHDIR = $(ARCHI)

ap.LDFLAGS = -lm -levent -lrt

ap.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS) -I$(SRC_BOOZ) -DPERIODIC_FREQ=$(PERIODIC_FREQ)

ap.srcs =$(SRC_RDY)/ready_main.c
ap.srcs+=$(SRC_RDY)/rdyb_gpio.c
ap.srcs+=$(SRC_RDY)/rdyb_timer.c

#ap.CFLAGS += -DUSE_XSENS_AHRS
ap.srcs+=$(SRC_RDY)/rdyb_xsens.c
ap.srcs+=$(SRC_FMS)/fms_serial_port.c
ap.srcs+=$(SRC_RDY)/rdyb_estimator.c
ap.srcs+=$(SRC_RDY)/rdyb_ahrs.c
#ap.srcs+=$(SRC_RDY)/rdyb_control.c
ap.srcs+=$(SRC_RDY)/ahrs_quat_fast_ekf.c

ap.srcs+=$(SRC_RDY)/elevator_control.c
ap.srcs+=$(SRC_RDY)/pid_generic.c
ap.srcs+= commands.c

ap.CFLAGS+= -DACTUATORS=\"rdyb_actuators.h\"
ap.srcs+= actuators.c $(SRC_RDY)/rdyb_actuators.c

ap.CFLAGS  += `pkg-config --cflags glib-2.0`
ap.LDFLAGS += `pkg-config --libs glib-2.0`
ap.srcs+= $(SRC_RDY)/rdyb_event_manager.c


ap.srcs+=$(SRC_RDY)/rdyb_can.c
ap.srcs+=$(SRC_RDY)/rdyb_throttle.c

ap.CFLAGS += -DDOWNLINK
ap.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
ap.srcs += $(SRC_FMS)/fms_network.c
ap.srcs += $(SRC_FMS)/udp_transport.c
ap.srcs += downlink.c

ap.srcs += $(SRC_RDY)/rdyb_telemetry.c
ap.srcs += $(SRC_RDY)/rdyb_datalink.c
ap.CFLAGS += -DLINK_HOST=\"255.255.255.255\"




#
#
# test timer
#
#
test_timer.ARCHDIR = $(ARCHI)

test_timer.LDFLAGS = -lm -levent -lrt

test_timer.CFLAGS += -I$(SRC_RDY)

test_timer.srcs=$(SRC_RDY)/test_timer.c
test_timer.srcs+=$(SRC_RDY)/rdyb_timer.c
test_timer.srcs+=$(SRC_RDY)/rdyb_gpio.c

#
#
# test telemetry
#
#
test_telemetry.ARCHDIR = $(ARCHI)

test_telemetry.LDFLAGS = -lm -levent -lrt

test_telemetry.CFLAGS += -I$(SRC_RDY) -I$(SRC_FMS)

test_telemetry.srcs =$(SRC_RDY)/test_telemetry.c
test_telemetry.srcs+=$(SRC_RDY)/rdyb_timer.c
test_telemetry.srcs+=$(SRC_RDY)/rdyb_gpio.c

test_telemetry.CFLAGS += -DDOWNLINK
test_telemetry.CFLAGS += -DDOWNLINK_TRANSPORT=UdpTransport
test_telemetry.srcs += $(SRC_FMS)/fms_network.c
test_telemetry.srcs += $(SRC_FMS)/udp_transport.c
test_telemetry.srcs += downlink.c


#
#
# test xtend
#
#
test_xtend.ARCHDIR = $(ARCHI)

test_xtend.LDFLAGS = -lm -levent -lrt

test_xtend.CFLAGS += -I$(SRC_RDY)

test_xtend.srcs=$(SRC_RDY)/test_xtend.c
test_xtend.srcs+=$(SRC_RDY)/rdyb_timer.c
test_xtend.srcs+=$(SRC_RDY)/rdyb_gpio.c

test_xtend.CFLAGS += -DXTEND_DEVICE=\"/dev/ttyS2\"
#test_xtend.srcs+=$(SRC_RDY)/rdyb_xtend.c
test_xtend.srcs+=$(SRC_FMS)/fms_serial_port.c



#
#
# CSC1
#
#
FLASH_MODE = ISP
LPC21ISP_PORT = /dev/ttyUSB0

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000

LPC21ISP_CONTROL = -control

LDSCRIPT=$(SRC_ARCH)/LPC2129-ROM.ld

BOARD_CFG = \"csc_board_v1_0.h\"

SRC_CSC=csc

csc1.CFLAGS +=-DCSC_FRIED_CHIP

csc1.CFLAGS += -DCSC_BOARD_ID=0

csc1.ARCHDIR = arm7
csc1.ARCH = arm7tdmi
csc1.TARGET = csc1
csc1.TARGETDIR = csc1


csc1.CFLAGS += -I$(SRC_CSC)
csc1.CFLAGS += -DCONFIG=$(BOARD_CFG)
csc1.srcs += $(SRC_CSC)/csc_main.c
csc1.CFLAGS += -DLED
# -DTIME_LED=1


csc1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
csc1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

csc1.srcs += $(SRC_ARCH)/uart_hw.c

#csc1.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600 -DUART0_VIC_SLOT=5
#csc1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
#	                  -DDOWNLINK_DEVICE=Uart0
#csc1.srcs += downlink.c pprz_transport.c

csc1.CFLAGS += -DAP_LINK_CAN
csc1.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_2MHz
csc1.CFLAGS +=  -DCAN2_VIC_SLOT=4 -DCAN_ERR_VIC_SLOT=7
csc1.srcs += $(SRC_CSC)/csc_can.c

csc1.srcs += $(SRC_CSC)/csc_ap_link.c

csc1.srcs += $(SRC_CSC)/csc_servos.c

csc1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
csc1.CFLAGS += -DTHROTTLE_LINK=Uart1 
csc1.srcs += $(SRC_CSC)/csc_throttle.c


#
#
# CSC2
#
#
csc2.CFLAGS += -DCSC_BOARD_ID=1

csc2.ARCHDIR = arm7
csc2.ARCH = arm7tdmi
csc2.TARGET = csc2
csc2.TARGETDIR = csc2


csc2.CFLAGS += -I$(SRC_CSC)
csc2.CFLAGS += -DCONFIG=$(BOARD_CFG)
csc2.srcs += $(SRC_CSC)/csc_main.c
csc2.CFLAGS += -DLED -DTIME_LED=1


csc2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))' -DTIMER0_VIC_SLOT=1
csc2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

csc2.srcs += $(SRC_ARCH)/uart_hw.c

csc2.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B57600 -DUART0_VIC_SLOT=5
csc2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport \
	                  -DDOWNLINK_DEVICE=Uart0
csc2.srcs += downlink.c pprz_transport.c

csc2.CFLAGS += -DAP_LINK_CAN
csc2.CFLAGS += -DUSE_CAN2 -DCAN2_BTR=CANBitrate125k_2MHz
csc2.CFLAGS +=  -DCAN2_VIC_SLOT=4 -DCAN_ERR_VIC_SLOT=7
csc2.srcs += $(SRC_CSC)/csc_can.c

csc2.srcs += $(SRC_CSC)/csc_ap_link.c

csc2.srcs += $(SRC_CSC)/csc_servos.c

csc2.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600 -DUART1_VIC_SLOT=6
csc2.CFLAGS += -DTHROTTLE_LINK=Uart1 
csc2.srcs += $(SRC_CSC)/csc_throttle.c

