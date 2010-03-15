ARCHI=stm32
SRC_LISA=lisa
SRC_ARCH=$(ARCHI)

BOARD_CFG=\"boards/olimex_stm32-h103.h\"


#
# test leds
#
test_led.ARCHDIR = $(ARCHI)
test_led.TARGET = test_led
test_led.TARGETDIR = test_led
test_led.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_led.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_led.srcs += $(SRC_LISA)/test_led.c       \
                 $(SRC_LISA)/exceptions.c     \
                 $(SRC_LISA)/vector_table.c
test_led.CFLAGS += -DUSE_LED


#
# test periodic
#
test_periodic.ARCHDIR = $(ARCHI)
test_periodic.TARGET = test_periodic
test_periodic.TARGETDIR = test_periodic
test_periodic.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_periodic.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_periodic.srcs += $(SRC_LISA)/test_periodic.c  \
                      $(SRC_LISA)/exceptions.c     \
                      $(SRC_LISA)/vector_table.c
test_periodic.CFLAGS += -DUSE_LED
test_periodic.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_periodic.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_periodic.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c


#
# test uart
#
test_uart.ARCHDIR = $(ARCHI)
test_uart.TARGET = test_uart
test_uart.TARGETDIR = test_uart
test_uart.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_uart.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_uart.srcs = $(SRC_LISA)/test_uart.c         \
                      $(SRC_LISA)/exceptions.c   \
                      $(SRC_LISA)/vector_table.c
test_uart.CFLAGS += -DUSE_LED
test_uart.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_uart.srcs += $(SRC_ARCH)/uart_hw.c



#
# test telemetry1
#
test_telemetry1.ARCHDIR = $(ARCHI)
test_telemetry1.TARGET = test_telemetry1
test_telemetry1.TARGETDIR = test_telemetry1
test_telemetry1.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry1.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry1.srcs = $(SRC_LISA)/test_telemetry.c \
                      $(SRC_LISA)/exceptions.c      \
                      $(SRC_LISA)/vector_table.c
test_telemetry1.CFLAGS += -DUSE_LED
test_telemetry1.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry1.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry1.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry1.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_telemetry1.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry1.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_telemetry1.srcs += downlink.c pprz_transport.c


#
# test telemetry2
#
test_telemetry2.ARCHDIR = $(ARCHI)
test_telemetry2.TARGET = test_telemetry2
test_telemetry2.TARGETDIR = test_telemetry2
test_telemetry2.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry2.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry2.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_LISA)/exceptions.c     \
                       $(SRC_LISA)/vector_table.c
test_telemetry2.CFLAGS += -DUSE_LED
test_telemetry2.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry2.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry2.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry2.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
test_telemetry2.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry2.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2 
test_telemetry2.srcs += downlink.c pprz_transport.c


#
# test telemetry3
#
test_telemetry3.ARCHDIR = $(ARCHI)
test_telemetry3.TARGET = test_telemetry3
test_telemetry3.TARGETDIR = test_telemetry3
test_telemetry3.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry3.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_telemetry3.srcs = $(SRC_LISA)/test_telemetry.c \
                       $(SRC_LISA)/exceptions.c     \
                       $(SRC_LISA)/vector_table.c
test_telemetry3.CFLAGS += -DUSE_LED
test_telemetry3.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry3.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry3.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry3.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_telemetry3.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry3.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart3 
test_telemetry3.srcs += downlink.c pprz_transport.c

#
# test datalink
#
test_datalink.ARCHDIR = $(ARCHI)
test_datalink.TARGET = test_datalink
test_datalink.TARGETDIR = test_datalink
test_datalink.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_datalink.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_datalink.srcs = $(SRC_LISA)/test_datalink.c \
                     $(SRC_LISA)/exceptions.c    \
                     $(SRC_LISA)/vector_table.c
test_datalink.CFLAGS += -DUSE_LED
test_datalink.CFLAGS += -DUSE_SYS_TIME # -DSYS_TIME_LED=1
test_datalink.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_datalink.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_datalink.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_datalink.srcs += $(SRC_ARCH)/uart_hw.c
test_datalink.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_datalink.srcs += downlink.c pprz_transport.c

test_datalink.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1


#
# test rc 2.4
#

SRC_BOOZ = booz
SRC_BOOZ_TEST = $(SRC_BOOZ)/test

test_rc_24.ARCHDIR = $(ARCHI)
test_rc_24.TARGET = test_rc_24
test_rc_24.TARGETDIR = test_rc_24
test_rc_24.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -DPERIPHERALS_AUTO_INIT
test_rc_24.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) 
test_rc_24.srcs += $(SRC_BOOZ_TEST)/booz2_test_radio_control.c \
                   $(SRC_LISA)/exceptions.c                     \
                   $(SRC_LISA)/vector_table.c
test_rc_24.CFLAGS += -DUSE_LED
test_rc_24.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_rc_24.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_rc_24.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_rc_24.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
test_rc_24.srcs += $(SRC_ARCH)/uart_hw.c
test_rc_24.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
test_rc_24.srcs += downlink.c pprz_transport.c

test_rc_24.CFLAGS += -DUSE_RADIO_CONTROL -DRADIO_CONTROL_LED=1
test_rc_24.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"booz_radio_control_spektrum.h\"
test_rc_24.CFLAGS += -DRADIO_CONTROL_SPEKTRUM_MODEL_H=\"booz_radio_control_spektrum_dx7se.h\"
test_rc_24.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B115200
test_rc_24.CFLAGS += -DRADIO_CONTROL_LINK=Uart2
test_rc_24.srcs += $(SRC_BOOZ)/booz_radio_control.c \
                   $(SRC_BOOZ)/booz_radio_control_spektrum.c \
#                   $(SRC_ARCH)/uart_hw.c


#
# test servos
#

SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCHI)

test_servos.ARCHDIR = $(ARCHI)
test_servos.TARGET = test_servos
test_servos.TARGETDIR = test_servos
test_servos.CFLAGS += -I$(SRC_LISA) -I$(ARCHI) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
test_servos.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_servos.LDFLAGS += -lm
test_servos.srcs += $(SRC_LISA)/test_servos.c 	\
                    $(SRC_LISA)/exceptions.c    \
                    $(SRC_LISA)/vector_table.c
test_servos.CFLAGS += -DUSE_LED
test_servos.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_servos.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
test_servos.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

test_servos.srcs += $(SRC_BOOZ)/actuators/booz_actuators_pwm.c $(SRC_BOOZ_ARCH)/actuators/booz_actuators_pwm_hw.c


#test_servos.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
#test_servos.srcs += $(SRC_ARCH)/uart_hw.c
#test_servos.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
#test_servos.srcs += downlink.c pprz_transport.c





#
# test Max1168
#


#
# test motor controllers
#
test_mc.ARCHDIR = $(ARCHI)
test_mc.TARGET = test_mc
test_mc.TARGETDIR = test_mc
test_mc.CFLAGS = -I$(SRC_LISA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_mc.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_mc.srcs = $(SRC_LISA)/test_mc.c      \
               $(SRC_LISA)/exceptions.c   \
               $(SRC_LISA)/vector_table.c
test_mc.CFLAGS += -DUSE_LED
test_mc.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_mc.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_mc.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

