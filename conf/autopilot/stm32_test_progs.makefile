ARCHI=stm32
SRC_FYA=fya
SRC_ARCH=$(ARCHI)

BOARD_CFG=\"fya_board.h\"


#
# test leds
#
test_led.ARCHDIR = $(ARCHI)
test_led.TARGET = test_led
test_led.TARGETDIR = test_led
test_led.CFLAGS += -I$(SRC_FYA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_led.CFLAGS += -DCONFIG=$(BOARD_CFG)
test_led.srcs += $(SRC_FYA)/test_led.c       \
                 $(SRC_FYA)/exceptions.c     \
                 $(SRC_FYA)/vector_table.c
test_led.CFLAGS += -DUSE_LED


#
# test periodic
#
test_periodic.ARCHDIR = $(ARCHI)
test_periodic.TARGET = test_periodic
test_periodic.TARGETDIR = test_periodic
test_periodic.CFLAGS += -I$(SRC_FYA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_periodic.CFLAGS += -DCONFIG=$(BOARD_CFG)
test_periodic.srcs += $(SRC_FYA)/test_periodic.c  \
                      $(SRC_FYA)/exceptions.c     \
                      $(SRC_FYA)/vector_table.c
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
test_uart.CFLAGS = -I$(SRC_FYA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_uart.CFLAGS += -DCONFIG=$(BOARD_CFG)
test_uart.srcs = $(SRC_FYA)/test_uart.c         \
                      $(SRC_FYA)/exceptions.c   \
                      $(SRC_FYA)/vector_table.c
test_uart.CFLAGS += -DUSE_LED
test_uart.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_uart.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_uart.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_uart.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_uart.srcs += $(SRC_ARCH)/uart_hw.c


#
# test telemetry
#
test_telemetry.ARCHDIR = $(ARCHI)
test_telemetry.TARGET = test_telemetry
test_telemetry.TARGETDIR = test_telemetry
test_telemetry.CFLAGS = -I$(SRC_FYA) -I$(ARCHI) -DPERIPHERALS_AUTO_INIT
test_telemetry.CFLAGS += -DCONFIG=$(BOARD_CFG)
test_telemetry.srcs = $(SRC_FYA)/test_telemetry.c         \
                      $(SRC_FYA)/exceptions.c   \
                      $(SRC_FYA)/vector_table.c
test_telemetry.CFLAGS += -DUSE_LED
test_telemetry.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
test_telemetry.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
test_telemetry.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_telemetry.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B57600
test_telemetry.srcs += $(SRC_ARCH)/uart_hw.c
test_telemetry.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart3 
test_telemetry.srcs += downlink.c pprz_transport.c