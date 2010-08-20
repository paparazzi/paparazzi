#
#
# Using the STM32 as IO processor
#
#

SRC_ARCH=$(ARCH)
SRC_BOOZ=booz
SRC_BOOZ_ARCH=$(SRC_BOOZ)/arch/$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
SRC_CSC=csc

CFG_LISA_PASSTHROUGH = $(PAPARAZZI_SRC)/conf/autopilot/subsystems/lisa_passthrough


stm_passthrough.ARCHDIR = stm32
stm_passthrough.TARGET = stm_passthrough
stm_passthrough.TARGETDIR = stm_passthrough
stm_passthrough.CFLAGS += -I$(SRC_LISA) -I$(SRC_LISA_ARCH) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
stm_passthrough.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
stm_passthrough.srcs = $(SRC_LISA)/lisa_stm_passthrough_main.c \
                       $(SRC_ARCH)/stm32_exceptions.c          \
                       $(SRC_ARCH)/stm32_vector_table.c

# Leds
stm_passthrough.CFLAGS += -DUSE_LED
stm_passthrough.srcs += $(SRC_ARCH)/led_hw.c

# Sys time
stm_passthrough.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
stm_passthrough.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
stm_passthrough.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

# Telemetry
stm_passthrough.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
stm_passthrough.srcs += $(SRC_ARCH)/uart_hw.c
stm_passthrough.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2 
stm_passthrough.srcs += downlink.c pprz_transport.c

# Link Overo
stm_passthrough.CFLAGS += -DUSE_OVERO_LINK -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
stm_passthrough.CFLAGS += -DOVERO_LINK_LED_OK=3 -DOVERO_LINK_LED_KO=2 -DUSE_DMA1_C2_IRQ
stm_passthrough.srcs += $(SRC_LISA)/lisa_overo_link.c           \
			$(SRC_LISA_ARCH)/lisa_overo_link_arch.c

# IMU
stm_passthrough.CFLAGS += -DBOOZ_IMU_TYPE_H=\"imu/booz_imu_b2.h\"
stm_passthrough.CFLAGS += -DIMU_B2_VERSION_1_1
stm_passthrough.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2001
stm_passthrough.srcs += $(SRC_BOOZ)/booz_imu.c
stm_passthrough.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
stm_passthrough.srcs += $(SRC_BOOZ)/imu/booz_imu_b2.c $(SRC_BOOZ_ARCH)/imu/booz_imu_b2_arch.c
stm_passthrough.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c
stm_passthrough.srcs += $(SRC_BOOZ)/peripherals/booz_ms2001.c  $(SRC_BOOZ_ARCH)/peripherals/booz_ms2001_arch.c
stm_passthrough.srcs += math/pprz_trig_int.c

stm_passthrough.srcs += $(SRC_BOOZ)/booz2_commands.c

# Radio control
#
# include subsystem/lisa_passthrough/radio_control_joby.makefile
# or
# include subsystem/lisa_passthrough/radio_control_spektrum.makefile
#


# Actuators
#stm_passthrough.srcs += $(SRC_BOOZ)/actuators/booz_supervision.c
#stm_passthrough.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL
#stm_passthrough.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
#stm_passthrough.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
#
#stm_passthrough.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
#stm_passthrough.CFLAGS += -DUSE_I2C1

# PWM actuator
ifndef SERVOS_REFRESH_FREQ
SERVOS_REFRESH_FREQ=50
endif
stm_passthrough.CFLAGS += -DSERVO_HZ=$(SERVOS_REFRESH_FREQ)
stm_passthrough.srcs += $(SRC_BOOZ)/actuators/booz_actuators_pwm.c
stm_passthrough.srcs += $(SRC_BOOZ_ARCH)/actuators/booz_actuators_pwm_arch.c

# Baro
stm_passthrough.srcs += $(SRC_LISA)/lisa_baro.c
stm_passthrough.CFLAGS += -DUSE_I2C2
stm_passthrough.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

# Vanes
stm_passthrough.CFLAGS += -I $(SRC_CSC) 
stm_passthrough.CFLAGS += -DUSE_CAN1 \
	-DUSE_CAN1 \
	-DUSE_USB_LP_CAN1_RX0_IRQ \
	-DCAN_PRESCALER=12 \
	-DCAN_SJW_TQ=CAN_SJW_1tq \
	-DCAN_BS1_TQ=CAN_BS1_3tq \
	-DCAN_BS2_TQ=CAN_BS2_4tq \
	-DCAN_ERR_RESUME=DISABLE
stm_passthrough.srcs += can.c $(SRC_ARCH)/can_hw.c
stm_passthrough.srcs += $(SRC_CSC)/csc_protocol.c 

# Battery monitor









#
#
#
#
# test passthrough , aka using stm32 as io processor
# this demonstrates
#   -link with io processor
#   -periodic event
#   -telemetry and datalink
#
SRC_FMS=fms

overo_test_passthrough.ARCHDIR  = omap
overo_test_passthrough.LDFLAGS += -levent -lm
overo_test_passthrough.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_test_passthrough.CFLAGS  += -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
overo_test_passthrough.srcs     = $(SRC_FMS)/overo_test_passthrough.c
overo_test_passthrough.CFLAGS  += -DFMS_PERIODIC_FREQ=512
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_periodic.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_spi_link.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_gs_com.c
overo_test_passthrough.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
overo_test_passthrough.srcs    += $(SRC_FMS)/udp_transport2.c downlink.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_network.c
