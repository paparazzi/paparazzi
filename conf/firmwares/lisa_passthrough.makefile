#
#
# Using the STM32 as IO processor
#
#

SRC_ARCH=arch/$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
SRC_CSC=csc
SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_ROTOR_ARCH=$(SRC_FIRMWARE)/actuators/arch/$(ARCH)
SRC_IMU_ARCH=$(SRC_SUBSYSTEMS)/imu/arch/$(ARCH)

CFG_LISA_PASSTHROUGH = $(PAPARAZZI_SRC)/conf/firmwares/subsystems/lisa_passthrough


stm_passthrough.ARCHDIR = stm32
stm_passthrough.CFLAGS += -I$(SRC_FIRMWARE) -I$(SRC_LISA) -I$(SRC_LISA_ARCH) -I$(SRC_BOARD) -I$(SRC_ROTOR_ARCH) -I$(SRC_IMU_ARCH)
stm_passthrough.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
stm_passthrough.CFLAGS += -DPERIPHERALS_AUTO_INIT
stm_passthrough.srcs = $(SRC_LISA)/lisa_stm_passthrough_main.c

# Leds
stm_passthrough.CFLAGS += -DUSE_LED
stm_passthrough.srcs += $(SRC_ARCH)/led_hw.c

# Sys time
stm_passthrough.CFLAGS += -DUSE_SYS_TIME
stm_passthrough.CFLAGS += -DPERIODIC_FREQUENCY='512.'
stm_passthrough.CFLAGS += -DSYS_TIME_LED=1
stm_passthrough.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

# Telemetry
stm_passthrough.CFLAGS += -DDOWNLINK
stm_passthrough.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=UART2
stm_passthrough.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c
stm_passthrough.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
stm_passthrough.srcs += mcu_periph/uart.c
stm_passthrough.srcs += $(SRC_ARCH)/mcu_periph/uart_arch.c

# Link Overo
stm_passthrough.CFLAGS += -DUSE_OVERO_LINK
stm_passthrough.CFLAGS += -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp
stm_passthrough.CFLAGS += -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
stm_passthrough.CFLAGS += -DOVERO_LINK_LED_OK=3 -DOVERO_LINK_LED_KO=2
stm_passthrough.srcs += $(SRC_LISA)/lisa_overo_link.c           \
            $(SRC_LISA_ARCH)/lisa_overo_link_arch.c

# IMU
#
# include subsystem/lisa_passthrough/imu_b2_v1_1.makefile
#


stm_passthrough.srcs += math/pprz_trig_int.c
stm_passthrough.srcs += lisa/plug_sys.c

stm_passthrough.srcs += subsystems/commands.c

# Radio control
#
# include subsystem/lisa_passthrough/radio_control_joby.makefile
# or
# include subsystem/lisa_passthrough/radio_control_spektrum.makefile
#


# Actuators
#stm_passthrough.srcs += $(SRC_FIRMWARE)/actuators/supervision.c
#stm_passthrough.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL
#stm_passthrough.srcs += $(SRC_FIRMWARE)/actuators/actuators_asctec.c
#stm_passthrough.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c
#
#stm_passthrough.CFLAGS += -DACTUATORS_ASCTEC_I2C_DEV=i2c1
#stm_passthrough.CFLAGS += -DUSE_I2C1

# PWM actuator
ifndef SERVOS_REFRESH_FREQ
SERVOS_REFRESH_FREQ=50
endif
stm_passthrough.CFLAGS += -DSERVO_HZ=$(SERVOS_REFRESH_FREQ)
stm_passthrough.srcs += $(SRC_FIRMWARE)/actuators/actuators_pwm.c
stm_passthrough.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c

# Baro
stm_passthrough.srcs += $(SRC_BOARD)/baro_board.c
stm_passthrough.CFLAGS += -DUSE_I2C2
stm_passthrough.srcs += mcu_periph/i2c.c $(SRC_ARCH)/mcu_periph/i2c_arch.c

# Vanes
stm_passthrough.CFLAGS += -I$(SRC_CSC)
stm_passthrough.CFLAGS += -DUSE_CAN1 \
    -DUSE_CAN1 \
    -DUSE_USB_LP_CAN1_RX0_IRQ \
    -DCAN_PRESCALER=12 \
    -DCAN_SJW_TQ=CAN_SJW_1tq \
    -DCAN_BS1_TQ=CAN_BS1_3tq \
    -DCAN_BS2_TQ=CAN_BS2_4tq \
    -DCAN_ERR_RESUME=DISABLE
stm_passthrough.srcs += mcu_periph/can.c $(SRC_ARCH)/mcu_periph/can_arch.c
stm_passthrough.srcs += $(SRC_CSC)/csc_protocol.c

# ADC

stm_passthrough.srcs += $(SRC_ARCH)/mcu_periph/adc_arch.c
stm_passthrough.CFLAGS += -DUSE_AD1 \
    -DUSE_AD1_1 \
    -DUSE_AD1_2 \
    -DUSE_AD1_3 \
    -DUSE_AD1_4


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
overo_test_passthrough.srcs    += $(SRC_FMS)/udp_transport2.c subsystems/datalink/downlink.c
overo_test_passthrough.srcs    += $(SRC_FMS)/fms_network.c

#
# use the passthrough to calibrate throttle range for castle creations motor pwm motor controller
#
overo_blmc_calibrate.ARCHDIR  = omap
overo_blmc_calibrate.LDFLAGS += -levent -lm
overo_blmc_calibrate.CFLAGS  += -I$(ACINCLUDE) -I. -I$(PAPARAZZI_HOME)/var/include
overo_blmc_calibrate.CFLAGS  += -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
overo_blmc_calibrate.srcs     = $(SRC_FMS)/overo_blmc_calibrate.c
overo_blmc_calibrate.CFLAGS  += -DFMS_PERIODIC_FREQ=512
overo_blmc_calibrate.srcs    += $(SRC_FMS)/fms_periodic.c
overo_blmc_calibrate.srcs    += $(SRC_FMS)/fms_spi_link.c
#overo_blmc_calibrate.srcs    += $(SRC_FMS)/fms_gs_com.c
#overo_blmc_calibrate.CFLAGS  += -DDOWNLINK -DDOWNLINK_TRANSPORT=UdpTransport
#overo_blmc_calibrate.srcs    += $(SRC_FMS)/udp_transport2.c subsystems/datalink/downlink.c
overo_blmc_calibrate.srcs    += $(SRC_FMS)/fms_network.c
