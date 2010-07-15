#
#
# Using the STM32 as IO processor
#
#

pt.ARCHDIR = stm32
pt.TARGET = pt
pt.TARGETDIR = pt
pt.CFLAGS += -I$(SRC_LISA) -I$(SRC_BOOZ) -I$(SRC_BOOZ_ARCH) -DPERIPHERALS_AUTO_INIT
pt.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
pt.srcs = $(SRC_LISA)/lisa_stm_passthrough_main.c \
          $(SRC_ARCH)/stm32_exceptions.c   \
          $(SRC_ARCH)/stm32_vector_table.c

# Leds
pt.CFLAGS += -DUSE_LED
pt.srcs += $(SRC_ARCH)/led_hw.c

# Sys time
pt.CFLAGS += -DUSE_SYS_TIME -DSYS_TIME_LED=1
pt.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./512.)'
pt.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

# Telemetry
pt.CFLAGS += -DUSE_UART2 -DUART2_BAUD=B57600
pt.srcs += $(SRC_ARCH)/uart_hw.c
pt.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart2 
pt.srcs += downlink.c pprz_transport.c

# Link Overo
pt.CFLAGS += -DUSE_OVERO_LINK -DOVERO_LINK_MSG_UP=AutopilotMessagePTUp -DOVERO_LINK_MSG_DOWN=AutopilotMessagePTDown
pt.CFLAGS += -DOVERO_LINK_LED_OK=3 -DOVERO_LINK_LED_KO=4 -DUSE_DMA1_C2_IRQ
pt.srcs += lisa/lisa_overo_link.c lisa/arch/stm32/lisa_overo_link_arch.c

# IMU
pt.CFLAGS += -DBOOZ_IMU_TYPE_H=\"imu/booz_imu_b2.h\"
pt.CFLAGS += -DIMU_B2_MAG_TYPE=IMU_B2_MAG_MS2001
pt.srcs += $(SRC_BOOZ)/booz_imu.c
pt.CFLAGS += -DUSE_SPI2 -DUSE_DMA1_C4_IRQ -DUSE_EXTI2_IRQ -DUSE_SPI2_IRQ
pt.srcs += $(SRC_BOOZ)/imu/booz_imu_b2.c $(SRC_BOOZ_ARCH)/imu/booz_imu_b2_arch.c
pt.srcs += $(SRC_BOOZ)/peripherals/booz_max1168.c $(SRC_BOOZ_ARCH)/peripherals/booz_max1168_arch.c
pt.srcs += $(SRC_BOOZ)/peripherals/booz_ms2001.c  $(SRC_BOOZ_ARCH)/peripherals/booz_ms2001_arch.c
pt.srcs += math/pprz_trig_int.c

pt.srcs += $(SRC_BOOZ)/booz2_commands.c

# Radio control
pt.CFLAGS += -DUSE_RADIO_CONTROL
pt.CFLAGS += -DRADIO_CONTROL_TYPE_H=\"radio_control/booz_radio_control_joby.h\"
pt.CFLAGS += -DRADIO_CONTROL_JOBY_MODEL_H=\"radio_control/booz_radio_control_joby_9ch.h\"
pt.srcs += $(SRC_BOOZ)/booz_radio_control.c \
           $(SRC_BOOZ)/radio_control/booz_radio_control_joby.c
pt.CFLAGS += -DRADIO_CONTROL_LED=6
pt.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B115200
pt.CFLAGS += -DRADIO_CONTROL_LINK=Uart3

# Actuators
#pt.srcs += $(SRC_BOOZ)/actuators/booz_supervision.c
#pt.CFLAGS += -DACTUATORS_ASCTEC_V2_PROTOCOL
#pt.srcs += $(SRC_BOOZ)/actuators/booz_actuators_asctec.c
#pt.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
#
#pt.CFLAGS += -DACTUATORS_ASCTEC_DEVICE=i2c1
#pt.CFLAGS += -DUSE_I2C1

# PWM actuator
pt.srcs += $(SRC_BOOZ)/actuators/booz_actuators_pwm.c
pt.srcs += $(SRC_BOOZ_ARCH)/actuators/booz_actuators_pwm_hw.c
