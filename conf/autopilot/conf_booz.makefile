ARCHI=arm7

FLASH_MODE = IAP


#
# filter CPU
#

flt.ARCHDIR = $(ARCHI)
flt.ARCH = arm7tdmi
flt.TARGET = flt
flt.TARGETDIR = flt

flt.CFLAGS += -DBOOZ_FILTER_MCU -DCONFIG=\"pprz_imu.h\"  -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(4e-3)'
flt.srcs = booz_filter_main.c sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c

flt.CFLAGS += -DLED

flt.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
flt.srcs += $(SRC_ARCH)/uart_hw.c

flt.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
flt.srcs += downlink.c pprz_transport.c booz_filter_telemetry.c

flt.CFLAGS += -DADC -DUSE_AD0 -DUSE_AD0_1 -DUSE_AD0_2 -DUSE_AD0_3 -DUSE_AD0_4
flt.srcs += $(SRC_ARCH)/adc_hw.c

flt.srcs += max1167.c $(SRC_ARCH)/max1167_hw.c

flt.CFLAGS += -DDISABLE_MAGNETOMETER
flt.srcs += micromag.c $(SRC_ARCH)/micromag_hw.c
flt.CFLAGS += -I2C_BUF_LEN=32 -DI2C_SCLL=150 -DI2C_SCLH=150 -DI2C_VIC_SLOT=10
flt.srcs += AMI601.c i2c.c $(SRC_ARCH)/i2c_hw.c
flt.srcs += imu_v3.c $(SRC_ARCH)/imu_v3_hw.c

flt.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_MULTITILT
flt.srcs += multitilt.c
flt.srcs += booz_ahrs.c

flt.srcs += booz_inter_mcu.c
flt.srcs += booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c


#
# controller CPU
#

ctl.ARCHDIR = $(ARCHI)
ctl.ARCH = arm7tdmi
ctl.TARGET = ctl
ctl.TARGETDIR = ctl

ctl.CFLAGS += -DBOOZ_CONTROLLER_MCU -DCONFIG=\"conf_booz.h\"
ctl.srcs = booz_controller_main.c

ctl.CFLAGS += -DLED

ctl.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(4e-3)' -DTIME_LED=1
ctl.srcs += sys_time.c $(SRC_ARCH)/sys_time_hw.c

ctl.srcs += $(SRC_ARCH)/armVIC.c

ctl.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B57600
ctl.srcs += $(SRC_ARCH)/uart_hw.c

ctl.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=Uart1 
ctl.srcs += booz_controller_telemetry.c downlink.c pprz_transport.c 

ctl.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
ctl.srcs += booz_datalink.c

#ctl.CFLAGS += -DACTUATORS=\"servos_4017_hw.h\" -DSERVOS_4017 -DSERVOS_4017_CLOCK_FALLING
#ctl.srcs += $(SRC_ARCH)/servos_4017_hw.c actuators.c

ctl.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
ctl.srcs += $(SRC_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
ctl.CFLAGS += -DI2C_SCLL=150 -DI2C_SCLH=150 -DI2C_VIC_SLOT=10
ctl.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c

ctl.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=4
ctl.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

ctl.srcs += booz_inter_mcu.c
ctl.CFLAGS += -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0
ctl.CFLAGS += -DSPI_SELECT_SLAVE0_PIN=20 -DSPI_SELECT_SLAVE0_PORT=0 -DSSPCPSR_VAL=0x10
ctl.srcs += booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c
ctl.srcs += spi.c $(SRC_ARCH)/spi_hw.c

ctl.srcs += commands.c
ctl.CFLAGS += -DDISABLE_NAV 
ctl.srcs += booz_estimator.c      \
            booz_control.c        \
            booz_nav.c  	  \
            booz_nav_hover.c  	  \
            booz_autopilot.c

#
# SITL Simulator
#

SIM_TYPE = BOOZ

sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = main
sim.TARGETDIR = main

sim.srcs = $(SIMDIR)/main_booz_sim.c           \
	   $(SIMDIR)/booz_flight_model.c       \
           $(SIMDIR)/booz_flight_model_utils.c \
           $(SIMDIR)/booz_sensors_model.c      \
           $(SIMDIR)/booz_flightgear.c         \
           $(SIMDIR)/booz_joystick.c           \

sim.CFLAGS += -DSITL
sim.CFLAGS += -DBOOZ_CONTROLLER_MCU
sim.CFLAGS += -DCONFIG=\"conf_booz.h\"

sim.srcs += booz_controller_main.c

sim.srcs += sys_time.c

sim.CFLAGS += -DRADIO_CONTROL 
sim.srcs += radio_control.c \
	    $(SRC_ARCH)/ppm_hw.c

sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" 
sim.srcs += actuators.c \
            $(SRC_ARCH)/actuators_buss_twi_blmc_hw.c \
            i2c.c $(SRC_ARCH)/i2c_hw.c

sim.CFLAGS += -DDOWNLINK 
sim.srcs += booz_controller_telemetry.c \
            downlink.c
sim.CFLAGS += -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs += $(SRC_ARCH)/ivy_transport.c

sim.srcs += booz_inter_mcu.c
sim.CFLAGS += -DUSE_SPI
sim.srcs += booz_link_mcu.c $(SRC_ARCH)/booz_link_mcu_hw.c
sim.srcs += spi.c $(SRC_ARCH)/spi_hw.c

sim.srcs += booz_estimator.c
sim.srcs += booz_control.c
sim.srcs += booz_nav.c
sim.srcs += booz_nav_hover.c

sim.srcs += booz_autopilot.c
sim.srcs += commands.c


sim.CFLAGS += -DBOOZ_FILTER_MCU
sim.srcs += booz_filter_main.c

sim.CFLAGS += -DADC_CHANNEL_AX=1 -DADC_CHANNEL_AY=2 -DADC_CHANNEL_AZ=3 -DADC_CHANNEL_BAT=4
sim.srcs  += $(SRC_ARCH)/adc_hw.c


sim.srcs += booz_filter_telemetry.c

sim.srcs  += max1167.c $(SRC_ARCH)/max1167_hw.c
sim.srcs  += micromag.c $(SRC_ARCH)/micromag_hw.c
sim.srcs  += imu_v3.c $(SRC_ARCH)/imu_v3_hw.c


sim.CFLAGS += -DFLOAT_T=float
sim.srcs += booz_ahrs.c
sim.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_MULTITILT
sim.srcs += multitilt.c
#sim.CFLAGS += -DBOOZ_AHRS_TYPE=BOOZ_AHRS_QUATERNION 
#sim.CFLAGS += -DEKF_UPDATE_DISCRETE
#sim.CFLAGS += -DEKF_PREDICT_ONLY
#sim.srcs += ahrs_quat_fast_ekf.c 