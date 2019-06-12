# Hey Emacs, this is a -*- makefile -*-
#
# px4fmu_4.0.makefile
#
# Take a look at https://pixhawk.org/modules/pixracer for details

# Board is a Pixracer v1.0 but adhering to px4 way of naming it is px4fmu v4
BOARD=px4fmu
BOARD_VERSION=4.0
#Pixracer has a PX4 FMU v4 core  thus default
BOARD_CFG=\"boards/px4fmu_4.0.h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/px4fmu_4.0.ld

HARD_FLOAT=yes

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/px4fmu_4.0.prototype"
#FIXME Test with more clone boards and add Wildcard
PX4_BL_PORT ?= "/dev/serial/by-id/usb-3D_Robotics_PX4_BL_FMU_v4.x_0-if00"

#
# default LED configuration but all on same LED gives multi colors...
#
RADIO_CONTROL_LED  ?= 3
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (RC receiver, telemetry modem, GPS)
#
SBUS_PORT ?= UART6
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART6
#RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT   ?= UART7

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
GPS_BAUD ?= B57600

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

#
# default External Current and Volt Sensor configuration
#
# ADC_CURRENT_SENSOR ?= ADC_3
# ADC_VOLTAGE_SENSOR ?= ADC_2
