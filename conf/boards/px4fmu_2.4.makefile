# Hey Emacs, this is a -*- makefile -*-
#
# px4fmu_2.4.makefile
#
# This is for the main MCU (STM32F427) on the PX4 board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=px4fmu
BOARD_VERSION=2.4
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/px4fmu_2.4.ld

HARD_FLOAT=yes

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/px4fmu-v2.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/pci-3D_Robotics*"


#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

#The modem serial on px4 is called serial 1, but connected to uart2 on the f4
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

#The GPS serial on px4 is called serial 3, but connected to uart4 on the f4
GPS_PORT ?= UART4
GPS_BAUD ?= B38400

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
