# Hey Emacs, this is a -*- makefile -*-
#
# crazybee_f4_1.0.makefile
#
# Take a look at https://www.openuas.org/  airframes for example

# Board is a crazybee F4 v1.0
BOARD=crazybee_f4
BOARD_VERSION=1.0
BOARD_CFG=\"boards/crazybee_f4_1.0.h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/crazybee_f4_1.0.ld 

HARD_FLOAT=yes

# Default flash mode is the STM32 DFU bootloader
# Theoreticlly possible are also SWD and JTAG_BMP
# But no simple physical connectors to the board...
# So... DFU it will be ...
FLASH_MODE?=DFU-UTIL

#idVendor=0483, idProduct=5740
#USB device strings: Mfr=1, Product=2, SerialNumber=3
#Product: Product: CrazyBee F4 (x)
#Manufacturer: Betaflight
#SerialNumber: 0x8000000

#TIP: ttyACM0: USB ACM device

#
# Default on PCB LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# Default UART configuration (RC receiver, telemetry modem, GPS)
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART2
RADIO_CONTROL_SBUS_PORT   ?= UART2

MODEM_PORT ?= UART1
MODEM_BAUD ?= B115200

#
# GPS via I2C just as Baro and Magneto... sparec amount of uart ports left on this board
# If one starts using a build in RX on SPI Bus then TX1/RX1 can be used.
#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# and by setting the correct "driver" attribute in servo section
ACTUATORS ?= actuators_pwm
