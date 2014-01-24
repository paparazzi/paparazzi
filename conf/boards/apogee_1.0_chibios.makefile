# Hey Emacs, this is a -*- makefile -*-
#
# apogee_1.0_chibios.makefile
#
#

BOARD=apogee
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
RTOS=chibios-libopencm3
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/stm32f4_chibios.ld

HARD_FLOAT=yes

# include Makefile.chibios-libopencm3 instead of Makefile.stm32
$(TARGET).MAKEFILE = chibios-libopencm3

# default flash mode is via usb dfu bootloader
# possibilities: DFU, SWD
FLASH_MODE ?= DFU
STLINK ?= y
DFU_UTIL ?= y

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART2

SBUS_PORT ?= UART2

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
