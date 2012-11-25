# Hey Emacs, this is a -*- makefile -*-
#
# lisa_l_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/User/LisaL
#


# we are actually still using the Lisa/L 1.0 header file

BOARD=lisa_l
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-l.ld
NO_LUFTBOOT=1

# -----------------------------------------------------------------------
ifeq ($(BOARD_PROCESSOR),'omap')

	ARCH  = omap
	$(TARGET).LDFLAGS += -levent -lm

# -----------------------------------------------------------------------
else

	ARCH=stm32

	$(TARGET).ARCHDIR = $(ARCH)
# not needed?

endif
# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = JTAG
endif

#
#
# some default values shared between different firmwares
#
#

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 5
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 7
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1


#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400




#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
#
ifndef ACTUATORS
ACTUATORS = actuators_pwm
endif


ifndef ADC_IR1
ADC_IR1      = 1
ADC_IR1_CHAN = 0
endif
ifndef ADC_IR2
ADC_IR2      = 2
ADC_IR2_CHAN = 1
endif
ifndef ADC_IR3
ADC_IR_TOP      = 4
ADC_IR_TOP_CHAN = 3
endif
ifndef ADC_IR_NB_SAMPLES
ADC_IR_NB_SAMPLES = 16
endif
