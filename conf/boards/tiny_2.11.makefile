#
# tiny_2.11.makefile
#
# http://paparazzi.enac.fr/wiki/Tiny_v2
#
# TODO: move all to new directories
# ARCH=lpc21
ARCH=arm7
ARCHI=arm7
BOARD_CFG = \"tiny_2_1_1.h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif


ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = arm7tdmi
fbw.TARGET = autopilot
fbw.TARGETDIR = autopilot

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


### default settings for tiny and twog
GPS_UART_NR	= 0
GPS_BAUD	= 38400
MODEM_UART_NR = 1
MODEM_BAUD = 57600

ADC_IR1 = ADC_1
ADC_IR2 = ADC_2
ADC_IR_TOP = ADC_0
ADC_IR_NB_SAMPLES = 16
ADC_GYRO_ROLL = ADC_3
ADC_GYRO_NB_SAMPLES = 16


