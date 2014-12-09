# Hey Emacs, this is a -*- makefile -*-

# W5100 ethernet chip.

W5100_IP ?= "192,168,25,47"
W5100_SUBNET ?= "255,255,255,0"
W5100_MULTICAST_IP ?= "224,1,1,11"
W5100_MULTICAST_PORT ?= "1234"

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=chip0
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=W5100 -DDefaultPeriodic='&telemetry_Ap'
ap.CFLAGS += -DW5100_IP=$(W5100_IP) -DW5100_SUBNET=$(W5100_SUBNET) -DW5100_MULTICAST_IP=$(W5100_MULTICAST_IP) -DW5100_MULTICAST_PORT=$(W5100_MULTICAST_PORT)
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/w5100.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c

# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

ifeq ($(ARCH), lpc21)
# only an issue of setting the DRDY pin in w5100.c, which is stm32 specific
$(error Not implemented for the LCP21x yet.)
ap.CFLAGS += -DUSE_SPI1
# default SPI device for W5100 is already SPI1
ap.CFLAGS += -DUSE_SPI_SLAVE0
ap.CFLAGS += -DW5100_SLAVE_IDX=0
else ifeq ($(ARCH), stm32)
# on extra SPI1 connector
ap.CFLAGS += -DUSE_SPI1
# default SPI device for W5100 is already SPI1
# Slave select configuration
# SLAVE1 is SS on external SPI1 connector (PA04)
ap.CFLAGS += -DUSE_SPI_SLAVE1
# default slave select for W5100 is already SLAVE1
#ap.CFLAGS += -DW5100_SLAVE_IDX=1
endif
