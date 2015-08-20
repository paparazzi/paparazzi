#
# The bluegiga module as telemetry downlink/uplink
#
#

# Include SPI if not yet included
include $(CFG_SHARED)/spi_master.makefile

# Set downlink to paparazzi transport over bluegiga protocol over SPI slave
ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=bluegiga_p
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=BLUEGIGA

BLUEGIGA_SPI_DEV ?= SPI2
BLUEGIGA_SPI_DEV_LOWER=$(shell echo $(BLUEGIGA_SPI_DEV) | tr A-Z a-z)
ap.CFLAGS += -DUSE_$(BLUEGIGA_SPI_DEV)_SLAVE -DSPI_SLAVE -DBLUEGIGA_SPI_DEV=$(BLUEGIGA_SPI_DEV_LOWER)

# LED
MODEM_LED ?= none
ifneq ($(MODEM_LED),none)
ap.CFLAGS += -DMODEM_LED=$(MODEM_LED)
endif

ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/bluegiga.c
ap.srcs += subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
