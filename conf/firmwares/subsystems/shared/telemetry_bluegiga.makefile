#
# The bluegiga module as telemetry downlink/uplink
#
# On Lisa-s, the Superbit-RF module can be replaced with a
# bluegiga-bluetooth 4 low power board. This subsystem puts
# the telemetry over this bluetooth 4 link.
#
# Bluegiga is a SPI-slave device without Chip select but with a compulsory DRDY
#
# Required:
# #define BLUEGIGA_SPI_DEV: the SPI device which MUST have a SLAVE mode without chipselect
#
# Optional: (defaults to SUPERBITRF DRDY)
# #define BLUEGIGA_DRDY_GPIO
# #define BLUEGIGA_DRDY_GPIO_PIN

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

ap.srcs += $(SRC_SUBSYSTEMS)/datalink/downlink.c subsystems/datalink/datalink.c $(SRC_SUBSYSTEMS)/datalink/bluegiga.c
ap.srcs += pprzlink/src/pprz_transport.c $(SRC_SUBSYSTEMS)/datalink/telemetry.c
