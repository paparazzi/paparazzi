#
# The bluegiga module as telemetry downlink/uplink
#
#
ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=bluegiga_p
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=BLUEGIGA

BLUEGIGA_PORT ?= SPI2
BLUEGIGA_PORT_LOWER=$(shell echo $(BLUEGIGA_PORT) | tr A-Z a-z)
ap.CFLAGS += -DUSE_$(BLUEGIGA_PORT)_SLAVE -DSPI_SLAVE -DBLUEGIGA_SPI_DEV=$(BLUEGIGA_PORT_LOWER)

MODEM_LED ?= none
ap.CFLAGS += -DMODEM_LED=$(MODEM_LED)

ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/bluegiga.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
