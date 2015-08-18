#
# The bluegiga module as telemetry downlink/uplink
#
#
ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=bluegiga_p
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=BLUEGIGA
#ap.CFLAGS += -DDefaultPeriodic='&telemetry_Main'
ap.CFLAGS += -DUSE_SPI2_SLAVE -DSPI_SLAVE

# for some reason the following line required to get GPS over datalink working
ap.CFLAGS += -DUSE_UART3 -DUART3_BAUD=B38400

MODEM_LED ?= none
ap.CFLAGS += -DMODEM_LED=$(MODEM_LED)

ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/bluegiga.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
