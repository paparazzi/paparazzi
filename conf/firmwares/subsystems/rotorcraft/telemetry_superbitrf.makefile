#
# The superbitRF module as telemetry downlink/uplink
#
#
ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=SuperbitRF
ap.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=SUPERBITRF
#ap.CFLAGS += -DUSE_SUPERBITRF -DUSE_SPI2 -DUSE_SPI_SLAVE2

ap.srcs += peripherals/cyrf6936.c
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/superbitrf.c subsystems/datalink/pprz_transport.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/telemetry.c
