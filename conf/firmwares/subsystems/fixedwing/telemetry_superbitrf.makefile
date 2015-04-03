#
# The superbitRF module as telemetry downlink/uplink
#
#
ap.CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_FBW_DEVICE=superbitrf -DDOWNLINK_AP_DEVICE=superbitrf
ap.CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=SUPERBITRF
#ap.CFLAGS += -DUSE_SUPERBITRF -DUSE_SPI2 -DUSE_SPI_SLAVE2

ap.srcs += peripherals/cyrf6936.c
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/superbitrf.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/ap_downlink.c

# avoid fbw_telemetry_mode error
ap.srcs += $(SRC_FIRMWARE)/fbw_downlink.c

fbw.srcs += $(SRC_FIRMWARE)/fbw_downlink.c
