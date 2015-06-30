#
# The superbitRF module as telemetry downlink/uplink
#
#

$(TARGET).CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=superbitrf
$(TARGET).CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=SUPERBITRF

$(TARGET).srcs += peripherals/cyrf6936.c
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/superbitrf.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
