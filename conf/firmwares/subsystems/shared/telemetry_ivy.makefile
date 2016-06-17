$(TARGET).CFLAGS += -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_TRANSPORT=ivy_tp -DDOWNLINK_DEVICE=ivy_tp
$(TARGET).srcs += pprzlink/src/ivy_transport.c
$(TARGET).srcs += subsystems/datalink/downlink.c subsystems/datalink/datalink.c subsystems/datalink/telemetry.c
