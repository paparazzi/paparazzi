
# Wifi telemetry

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=Wifi
ap.CFLAGS += -DDOWNLINK_TRANSPORT=WifiTransport -DDATALINK=WIFI
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/wifi.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/telemetry.c
ap.srcs += fms/fms_network.c
