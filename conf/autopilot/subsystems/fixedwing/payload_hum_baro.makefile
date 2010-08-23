# Payload: Sensirion humidity/temp, VTI pressure/temp

ap.srcs += humid_sht.c
ap.CFLAGS += -DUSE_HUMID_SHT -DDAT_PIN=3  -DSCK_PIN=2

ap.srcs += baro_scp.c
ap.CFLAGS += -DUSE_BARO_SCP
