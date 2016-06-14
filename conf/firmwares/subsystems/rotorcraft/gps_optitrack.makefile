include $(CFG_ROTORCRAFT)/gps_datalink.makefile
# Always zero to get heading update
ap.CFLAGS += -DAHRS_HEADING_UPDATE_GPS_MIN_SPEED=0
