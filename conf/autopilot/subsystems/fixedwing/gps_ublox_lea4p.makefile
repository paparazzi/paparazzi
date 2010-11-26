# UBlox LEA 4P


ap.CFLAGS += -DUSE_GPS -DUBX
ap.CFLAGS += -DGPS_LINK=Uart$(GPS_UART_NR)
ap.CFLAGS += -DUSE_UART$(GPS_UART_NR)
ap.CFLAGS += -DUART$(GPS_UART_NR)_BAUD=$(GPS_BAUD)

ifneq ($(GPS_LED),none)
  ap.CFLAGS += -DGPS_LED=$(GPS_LED)
endif

ap.srcs   += $(SRC_FIXEDWING)/gps_ubx.c

$(TARGET).srcs += $(SRC_FIXEDWING)/gps.c $(SRC_FIXEDWING)/latlong.c

