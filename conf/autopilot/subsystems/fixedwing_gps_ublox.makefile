


ifeq ($(GPS_UART), Uart0)
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B$(GPS_BAUD)
else
ap.CFLAGS += -DUSE_UART1 -DUART1_BAUD=B$(GPS_BAUD)
endif


ap.CFLAGS += -DGPS -DUBX -DGPS_LINK=$(GPS_UART)
ap.srcs += $(SRC_FIXEDWING)/gps_ubx.c $(SRC_FIXEDWING)/gps.c $(SRC_FIXEDWING)/latlong.c

