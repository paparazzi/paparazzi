
ap.srcs += $(SRC_BOOZ)/booz2_gps.c math/pprz_geodetic_int.c math/pprz_geodetic_float.c
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400 -DUART0_VIC_SLOT=5
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=Uart0 -DGPS_LED=3
