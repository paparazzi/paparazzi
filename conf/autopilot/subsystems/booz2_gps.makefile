
ap.srcs += $(SRC_BOOZ)/booz2_gps.c
ap.CFLAGS += -DUSE_UART0 -DUART0_BAUD=B38400 -DUART0_VIC_SLOT=5
ap.CFLAGS += -DUSE_GPS -DGPS_LINK=Uart0 -DGPS_LED=4


sim.CFLAGS += -DUSE_GPS
sim.srcs += $(SRC_BOOZ)/booz2_gps.c
