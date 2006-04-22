ap.srcs += main_ap.c sys_time.c main.c inter_mcu.c link_mcu.c gps_ubx.c gps.c infrared.c pid.c nav.c estimator.c cam.c downlink.c
ap.CFLAGS += -DMCU_SPI_LINK -DGPS -DUBX -DINFRARED -DRADIO_CONTROL -DINTER_MCU

fbw.srcs +=  sys_time.c main_fbw.c main.c commands.c radio_control.c pprz_transport.c downlink.c  inter_mcu.c
fbw.CFLAGS += -DRADIO_CONTROL -DDOWNLINK -DUSE_UART0 -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_FBW_DEVICE=Uart0 -DINTER_MCU -DMCU_SPI_LINK

