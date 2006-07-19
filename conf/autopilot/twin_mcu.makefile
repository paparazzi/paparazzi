ap.srcs += main_ap.c sys_time.c main.c inter_mcu.c link_mcu.c gps_ubx.c gps.c infrared.c pid.c nav.c estimator.c cam.c spi.c
ap.CFLAGS += -DMCU_SPI_LINK -DGPS -DUBX -DINFRARED -DRADIO_CONTROL -DINTER_MCU -DSPI_MASTER -DUSE_SPI -DNAV

fbw.srcs +=  sys_time.c main_fbw.c main.c commands.c radio_control.c pprz_transport.c downlink.c  inter_mcu.c spi.c link_mcu.c
fbw.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DDOWNLINK -DUSE_UART0 -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_FBW_DEVICE=Uart0 -DINTER_MCU -DMCU_SPI_LINK -DUART0_BAUD=B38400 -DSPI_SLAVE -DUSE_SPI

