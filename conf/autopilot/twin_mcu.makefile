ap.srcs += $(SRC_FIRMWARE)/main_ap.c sys_time.c $(SRC_FIRMWARE)/main.c inter_mcu.c link_mcu.c gps_ubx.c gps.c infrared.c fw_h_ctl.c fw_v_ctl.c nav.c estimator.c cam.c spi.c rc_settings.c latlong.c nav_survey_rectangle.c
ap.CFLAGS += -DMCU_SPI_LINK -DUSE_GPS -DUBX -DUSE_INFRARED -DRADIO_CONTROL -DINTER_MCU -DSPI_MASTER -DUSE_SPI -DNAV -DRADIO_CONTROL_SETTINGS

fbw.srcs +=  sys_time.c $(SRC_FIRMWARE)/main_fbw.c $(SRC_FIRMWARE)/main.c commands.c radio_control.c pprz_transport.c downlink.c  inter_mcu.c spi.c link_mcu.c
fbw.CFLAGS += -DRADIO_CONTROL -DDOWNLINK -DUSE_UART0 -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_FBW_DEVICE=Uart0 -DINTER_MCU -DMCU_SPI_LINK -DUART0_BAUD=B38400 -DSPI_SLAVE -DUSE_SPI

