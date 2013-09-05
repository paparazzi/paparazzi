# Hey Emacs, this is a -*- makefile -*-
#
# Drivers for baros already on an autopilot board


# booz baro
ifeq ($(BOARD), booz)
  ap.srcs += $(SRC_BOARD)/baro_board.c

# Lisa/L
else ifeq ($(BOARD), lisa_l)
  ap.CFLAGS += -DUSE_I2C2
  ap.srcs += $(SRC_BOARD)/baro_board.c

# Ardrone baro
else ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
  ap.srcs += $(SRC_BOARD)/baro_board.c

# Lisa/M baro
else ifeq ($(BOARD), lisa_m)
# defaults to i2c baro bmp085 on the board
LISA_M_BARO ?= BARO_BOARD_BMP085
  ifeq ($(LISA_M_BARO), BARO_MS5611_SPI)
    include $(CFG_SHARED)/spi_master.makefile
    ap.CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    ap.srcs += peripherals/ms5611.c
    ap.srcs += peripherals/ms5611_spi.c
    ap.srcs += subsystems/sensors/baro_ms5611_spi.c
  else ifeq ($(LISA_M_BARO), BARO_MS5611_I2C)
    ap.CFLAGS += -DUSE_I2C2
    ap.srcs += peripherals/ms5611.c
    ap.srcs += peripherals/ms5611_i2c.c
    ap.srcs += subsystems/sensors/baro_ms5611_i2c.c
  else ifeq ($(LISA_M_BARO), BARO_BOARD_BMP085)
	ap.srcs += peripherals/bmp085.c
    ap.srcs += $(SRC_BOARD)/baro_board.c
    ap.CFLAGS += -DUSE_I2C2
  endif
  ap.CFLAGS += -D$(LISA_M_BARO)

# Lisa/S baro
else ifeq ($(BOARD), lisa_s)
# defaults to SPI baro MS5611 on the board
  include $(CFG_SHARED)/spi_master.makefile
  ap.CFLAGS += -DUSE_SPI1 -DUSE_SPI_SLAVE1
  ap.CFLAGS += -DMS5611_SPI_DEV=spi1
  ap.CFLAGS += -DMS5611_SLAVE_DEV=SPI_SLAVE1
  ap.srcs += peripherals/ms5611.c
  ap.srcs += peripherals/ms5611_spi.c
  ap.srcs += subsystems/sensors/baro_ms5611_spi.c
  ap.CFLAGS += -DBARO_MS5611_SPI

# Lia baro (no bmp onboard)
else ifeq ($(BOARD), lia)
# fixme, reuse the baro drivers in lisa_m dir
LIA_BARO ?= BARO_MS5611_SPI
  ifeq ($(LIA_BARO), BARO_MS5611_SPI)
    include $(CFG_SHARED)/spi_master.makefile
    ap.CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    ap.srcs += peripherals/ms5611.c
    ap.srcs += peripherals/ms5611_spi.c
    ap.srcs += subsystems/sensors/baro_ms5611_spi.c
  else ifeq ($(LIA_BARO), BARO_MS5611_I2C)
    ap.CFLAGS += -DUSE_I2C2
    ap.srcs += peripherals/ms5611.c
    ap.srcs += peripherals/ms5611_i2c.c
    ap.srcs += subsystems/sensors/baro_ms5611_i2c.c
  endif
  ap.CFLAGS += -D$(LIA_BARO)

# navgo baro
else ifeq ($(BOARD), navgo)
  include $(CFG_SHARED)/spi_master.makefile
  ap.CFLAGS += -DUSE_SPI_SLAVE0
  ap.CFLAGS += -DUSE_SPI1
  ap.srcs += peripherals/mcp355x.c
  ap.srcs += $(SRC_BOARD)/baro_board.c

# krooz baro
else ifeq ($(BOARD), krooz)
  ap.CFLAGS += -DMS5611_I2C_DEV=i2c2 -DMS5611_SLAVE_ADDR=0xEC
  ap.srcs += peripherals/ms5611.c
  ap.srcs += peripherals/ms5611_i2c.c
  ap.srcs += subsystems/sensors/baro_ms5611_i2c.c

# PX4FMU
else ifeq ($(BOARD), px4fmu)
  ap.CFLAGS += -DUSE_I2C2 -DMS5611_I2C_DEV=i2c2
  ap.srcs += peripherals/ms5611.c
  ap.srcs += peripherals/ms5611_i2c.c
  ap.srcs += subsystems/sensors/baro_ms5611_i2c.c

# apogee baro
else ifeq ($(BOARD), apogee)
  ap.CFLAGS += -DUSE_I2C1
  ap.CFLAGS += -DMPL3115_I2C_DEV=i2c1
  ap.CFLAGS += -DMPL3115_ALT_MODE=0
  ap.srcs += peripherals/mpl3115.c
  ap.srcs += $(SRC_BOARD)/baro_board.c

# Umarim
else ifeq ($(BOARD), umarim)
  ifeq ($(BOARD_VERSION), 1.0)
    ap.srcs 	+= boards/umarim/baro_board.c
    ap.CFLAGS += -DUSE_I2C1 -DUSE_ADS1114_1
    ap.CFLAGS += -DADS1114_I2C_DEV=i2c1
    ap.srcs 	+= peripherals/ads1114.c
  endif

endif # End baro

ifneq ($(BARO_LED),none)
ap.CFLAGS += -DBARO_LED=$(BARO_LED)
endif
