# Hey Emacs, this is a -*- makefile -*-
#
# Drivers for baros already on an autopilot board


# check that onboard baro was not explicitly disabled with
# <configure name="USE_BARO_BOARD" value="FALSE"/>

USE_BARO_BOARD ?= TRUE
ifeq ($(USE_BARO_BOARD), TRUE)

# booz baro
ifeq ($(BOARD), booz)
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# Lisa/L
else ifeq ($(BOARD), lisa_l)
  BARO_BOARD_CFLAGS += -DUSE_I2C2
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# Navstik baro
else ifeq ($(BOARD), navstik)
  BARO_BOARD_CFLAGS += -DUSE_I2C3
  BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_BOARD_BMP085
  BARO_BOARD_SRCS += peripherals/bmp085.c
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# Ardrone baro
else ifeq ($(BOARD)$(BOARD_TYPE), ardroneraw)
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# Bebop baro
else ifeq ($(BOARD), bebop)
  BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_I2C -DBB_MS5611_SLAVE_ADDR=0x77
  BARO_BOARD_CFLAGS += -DUSE_I2C1
  BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c1
  BARO_BOARD_SRCS += peripherals/ms5611.c
  BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
  BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c

# Lisa/M baro
else ifeq ($(BOARD), lisa_m)
  ifeq ($(BOARD_VERSION), 1.0)
    # on lisa_m_1.0: defaults to i2c baro bmp085 on the board
    LISA_M_BARO ?= BARO_BOARD_BMP085
  else ifeq ($(BOARD_VERSION), 2.0)
    # on lisa_m_2.0: defaults to MS5611 baro connected via SPI on Aspirin2.2
    LISA_M_BARO ?= BARO_MS5611_SPI
  else ifeq ($(BOARD_VERSION), 2.1)
    # on lisa_m_2.1: defaults to MS5611 baro connected via SPI on the integrated Aspirin2.2
    LISA_M_BARO ?= BARO_MS5611_SPI
  endif
  ifeq ($(LISA_M_BARO), BARO_MS5611_SPI)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_SPI
    include $(CFG_SHARED)/spi_master.makefile
    BARO_BOARD_CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    BARO_BOARD_CFLAGS += -DBB_MS5611_SPI_DEV=spi2
    BARO_BOARD_CFLAGS += -DBB_MS5611_SLAVE_IDX=SPI_SLAVE3
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_spi.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_spi.c
  else ifeq ($(LISA_M_BARO), BARO_MS5611_I2C)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_I2C
    BARO_BOARD_CFLAGS += -DUSE_I2C2
    BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c2
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c
  else ifeq ($(LISA_M_BARO), BARO_BOARD_BMP085)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_BOARD_BMP085
    BARO_BOARD_CFLAGS += -DUSE_I2C2
    BARO_BOARD_SRCS += peripherals/bmp085.c
    BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c
  endif

else ifeq ($(BOARD), lisa_mx)
# defaults to MS5611 via SPI on Aspirin
LISA_MX_BARO ?= BARO_MS5611_SPI
  ifeq ($(LISA_MX_BARO), BARO_MS5611_SPI)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_SPI
    include $(CFG_SHARED)/spi_master.makefile
    BARO_BOARD_CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    BARO_BOARD_CFLAGS += -DBB_MS5611_SPI_DEV=spi2
    BARO_BOARD_CFLAGS += -DBB_MS5611_SLAVE_IDX=SPI_SLAVE3
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_spi.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_spi.c
  else ifeq ($(LISA_MX_BARO), BARO_MS5611_I2C)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_I2C
    BARO_BOARD_CFLAGS += -DUSE_I2C2
    BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c2
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c
  else ifeq ($(LISA_MX_BARO), BARO_BOARD_BMP085)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_BOARD_BMP085
    BARO_BOARD_CFLAGS += -DUSE_I2C2
    BARO_BOARD_SRCS += peripherals/bmp085.c
    BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c
  endif

# Lisa/S baro
else ifeq ($(BOARD), lisa_s)
# defaults to SPI baro MS5611 on the board
  include $(CFG_SHARED)/spi_master.makefile
  BARO_BOARD_CFLAGS += -DUSE_SPI1 -DUSE_SPI_SLAVE1
  BARO_BOARD_CFLAGS += -DBB_MS5611_SPI_DEV=spi1
  BARO_BOARD_CFLAGS += -DBB_MS5611_SLAVE_IDX=SPI_SLAVE1
  BARO_BOARD_SRCS += peripherals/ms5611.c
  BARO_BOARD_SRCS += peripherals/ms5611_spi.c
  BARO_BOARD_SRCS += boards/baro_board_ms5611_spi.c

# Lia baro (no bmp onboard)
else ifeq ($(BOARD), lia)
# fixme, reuse the baro drivers in lisa_m dir
LIA_BARO ?= BARO_MS5611_SPI
  ifeq ($(LIA_BARO), BARO_MS5611_SPI)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_SPI
    include $(CFG_SHARED)/spi_master.makefile
    BARO_BOARD_CFLAGS += -DUSE_SPI2 -DUSE_SPI_SLAVE3
    BARO_BOARD_CFLAGS += -DBB_MS5611_SPI_DEV=spi2
    BARO_BOARD_CFLAGS += -DBB_MS5611_SLAVE_IDX=SPI_SLAVE3
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_spi.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_spi.c
  else ifeq ($(LIA_BARO), BARO_MS5611_I2C)
    BARO_BOARD_CFLAGS += -DBARO_BOARD=BARO_MS5611_I2C
    BARO_BOARD_CFLAGS += -DUSE_I2C2
    BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c2
    BARO_BOARD_SRCS += peripherals/ms5611.c
    BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
    BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c
  endif

# navgo baro
else ifeq ($(BOARD), navgo)
  include $(CFG_SHARED)/spi_master.makefile
  BARO_BOARD_CFLAGS += -DUSE_SPI_SLAVE0
  BARO_BOARD_CFLAGS += -DUSE_SPI1
  BARO_BOARD_SRCS += peripherals/mcp355x.c
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# hbmini baro
else ifeq ($(BOARD), hbmini)
  BARO_BOARD_CFLAGS += -DUSE_I2C1
  BARO_BOARD_SRCS += peripherals/bmp085.c
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# krooz baro
else ifeq ($(BOARD), krooz)
  BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c2
  BARO_BOARD_CFLAGS += -DBB_MS5611_SLAVE_ADDR=0xEC
  BARO_BOARD_SRCS += peripherals/ms5611.c
  BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
  BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c

# PX4FMU
else ifeq ($(BOARD), px4fmu)
  BARO_BOARD_CFLAGS += -DUSE_I2C2
  BARO_BOARD_CFLAGS += -DBB_MS5611_I2C_DEV=i2c2
  BARO_BOARD_SRCS += peripherals/ms5611.c
  BARO_BOARD_SRCS += peripherals/ms5611_i2c.c
  BARO_BOARD_SRCS += boards/baro_board_ms5611_i2c.c

# apogee baro
else ifeq ($(BOARD), apogee)
  BARO_BOARD_CFLAGS += -DUSE_I2C1
  BARO_BOARD_SRCS += peripherals/mpl3115.c
  BARO_BOARD_SRCS += $(SRC_BOARD)/baro_board.c

# Umarim
else ifeq ($(BOARD), umarim)
  ifeq ($(BOARD_VERSION), 1.0)
    BARO_BOARD_CFLAGS += -DUSE_I2C1 -DUSE_ADS1114_1
    BARO_BOARD_CFLAGS += -DADS1114_I2C_DEV=i2c1
    BARO_BOARD_SRCS += peripherals/ads1114.c
    BARO_BOARD_SRCS += boards/umarim/baro_board.c
  endif

endif # check board

ifneq ($(BARO_LED),none)
BARO_BOARD_CFLAGS += -DBARO_LED=$(BARO_LED)
endif

# make sure you can also use <configure name="BARO_PERIODIC_FREQUENCY" value="x"/> instead of define
ifdef BARO_PERIODIC_FREQUENCY
BARO_BOARD_CFLAGS += -DBARO_PERIODIC_FREQUENCY=$(BARO_PERIODIC_FREQUENCY)
endif

ap.CFLAGS += $(BARO_BOARD_CFLAGS)
ap.srcs += $(BARO_BOARD_SRCS)

# don't use for NPS
ifeq ($(TARGET),nps)
$(TARGET).CFLAGS += -DUSE_BARO_BOARD=FALSE
endif

else # USE_BARO_BOARD is not TRUE, was explicitly disabled

ap.CFLAGS += -DUSE_BARO_BOARD=FALSE

endif # check USE_BARO_BOARD
