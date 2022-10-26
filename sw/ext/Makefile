# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2012 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# The default is to produce a quiet echo of compilation commands
# Launch with "make Q=''" to get full echo

Q=@

PAPARAZZI_SRC=../..
EXT_DIR=$(PAPARAZZI_SRC)/sw/ext

#
# find compiler toolchain
#
include $(PAPARAZZI_SRC)/conf/Makefile.arm-embedded-toolchain


all: libopencm3 luftboot chibios fatfs libsbp TRICAL hacl-c key_generator rustlink ecl matrix

# update (and init if needed) all submodules
update_submodules:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule update --init --recursive; \
	fi

# update (and init if needed) a specific submodule
%.update:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule update --init --recursive sw/ext/$*; \
	fi

# sync a specific submodule
%.sync:
	$(Q)if [ -d $(PAPARAZZI_SRC)/.git ]; then \
		cd $(PAPARAZZI_SRC) && git submodule sync --recursive sw/ext/$*; \
	fi

# only build currentl checkout of libopencm3
libopencm3.build:
	$(Q)$(MAKE) -C libopencm3 lib PREFIX=$(PREFIX) TARGETS="stm32/f1"
	$(Q)$(MAKE) -C libopencm3 lib PREFIX=$(PREFIX) FP_FLAGS="-mfloat-abi=softfp -mfpu=fpv4-sp-d16" TARGETS="stm32/f4"

# update and then build libopencm3
libopencm3: libopencm3.update libopencm3.build

# only build current checkout of luftboot
luftboot.build: libopencm3.build
	$(Q)$(MAKE) -C luftboot/src all LIBOPENCM3=../../libopencm3 PREFIX=$(PREFIX)

# update libopencm3 and luftboot, then build it
luftboot: libopencm3.update luftboot.update luftboot.build

luftboot_flash: luftboot.build
	$(Q)$(MAKE) -C luftboot/src flash BMP_PORT?=/dev/ttyACM0 LIBOPENCM3=../../libopencm3 PREFIX=$(PREFIX)

hacl-c: hacl-c.update

key_generator: key_generator.update

rustlink: rustlink.update

chibios: chibios.sync chibios.update

TRICAL: TRICAL.update

fatfs: fatfs.sync fatfs.update

ecl: ecl.update

matrix: matrix.update

mavlink: mavlink.update mavlink.build

mavlink.build:
	@echo GENERATE $(PAPARAZZI_SRC)/var/include/mavlink
	$(Q)PYTHONPATH=$(EXT_DIR)/mavlink python $(EXT_DIR)/mavlink/pymavlink/tools/mavgen.py --output $(PAPARAZZI_SRC)/var/include/mavlink --lang C $(EXT_DIR)/mavlink/message_definitions/v1.0/paparazzi.xml --no-validate > /dev/null

libsbp: libsbp.update

pprzlink: pprzlink.update pprzlink.build

pprzlink.build:
	$(Q)$(MAKE) -C pprzlink

opencv_bebop: opencv_bebop.update opencv_bebop.build

opencv_bebop.build:
	$(Q)$(MAKE) -C opencv_bebop

clean:
	$(Q)if [ -f libopencm3/Makefile ]; then \
		$(MAKE) -C $(EXT_DIR)/libopencm3 clean; \
	fi
	$(Q)if [ -f luftboot/src/Makefile ]; then \
		$(MAKE) -C luftboot/src clean; \
	fi

clean_opencv_bebop:
	$(Q)if [ -f opencv_bebop/Makefile ]; then \
		$(MAKE) -C opencv_bebop clean; \
	fi

.NOTPARALLEL: libopencm3 luftboot
.PHONY: all clean update_submodules libopencm3 luftboot chibios fatfs luftboot_flash libopencm3.build luftboot.build mavlink.build libsbp pprzlink pprzlink.build opencv_bebop opencv_bebop.build clean_opencv_bebop TRICAL
