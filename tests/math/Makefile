# Copyright (C) 2014 Piotr Esden-Tempski
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
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.

# The default is to produce a quiet echo of compilation commands
# Launch with "make Q=''" to get full echo

# Make sure all our environment is set properly in case we run make not from toplevel director.
Q ?= @

PAPARAZZI_SRC ?= $(shell pwd)/../..
ifeq ($(PAPARAZZI_HOME),)
PAPARAZZI_HOME=$(PAPARAZZI_SRC)
endif

# export the PAPARAZZI environment to sub-make
export PAPARAZZI_SRC
export PAPARAZZI_HOME

MATHSRC_PATH=$(PAPARAZZI_SRC)/sw/airborne/math
MATHLIB_PATH=$(PAPARAZZI_SRC)/var/build/math

#####################################################
# If you add more test files you add their names here
TESTS = test_pprz_math.run test_pprz_geodetic.run test_state_interface.run

###################################################
# You should not need to touch the rest of the file

TEST_VERBOSE ?= 0
ifneq ($(TEST_VERBOSE), 0)
VERBOSE = --verbose
endif

all: test

math_shlib:
	$(Q)cd $(MATHSRC_PATH); make shared_lib

build_tests: math_shlib $(TESTS)

test: build_tests
	LD_LIBRARY_PATH=$(MATHLIB_PATH):$LD_LIBRARY_PATH prove $(VERBOSE) --exec '' ./*.run

# test_state_interface also depends on state.c
test_state_interface.run: $(PAPARAZZI_SRC)/sw/airborne/state.c

%.run: %.c | math_shlib
	@echo BUILD $@
	$(Q)$(CC) -L$(MATHLIB_PATH) -I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/sw/include $(USER_CFLAGS) tap.c $^ -lpprzmath -o $@

clean:
	$(Q)rm -f $(MATHLIB_PATH)/*.o $(MATHLIB_PATH)/libpprzmath.so
	$(Q)rm -f $(TESTS)


.PHONY: math_shlib build_tests test clean all
