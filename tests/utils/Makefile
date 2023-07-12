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

#####################################################
# If you add more test files you add their names here
TESTS = test_circular_buffer.run

###################################################
# You should not need to touch the rest of the file

TEST_VERBOSE ?= 0
ifneq ($(TEST_VERBOSE), 0)
VERBOSE = --verbose
endif

all: test

build_tests: $(TESTS)

test: build_tests
	prove $(VERBOSE) --exec '' ./*.run

test_circular_buffer.run: $(PAPARAZZI_SRC)/sw/airborne/utils/circular_buffer.c

%.run: %.c
	@echo BUILD $@
	$(Q)$(CC) -I$(PAPARAZZI_SRC)/sw/airborne/utils -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/tests/common $(USER_CFLAGS) $(PAPARAZZI_SRC)/tests/common/tap.c $^ -o $@

clean:
	$(Q)rm -f $(TESTS)


.PHONY: build_tests test clean all
