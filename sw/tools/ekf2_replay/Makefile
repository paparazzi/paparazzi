# Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
#
# This file is part of paparazzi.
#
# Build the EKF2 replay tool with the same EKF2 code and features as the
# ins_ekf2 module (sources and CONFIG_EKF2_* defines read from its module file).

Q ?= @
PAPARAZZI_SRC ?= $(abspath ../../..)
EKF2 = $(PAPARAZZI_SRC)/sw/ext/px4_ekf2
MODULE = $(PAPARAZZI_SRC)/conf/modules/ins_ekf2.xml

EKF2_SRCS = $(shell sed -n 's|.*<file name="\([^"]*\)" dir="px4_ekf2/\([^"]*\)".*|$(EKF2)/\2/\1|p' $(MODULE))
EKF2_DEFINES = $(shell sed -n 's|.*<define name="\(CONFIG_EKF2_[A-Z_]*\)"/>.*|-D\1|p' $(MODULE))

CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -g -Wall -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable \
	-fno-associative-math -fno-strict-aliasing \
	-D__PAPARAZZI -DMODULE_NAME $(EKF2_DEFINES) \
	-I$(EKF2) -I$(EKF2)/lib -I$(EKF2)/lib/matrix -I$(EKF2)/EKF -I$(EKF2)/EKF/python \
	-I$(EKF2)/generated -I$(EKF2)/stubs

BUILD = build
OBJS = $(BUILD)/ekf2_replay.o $(patsubst $(EKF2)/%.cpp,$(BUILD)/px4_ekf2/%.o,$(EKF2_SRCS))

all: ekf2_replay

ekf2_replay: $(OBJS)
	@echo LD $@
	$(Q)$(CXX) -o $@ $^ -lm

$(BUILD)/ekf2_replay.o: ekf2_replay.cpp $(EKF2)/generated/ekf2_params.h
	@mkdir -p $(dir $@)
	@echo CXX $<
	$(Q)$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD)/px4_ekf2/%.o: $(EKF2)/%.cpp
	@mkdir -p $(dir $@)
	@echo CXX $(notdir $<)
	$(Q)$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD) ekf2_replay

.PHONY: all clean
