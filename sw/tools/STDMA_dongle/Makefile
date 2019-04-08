TARGET = bluetooth-proximity

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)
DEPS := $(SRCS:.c=.d)

EMBED ?=

ifeq ($(EMBED),)
CC = gcc

MACHINE = $(shell $(CC) -dumpmachine)
# Windows
ifneq (,$(or $(findstring mingw, $(MACHINE)), $(findstring cygwin, $(MACHINE))))
	PLATFORM = WIN
	LIBS = -lm -lsetupapi
	RM = del
# POSIX
else
	PLATFORM = POSIX
	LIBS = -lm
endif

all: $(TARGET)

%.o: %.c
	$(CC) -O3 -Wall -c -fmessage-length=0 -DPLATFORM_$(PLATFORM) -MMD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<" -DPRINT

$(TARGET): $(OBJS)
	@echo 'Building target: $@'
	$(CC) -o"$(TARGET)" $(OBJS) $(LIBS) $(CFLAG)
	@echo 'Finished building target: $@'

clean:
	-$(RM) $(OBJS) $(DEPS) $(TARGET)

.PHONY: all clean

else

CROSS_COMPILE   ?= arm-linux-gnueabi-
CC              := $(CROSS_COMPILE)gcc
CFLAGS          := -O3 -Wall -Wextra -Wno-unused-parameter -Wmissing-prototypes
LDFLAGS         := -Wl,--no-undefined -Wl,--as-needed

all: $(TARGET)

%.o: %.c
	$(CC) -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"

$(TARGET): $(OBJS)
	@echo 'Building target: $@'
	$(CC) -o"$(TARGET).elf" $(OBJS) $(LDFLAGS)
	@echo 'Finished building target: $@'

clean:
	-$(RM) $(OBJS) $(DEPS) $(TARGET).elf

#include $(PAPARAZZI_SRC)/conf/Makefile.linux
DRONE ?= $(PAPARAZZI_HOME)/sw/tools/parrot/ardrone2.py

SUB_DIR = paparazzi

HOST ?= 192.168.1.1

 # Program the device and start it.
upload_program: $(TARGET).elf
	#$(Q)$(DRONE) --host=$(HOST) upload_file $(TARGET).elf $(SUB_DIR)
	$(Q)$(DRONE) --host=$(HOST) insmod cdc-acm.ko
	$(Q)$(DRONE) --host=$(HOST) upload_file_and_run $(TARGET).elf $(SUB_DIR)
	$(Q)$(DRONE) --host=$(HOST) status
	# --host=$(HOST)

endif

.PHONY: all clean upload_program debug
