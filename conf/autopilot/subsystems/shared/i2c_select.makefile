#generic i2c driver

# TODO: this file was created to be able to select different driver files. Once 1 driver is selected as the best others can be removed including this file

$(TARGET).srcs += mcu_periph/i2c.c
ifeq ($(ARCH), stm32)
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
#$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.rewritten.c
else
$(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
endif

