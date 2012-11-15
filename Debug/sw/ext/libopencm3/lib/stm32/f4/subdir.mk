################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/stm32/f4/adc.o \
../sw/ext/libopencm3/lib/stm32/f4/assert.o \
../sw/ext/libopencm3/lib/stm32/f4/exti2.o \
../sw/ext/libopencm3/lib/stm32/f4/flash.o \
../sw/ext/libopencm3/lib/stm32/f4/gpio2.o \
../sw/ext/libopencm3/lib/stm32/f4/i2c.o \
../sw/ext/libopencm3/lib/stm32/f4/nvic.o \
../sw/ext/libopencm3/lib/stm32/f4/pwr.o \
../sw/ext/libopencm3/lib/stm32/f4/rcc.o \
../sw/ext/libopencm3/lib/stm32/f4/scb.o \
../sw/ext/libopencm3/lib/stm32/f4/spi.o \
../sw/ext/libopencm3/lib/stm32/f4/systick.o \
../sw/ext/libopencm3/lib/stm32/f4/timer.o \
../sw/ext/libopencm3/lib/stm32/f4/usart.o \
../sw/ext/libopencm3/lib/stm32/f4/usb.o \
../sw/ext/libopencm3/lib/stm32/f4/usb_control.o \
../sw/ext/libopencm3/lib/stm32/f4/usb_f107.o \
../sw/ext/libopencm3/lib/stm32/f4/usb_f207.o \
../sw/ext/libopencm3/lib/stm32/f4/usb_fx07_common.o \
../sw/ext/libopencm3/lib/stm32/f4/usb_standard.o \
../sw/ext/libopencm3/lib/stm32/f4/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/stm32/f4/adc.c \
../sw/ext/libopencm3/lib/stm32/f4/dma.c \
../sw/ext/libopencm3/lib/stm32/f4/flash.c \
../sw/ext/libopencm3/lib/stm32/f4/pwr.c \
../sw/ext/libopencm3/lib/stm32/f4/rcc.c \
../sw/ext/libopencm3/lib/stm32/f4/vector_chipset.c \
../sw/ext/libopencm3/lib/stm32/f4/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/stm32/f4/adc.o \
./sw/ext/libopencm3/lib/stm32/f4/dma.o \
./sw/ext/libopencm3/lib/stm32/f4/flash.o \
./sw/ext/libopencm3/lib/stm32/f4/pwr.o \
./sw/ext/libopencm3/lib/stm32/f4/rcc.o \
./sw/ext/libopencm3/lib/stm32/f4/vector_chipset.o \
./sw/ext/libopencm3/lib/stm32/f4/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/stm32/f4/adc.d \
./sw/ext/libopencm3/lib/stm32/f4/dma.d \
./sw/ext/libopencm3/lib/stm32/f4/flash.d \
./sw/ext/libopencm3/lib/stm32/f4/pwr.d \
./sw/ext/libopencm3/lib/stm32/f4/rcc.d \
./sw/ext/libopencm3/lib/stm32/f4/vector_chipset.d \
./sw/ext/libopencm3/lib/stm32/f4/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/stm32/f4/%.o: ../sw/ext/libopencm3/lib/stm32/f4/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


