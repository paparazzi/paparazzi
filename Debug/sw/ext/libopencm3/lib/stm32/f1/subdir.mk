################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/stm32/f1/adc.o \
../sw/ext/libopencm3/lib/stm32/f1/assert.o \
../sw/ext/libopencm3/lib/stm32/f1/can.o \
../sw/ext/libopencm3/lib/stm32/f1/crc.o \
../sw/ext/libopencm3/lib/stm32/f1/dac.o \
../sw/ext/libopencm3/lib/stm32/f1/desig.o \
../sw/ext/libopencm3/lib/stm32/f1/dma.o \
../sw/ext/libopencm3/lib/stm32/f1/ethernet.o \
../sw/ext/libopencm3/lib/stm32/f1/exti.o \
../sw/ext/libopencm3/lib/stm32/f1/flash.o \
../sw/ext/libopencm3/lib/stm32/f1/gpio.o \
../sw/ext/libopencm3/lib/stm32/f1/i2c.o \
../sw/ext/libopencm3/lib/stm32/f1/iwdg.o \
../sw/ext/libopencm3/lib/stm32/f1/nvic.o \
../sw/ext/libopencm3/lib/stm32/f1/pwr.o \
../sw/ext/libopencm3/lib/stm32/f1/rcc.o \
../sw/ext/libopencm3/lib/stm32/f1/rtc.o \
../sw/ext/libopencm3/lib/stm32/f1/scb.o \
../sw/ext/libopencm3/lib/stm32/f1/spi.o \
../sw/ext/libopencm3/lib/stm32/f1/systick.o \
../sw/ext/libopencm3/lib/stm32/f1/timer.o \
../sw/ext/libopencm3/lib/stm32/f1/usart.o \
../sw/ext/libopencm3/lib/stm32/f1/usb.o \
../sw/ext/libopencm3/lib/stm32/f1/usb_control.o \
../sw/ext/libopencm3/lib/stm32/f1/usb_f103.o \
../sw/ext/libopencm3/lib/stm32/f1/usb_f107.o \
../sw/ext/libopencm3/lib/stm32/f1/usb_fx07_common.o \
../sw/ext/libopencm3/lib/stm32/f1/usb_standard.o \
../sw/ext/libopencm3/lib/stm32/f1/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/stm32/f1/adc.c \
../sw/ext/libopencm3/lib/stm32/f1/dma.c \
../sw/ext/libopencm3/lib/stm32/f1/ethernet.c \
../sw/ext/libopencm3/lib/stm32/f1/exti.c \
../sw/ext/libopencm3/lib/stm32/f1/flash.c \
../sw/ext/libopencm3/lib/stm32/f1/gpio.c \
../sw/ext/libopencm3/lib/stm32/f1/pwr.c \
../sw/ext/libopencm3/lib/stm32/f1/rcc.c \
../sw/ext/libopencm3/lib/stm32/f1/rtc.c \
../sw/ext/libopencm3/lib/stm32/f1/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/stm32/f1/adc.o \
./sw/ext/libopencm3/lib/stm32/f1/dma.o \
./sw/ext/libopencm3/lib/stm32/f1/ethernet.o \
./sw/ext/libopencm3/lib/stm32/f1/exti.o \
./sw/ext/libopencm3/lib/stm32/f1/flash.o \
./sw/ext/libopencm3/lib/stm32/f1/gpio.o \
./sw/ext/libopencm3/lib/stm32/f1/pwr.o \
./sw/ext/libopencm3/lib/stm32/f1/rcc.o \
./sw/ext/libopencm3/lib/stm32/f1/rtc.o \
./sw/ext/libopencm3/lib/stm32/f1/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/stm32/f1/adc.d \
./sw/ext/libopencm3/lib/stm32/f1/dma.d \
./sw/ext/libopencm3/lib/stm32/f1/ethernet.d \
./sw/ext/libopencm3/lib/stm32/f1/exti.d \
./sw/ext/libopencm3/lib/stm32/f1/flash.d \
./sw/ext/libopencm3/lib/stm32/f1/gpio.d \
./sw/ext/libopencm3/lib/stm32/f1/pwr.d \
./sw/ext/libopencm3/lib/stm32/f1/rcc.d \
./sw/ext/libopencm3/lib/stm32/f1/rtc.d \
./sw/ext/libopencm3/lib/stm32/f1/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/stm32/f1/%.o: ../sw/ext/libopencm3/lib/stm32/f1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


