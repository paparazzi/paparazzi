################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/stm32/f2/assert.o \
../sw/ext/libopencm3/lib/stm32/f2/exti2.o \
../sw/ext/libopencm3/lib/stm32/f2/flash.o \
../sw/ext/libopencm3/lib/stm32/f2/gpio2.o \
../sw/ext/libopencm3/lib/stm32/f2/i2c.o \
../sw/ext/libopencm3/lib/stm32/f2/nvic.o \
../sw/ext/libopencm3/lib/stm32/f2/rcc.o \
../sw/ext/libopencm3/lib/stm32/f2/scb.o \
../sw/ext/libopencm3/lib/stm32/f2/spi.o \
../sw/ext/libopencm3/lib/stm32/f2/systick.o \
../sw/ext/libopencm3/lib/stm32/f2/timer.o \
../sw/ext/libopencm3/lib/stm32/f2/usart.o \
../sw/ext/libopencm3/lib/stm32/f2/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/stm32/f2/flash.c \
../sw/ext/libopencm3/lib/stm32/f2/rcc.c \
../sw/ext/libopencm3/lib/stm32/f2/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/stm32/f2/flash.o \
./sw/ext/libopencm3/lib/stm32/f2/rcc.o \
./sw/ext/libopencm3/lib/stm32/f2/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/stm32/f2/flash.d \
./sw/ext/libopencm3/lib/stm32/f2/rcc.d \
./sw/ext/libopencm3/lib/stm32/f2/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/stm32/f2/%.o: ../sw/ext/libopencm3/lib/stm32/f2/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


