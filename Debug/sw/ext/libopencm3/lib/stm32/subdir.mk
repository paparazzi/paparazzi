################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/lib/stm32/can.c \
../sw/ext/libopencm3/lib/stm32/crc.c \
../sw/ext/libopencm3/lib/stm32/dac.c \
../sw/ext/libopencm3/lib/stm32/desig.c \
../sw/ext/libopencm3/lib/stm32/exti2.c \
../sw/ext/libopencm3/lib/stm32/gpio2.c \
../sw/ext/libopencm3/lib/stm32/i2c.c \
../sw/ext/libopencm3/lib/stm32/iwdg.c \
../sw/ext/libopencm3/lib/stm32/spi.c \
../sw/ext/libopencm3/lib/stm32/timer.c \
../sw/ext/libopencm3/lib/stm32/usart.c 

OBJS += \
./sw/ext/libopencm3/lib/stm32/can.o \
./sw/ext/libopencm3/lib/stm32/crc.o \
./sw/ext/libopencm3/lib/stm32/dac.o \
./sw/ext/libopencm3/lib/stm32/desig.o \
./sw/ext/libopencm3/lib/stm32/exti2.o \
./sw/ext/libopencm3/lib/stm32/gpio2.o \
./sw/ext/libopencm3/lib/stm32/i2c.o \
./sw/ext/libopencm3/lib/stm32/iwdg.o \
./sw/ext/libopencm3/lib/stm32/spi.o \
./sw/ext/libopencm3/lib/stm32/timer.o \
./sw/ext/libopencm3/lib/stm32/usart.o 

C_DEPS += \
./sw/ext/libopencm3/lib/stm32/can.d \
./sw/ext/libopencm3/lib/stm32/crc.d \
./sw/ext/libopencm3/lib/stm32/dac.d \
./sw/ext/libopencm3/lib/stm32/desig.d \
./sw/ext/libopencm3/lib/stm32/exti2.d \
./sw/ext/libopencm3/lib/stm32/gpio2.d \
./sw/ext/libopencm3/lib/stm32/i2c.d \
./sw/ext/libopencm3/lib/stm32/iwdg.d \
./sw/ext/libopencm3/lib/stm32/spi.d \
./sw/ext/libopencm3/lib/stm32/timer.d \
./sw/ext/libopencm3/lib/stm32/usart.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/stm32/%.o: ../sw/ext/libopencm3/lib/stm32/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


