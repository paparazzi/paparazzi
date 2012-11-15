################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/lpc13xx/assert.o \
../sw/ext/libopencm3/lib/lpc13xx/gpio.o \
../sw/ext/libopencm3/lib/lpc13xx/nvic.o \
../sw/ext/libopencm3/lib/lpc13xx/scb.o \
../sw/ext/libopencm3/lib/lpc13xx/systick.o \
../sw/ext/libopencm3/lib/lpc13xx/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/lpc13xx/gpio.c \
../sw/ext/libopencm3/lib/lpc13xx/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/lpc13xx/gpio.o \
./sw/ext/libopencm3/lib/lpc13xx/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/lpc13xx/gpio.d \
./sw/ext/libopencm3/lib/lpc13xx/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/lpc13xx/%.o: ../sw/ext/libopencm3/lib/lpc13xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


