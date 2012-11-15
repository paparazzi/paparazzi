################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/lpc17xx/assert.o \
../sw/ext/libopencm3/lib/lpc17xx/gpio.o \
../sw/ext/libopencm3/lib/lpc17xx/nvic.o \
../sw/ext/libopencm3/lib/lpc17xx/scb.o \
../sw/ext/libopencm3/lib/lpc17xx/systick.o \
../sw/ext/libopencm3/lib/lpc17xx/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/lpc17xx/gpio.c \
../sw/ext/libopencm3/lib/lpc17xx/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/lpc17xx/gpio.o \
./sw/ext/libopencm3/lib/lpc17xx/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/lpc17xx/gpio.d \
./sw/ext/libopencm3/lib/lpc17xx/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/lpc17xx/%.o: ../sw/ext/libopencm3/lib/lpc17xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


