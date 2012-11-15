################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/lm3s/assert.o \
../sw/ext/libopencm3/lib/lm3s/gpio.o \
../sw/ext/libopencm3/lib/lm3s/nvic.o \
../sw/ext/libopencm3/lib/lm3s/scb.o \
../sw/ext/libopencm3/lib/lm3s/systick.o \
../sw/ext/libopencm3/lib/lm3s/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/lm3s/gpio.c \
../sw/ext/libopencm3/lib/lm3s/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/lm3s/gpio.o \
./sw/ext/libopencm3/lib/lm3s/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/lm3s/gpio.d \
./sw/ext/libopencm3/lib/lm3s/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/lm3s/%.o: ../sw/ext/libopencm3/lib/lm3s/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


