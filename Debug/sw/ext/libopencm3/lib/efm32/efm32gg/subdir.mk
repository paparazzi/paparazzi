################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/efm32/efm32gg/assert.o \
../sw/ext/libopencm3/lib/efm32/efm32gg/nvic.o \
../sw/ext/libopencm3/lib/efm32/efm32gg/scb.o \
../sw/ext/libopencm3/lib/efm32/efm32gg/systick.o \
../sw/ext/libopencm3/lib/efm32/efm32gg/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/efm32/efm32gg/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/efm32/efm32gg/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/efm32/efm32gg/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/efm32/efm32gg/%.o: ../sw/ext/libopencm3/lib/efm32/efm32gg/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


