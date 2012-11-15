################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/efm32/efm32g/assert.o \
../sw/ext/libopencm3/lib/efm32/efm32g/nvic.o \
../sw/ext/libopencm3/lib/efm32/efm32g/scb.o \
../sw/ext/libopencm3/lib/efm32/efm32g/systick.o \
../sw/ext/libopencm3/lib/efm32/efm32g/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/efm32/efm32g/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/efm32/efm32g/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/efm32/efm32g/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/efm32/efm32g/%.o: ../sw/ext/libopencm3/lib/efm32/efm32g/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


