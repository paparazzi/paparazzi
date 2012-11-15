################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/lib/cm3/assert.c \
../sw/ext/libopencm3/lib/cm3/nvic.c \
../sw/ext/libopencm3/lib/cm3/scb.c \
../sw/ext/libopencm3/lib/cm3/systick.c \
../sw/ext/libopencm3/lib/cm3/vector.c 

OBJS += \
./sw/ext/libopencm3/lib/cm3/assert.o \
./sw/ext/libopencm3/lib/cm3/nvic.o \
./sw/ext/libopencm3/lib/cm3/scb.o \
./sw/ext/libopencm3/lib/cm3/systick.o \
./sw/ext/libopencm3/lib/cm3/vector.o 

C_DEPS += \
./sw/ext/libopencm3/lib/cm3/assert.d \
./sw/ext/libopencm3/lib/cm3/nvic.d \
./sw/ext/libopencm3/lib/cm3/scb.d \
./sw/ext/libopencm3/lib/cm3/systick.d \
./sw/ext/libopencm3/lib/cm3/vector.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/cm3/%.o: ../sw/ext/libopencm3/lib/cm3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


