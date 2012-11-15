################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/i2c_stts75_sensor.c \
../sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/stts75.c 

OBJS += \
./sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/i2c_stts75_sensor.o \
./sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/stts75.o 

C_DEPS += \
./sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/i2c_stts75_sensor.d \
./sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/stts75.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/%.o: ../sw/ext/libopencm3/examples/stm32/f1/other/i2c_stts75_sensor/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


