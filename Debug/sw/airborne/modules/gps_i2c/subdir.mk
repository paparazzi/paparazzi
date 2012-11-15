################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/gps_i2c/gps_i2c.c 

OBJS += \
./sw/airborne/modules/gps_i2c/gps_i2c.o 

C_DEPS += \
./sw/airborne/modules/gps_i2c/gps_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/gps_i2c/%.o: ../sw/airborne/modules/gps_i2c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


