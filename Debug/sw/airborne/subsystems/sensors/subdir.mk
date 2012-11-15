################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/sensors/infrared.c \
../sw/airborne/subsystems/sensors/infrared_adc.c \
../sw/airborne/subsystems/sensors/infrared_i2c.c 

OBJS += \
./sw/airborne/subsystems/sensors/infrared.o \
./sw/airborne/subsystems/sensors/infrared_adc.o \
./sw/airborne/subsystems/sensors/infrared_i2c.o 

C_DEPS += \
./sw/airborne/subsystems/sensors/infrared.d \
./sw/airborne/subsystems/sensors/infrared_adc.d \
./sw/airborne/subsystems/sensors/infrared_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/sensors/%.o: ../sw/airborne/subsystems/sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


