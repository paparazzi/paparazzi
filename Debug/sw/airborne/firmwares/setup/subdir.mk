################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/setup/setup_actuators.c 

OBJS += \
./sw/airborne/firmwares/setup/setup_actuators.o 

C_DEPS += \
./sw/airborne/firmwares/setup/setup_actuators.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/setup/%.o: ../sw/airborne/firmwares/setup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


